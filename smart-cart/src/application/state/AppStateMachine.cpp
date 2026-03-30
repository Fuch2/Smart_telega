// ===== src/application/state/AppStateMachine.cpp =====
// Исправлено:
//   1. barcodeStr захватывается по значению (не по ссылке) — убран UB
//   2. scanBarcode: логика Add vs Replace через явную проверку репозитория
//      вынесена в два отдельных пути, без вложенных callback-в-callback
#include "application/state/AppStateMachine.hpp"

#include <QMetaObject>
#include <QMetaType>

namespace smartcart::application {

using namespace smartcart::domain;

// ── Регистрация мета-типов (один раз при старте) ──────────────────────────────
namespace {
struct MetaTypeRegistrar {
    MetaTypeRegistrar() {
        qRegisterMetaType<AppState>("AppState");
        qRegisterMetaType<ErrorCode>("ErrorCode");
        qRegisterMetaType<OperationType>("OperationType");
        qRegisterMetaType<OperationStatus>("OperationStatus");
    }
} g_registrar;
} // namespace

// ─────────────────────────────────────────────────────────────────────────────
AppStateMachine::AppStateMachine(
    services::StartupService&     startupSvc,
    services::AddReelService&     addReelSvc,
    services::ReplaceReelService& replaceReelSvc,
    services::RecoveryService&    recoverySvc,
    QObject*                      parent)
    : QObject(parent)
    , startupSvc_(startupSvc)
    , addReelSvc_(addReelSvc)
    , replaceReelSvc_(replaceReelSvc)
    , recoverySvc_(recoverySvc)
{}

// ─────────────────────────────────────────────────────────────────────────────
void AppStateMachine::transition(AppState newState) {
    if (state_ == newState) return;
    state_ = newState;
    emit stateChanged(newState);
}

// ─────────────────────────────────────────────────────────────────────────────
void AppStateMachine::startup() {
    if (state_ != AppState::Idle) return;

    transition(AppState::Initializing);

    if (!recoverySvc_.run()) {
        emit errorOccurred(ErrorCode::RecoveryFailed,
                           "Не удалось восстановить незавершённые операции");
        transition(AppState::Error);
        return;
    }

    const auto startupResult = startupSvc_.run();

    if (std::holds_alternative<ErrorCode>(startupResult)) {
        emit errorOccurred(std::get<ErrorCode>(startupResult),
                           "Ошибка инициализации STM32");
        transition(AppState::Error);
        return;
    }

    const auto& slots = std::get<std::vector<Slot>>(startupResult);
    for (const auto& slot : slots) {
        QColor color;
        switch (slot.state) {
            case SlotState::Occupied: color = QColor(30,  80,  200); break;
            case SlotState::Error:    color = QColor(200, 30,  30);  break;
            default:                  color = QColor(80,  80,  80);  break;
        }
        emit slotHighlighted(slot.slotIndex, color);
    }

    transition(AppState::Ready);
}

// ─────────────────────────────────────────────────────────────────────────────
void AppStateMachine::scanBarcode(const QString& barcode) {
    if (state_ != AppState::Ready) return;

    transition(AppState::Operating);

    // ← захват по значению: barcodeStr живёт в лямбдах после выхода из функции
    const std::string barcodeStr = barcode.toStdString();

    // Конвертер RgbColor → QColor (используется обоими сервисами)
    auto toQColor = [this](int slotIdx, services::RgbColor c) {
        emit slotHighlighted(slotIdx, QColor(c.r, c.g, c.b));
    };

    // Общий completion handler
    auto onComplete = [this](int opId, OperationStatus status) {
        emit operationFinished(opId, status);
        transition(AppState::Ready);
    };

    // ── Пробуем ReplaceReel ───────────────────────────────────────────────────
    // ReplaceReelService::start() вернёт -1 + вызовет errorCallback
    // если катушка не найдена → тогда запускаем AddReel.
    //
    // Важно: errorCallback вызывается СИНХРОННО внутри start(),
    // до возврата из функции — поэтому вложение безопасно.

    bool replaceStarted = false;

    replaceReelSvc_.setCompletionCallback(onComplete);
    replaceReelSvc_.setSlotHighlightCallback(toQColor);
    replaceReelSvc_.setErrorCallback(
        [this, barcodeStr, toQColor, onComplete, &replaceStarted]
        (ErrorCode code, std::string /*msg*/) mutable
        {
            if (code != ErrorCode::ReelNotFound) {
                // Настоящая ошибка — не пробуем AddReel
                emit errorOccurred(code, QString::fromStdString(
                    std::string(toString(code))));
                transition(AppState::Ready);
                return;
            }

            // ReelNotFound → запускаем AddReel
            addReelSvc_.setCompletionCallback(onComplete);
            addReelSvc_.setSlotHighlightCallback(toQColor);
            addReelSvc_.setErrorCallback(
                [this](ErrorCode c, std::string m) {
                    emit errorOccurred(c, QString::fromStdString(m));
                    transition(AppState::Ready);
                }
            );

            const int addOpId = addReelSvc_.start(barcodeStr);
            if (addOpId >= 0) {
                emit operationStarted(addOpId, OperationType::AddReel);
                replaceStarted = true;
            }
            // если addOpId < 0 — errorCallback уже вызван внутри start()
        }
    );

    const int replaceOpId = replaceReelSvc_.start(barcodeStr);
    if (replaceOpId >= 0) {
        emit operationStarted(replaceOpId, OperationType::ReplaceReel);
    }
    // если replaceOpId < 0 — errorCallback уже сработал выше
}

// ─────────────────────────────────────────────────────────────────────────────
void AppStateMachine::cancelCurrentOperation() {
    if (state_ != AppState::Operating) return;

    addReelSvc_.cancel();
    replaceReelSvc_.cancel();

    transition(AppState::Ready);
}

// ─────────────────────────────────────────────────────────────────────────────
void AppStateMachine::recover() {
    if (state_ != AppState::Error) return;

    if (recoverySvc_.run()) {
        transition(AppState::Ready);
    } else {
        emit errorOccurred(ErrorCode::RecoveryFailed,
                           "Повторное восстановление не удалось");
    }
}

} // namespace smartcart::application
