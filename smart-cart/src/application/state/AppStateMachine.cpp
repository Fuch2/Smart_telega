// ===== src/application/state/AppStateMachine.cpp =====
#include "application/state/AppStateMachine.hpp"

#include <QMetaObject>
#include <QMetaType>

namespace smartcart::application {

using namespace smartcart::domain;

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

AppStateMachine::AppStateMachine(
    services::StartupService&     startupSvc,
    services::AddReelService&     addReelSvc,
    services::ReplaceReelService& replaceReelSvc,
    services::RecoveryService&    recoverySvc,
    ports::IReelRepository&       reelRepo,
    QObject*                      parent)
    : QObject(parent)
    , startupSvc_(startupSvc)
    , addReelSvc_(addReelSvc)
    , replaceReelSvc_(replaceReelSvc)
    , recoverySvc_(recoverySvc)
    , reelRepo_(reelRepo)
{}

void AppStateMachine::transition(AppState newState) {
    if (state_ == newState) return;
    state_ = newState;
    emit stateChanged(newState);
}

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

    const auto& slotList = std::get<std::vector<Slot>>(startupResult);
    for (const auto& slot : slotList) {
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

void AppStateMachine::scanBarcode(const QString& barcode) {
    if (state_ != AppState::Ready) return;

    transition(AppState::Operating);

    const std::string barcodeStr = barcode.toStdString();

    auto toQColor = [this](int slotIdx, services::RgbColor c) {
        emit slotHighlighted(slotIdx, QColor(c.r, c.g, c.b));
    };

    auto onComplete = [this](int opId, OperationStatus status) {
        emit operationFinished(opId, status);
        transition(AppState::Ready);
    };

    replaceReelSvc_.setCompletionCallback(onComplete);
    replaceReelSvc_.setSlotHighlightCallback(toQColor);
    replaceReelSvc_.setErrorCallback(
        [this, barcodeStr, toQColor, onComplete]
        (ErrorCode code, std::string /*msg*/) mutable
        {
            if (code != ErrorCode::ReelNotFound) {
                emit errorOccurred(code, QString::fromStdString(
                    std::string(toString(code))));
                transition(AppState::Ready);
                return;
            }

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
            } else {
                transition(AppState::Ready);
            }
        }
    );

    const int replaceOpId = replaceReelSvc_.start(barcodeStr);
    if (replaceOpId >= 0) {
        emit operationStarted(replaceOpId, OperationType::ReplaceReel);
    } else {
        // start() вернул -1 синхронно (невалидный баркод отклонён до потока)
        // onError_ будет вызван из start() — он сам вызовет transition(Ready)
        // через addReelSvc_ ветку. Но если onError_ не вызван (баркод невалиден
        // на уровне ReplaceReelService до запуска потока) — возвращаем Ready.
        transition(AppState::Ready);
    }
}

void AppStateMachine::cancelCurrentOperation() {
    if (state_ != AppState::Operating) return;
    addReelSvc_.cancel();
    replaceReelSvc_.cancel();
    transition(AppState::Ready);
}

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
