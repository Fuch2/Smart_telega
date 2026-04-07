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
    services::Stm32PollingService& pollingSvc,
    QObject*                      parent)
    : QObject(parent)
    , startupSvc_(startupSvc)
    , addReelSvc_(addReelSvc)
    , replaceReelSvc_(replaceReelSvc)
    , recoverySvc_(recoverySvc)
    , reelRepo_(reelRepo)
    , pollingSvc_(pollingSvc)
{}

services::Stm32ConnectionStatus AppStateMachine::stm32ConnectionStatus() const {
    return pollingSvc_.connectionStatus();
}

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
    const QString trimmedBarcode = barcode.trimmed();
    if (trimmedBarcode.isEmpty()) {
        emit errorOccurred(ErrorCode::InvalidBarcode,
                           "Пустой штрихкод проигнорирован");
        return;
    }

    const auto result = pollingSvc_.recordBarcodeScan(trimmedBarcode.toStdString());
    if (!result) {
        emit errorOccurred(result.error,
                           QString::fromStdString(result.message));
        return;
    }

    emit slotHighlighted(result.targetSlot, QColor(0, 255, 0));
    emit operationStarted(result.operationId, OperationType::AddReel);
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
