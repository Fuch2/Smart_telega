// ===== src/application/state/AppStateMachine.hpp =====
// Исправлено: AppState и ErrorCode зарегистрированы как Qt мета-типы
#pragma once

#include "application/services/StartupService.hpp"
#include "application/services/AddReelService.hpp"
#include "application/services/ReplaceReelService.hpp"
#include "application/services/RecoveryService.hpp"
#include "domain/entities/Operation.hpp"
#include "domain/errors/ErrorCode.hpp"

#include <QObject>
#include <QColor>
#include <QString>

// Регистрация enum для Qt сигналов/слотов
Q_DECLARE_METATYPE(smartcart::domain::ErrorCode)
Q_DECLARE_METATYPE(smartcart::domain::OperationType)
Q_DECLARE_METATYPE(smartcart::domain::OperationStatus)

namespace smartcart::application {

enum class AppState {
    Idle,
    Initializing,
    Ready,
    Operating,
    Error
};

} // namespace smartcart::application

Q_DECLARE_METATYPE(smartcart::application::AppState)

namespace smartcart::application {

class AppStateMachine : public QObject {
    Q_OBJECT

public:
    explicit AppStateMachine(
        services::StartupService&     startupSvc,
        services::AddReelService&     addReelSvc,
        services::ReplaceReelService& replaceReelSvc,
        services::RecoveryService&    recoverySvc,
        QObject*                      parent = nullptr
    );

    AppState currentState() const noexcept { return state_; }

public slots:
    void startup();
    void scanBarcode(const QString& barcode);
    void cancelCurrentOperation();
    void recover();

signals:
    void stateChanged(smartcart::application::AppState newState);
    void operationStarted(int operationId, smartcart::domain::OperationType type);
    void operationFinished(int operationId, smartcart::domain::OperationStatus status);
    void slotHighlighted(int slotIndex, QColor color);
    void errorOccurred(smartcart::domain::ErrorCode code, QString message);

private:
    void transition(AppState newState);

    services::StartupService&     startupSvc_;
    services::AddReelService&     addReelSvc_;
    services::ReplaceReelService& replaceReelSvc_;
    services::RecoveryService&    recoverySvc_;

    AppState state_ = AppState::Idle;
};

} // namespace smartcart::application
