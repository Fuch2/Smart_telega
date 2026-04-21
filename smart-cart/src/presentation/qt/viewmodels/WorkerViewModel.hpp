#pragma once

#include "application/state/AppStateMachine.hpp"
#include "application/ports/IReelRepository.hpp"
#include "application/ports/IOperationRepository.hpp"
#include "application/ports/IOrderRepository.hpp"
#include "application/ports/IWorkflowRepository.hpp"
#include "application/ports/IModuleRepository.hpp"
#include "application/services/OrderImportService.hpp"
#include "application/services/WorkflowService.hpp"
#include "domain/entities/CartWorkflow.hpp"
#include "domain/entities/Slot.hpp"
#include "domain/entities/Operation.hpp"
#include "domain/errors/ErrorCode.hpp"
#include "presentation/qt/SlotCellData.hpp"  // ← только DTO, без QWidget

#include <QColor>
#include <QObject>
#include <QString>
#include <QVector>

class WorkerViewModel : public QObject {
    Q_OBJECT

public:
    explicit WorkerViewModel(
        smartcart::application::ports::IModuleRepository&    moduleRepo,
        smartcart::application::ports::IReelRepository&      reelRepo,
        smartcart::application::ports::IOperationRepository& opRepo,
        smartcart::application::ports::IOrderRepository&     orderRepo,
        smartcart::application::ports::IWorkflowRepository&  workflowRepo,
        smartcart::application::services::OrderImportService& orderImportSvc,
        smartcart::application::services::WorkflowService& workflowSvc,
        smartcart::application::AppStateMachine&             stateMachine,
        QObject*                                             parent = nullptr
    );

    // slotItems() — не slots() во избежание конфликта с Qt-макросом
    const QVector<SlotCellData>& slotItems()  const noexcept { return slots_; }
    QString                      stateLabel() const;

    smartcart::application::AppStateMachine& stateMachineRef() noexcept {
        return stateMachine_;
    }

signals:
    void slotsUpdated(QVector<SlotCellData> slots);
    void workflowUpdated(QString workflow,
                         QString order,
                         QString checklist,
                         QString progress,
                         bool showStartPage);
    void workflowControlsUpdated(QString stateKey);
    void stm32StatusUpdated(QString status);
    void activeModuleUpdated(QString moduleSummary);
    void activeModuleAvailabilityChanged(bool online);
    void operationStateChanged(QString state, QString message);
    void errorOccurred(QString message);

public Q_SLOTS:
    void submitBarcode(const QString& barcode);
    void onBarcodeScanned(const QString& barcode);
    void onSlotPhysicalChange(int slotIndex, bool occupied);
    void cancelCurrentOperation();
    void importOrderFromFile(const QString& path,
                             bool skipFeederLoading = false);
    void startFeederLoading();
    void completeFeederLoading();
    void markCartArrivedToFeederPrep();
    void startFeederPrep();
    void markFeederPrepCompleted();
    void markCartArrivedToLine();
    void startIssuingToLine();
    void markItemIssued(const QString& barcode);
    void completeIssuing();
    void inspectLeftovers();
    void startReturningLeftovers();
    void markLeftoverReturned(const QString& barcodeOrSlot);
    void reload();

private Q_SLOTS:
    void onStateChanged(smartcart::application::AppState newState);
    void onSlotHighlighted(int slotIndex, QColor color);
    void onOperationStarted(int operationId,
                            smartcart::domain::OperationType type);
    void onOperationFinished(int                              operationId,
                             smartcart::domain::OperationStatus status);
    void onError(smartcart::domain::ErrorCode code, QString message);

private:
    smartcart::application::ports::IModuleRepository&    moduleRepo_;
    smartcart::application::ports::IReelRepository&      reelRepo_;
    smartcart::application::ports::IOperationRepository& opRepo_;
    smartcart::application::ports::IOrderRepository&     orderRepo_;
    smartcart::application::ports::IWorkflowRepository&  workflowRepo_;
    smartcart::application::services::OrderImportService& orderImportSvc_;
    smartcart::application::services::WorkflowService& workflowSvc_;
    smartcart::application::AppStateMachine&             stateMachine_;

    QVector<SlotCellData> slots_;

    SlotCellData*  findSlot(int slotIndex);
    void           rebuildSlots();
    void           rebuildWorkflowSummary();
    void           rebuildStm32Status();
    void           rebuildModuleStatus();
    bool           ensureActiveModuleOnline(const QString& actionLabel);
    bool           isActiveModuleOnline() const;
    void           handleWorkflowResult(
        const smartcart::application::services::WorkflowActionResult& result);
    static QString errorMessage(smartcart::domain::ErrorCode code);
    static QString workflowLabel(smartcart::domain::CartWorkflowState state);
    static QString itemStatusLabel(smartcart::domain::OrderItemStatus status);
    static QString priorityColor(const std::string& priority);
    static QString priorityLabel(const std::string& priority);
    static QString orderTimeText(const smartcart::domain::OrderInfo& order);
};
