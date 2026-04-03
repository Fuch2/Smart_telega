#pragma once

#include "application/state/AppStateMachine.hpp"
#include "application/ports/IReelRepository.hpp"
#include "application/ports/IOperationRepository.hpp"
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
        smartcart::application::ports::IReelRepository&      reelRepo,
        smartcart::application::ports::IOperationRepository& opRepo,
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
    void operationStateChanged(QString state, QString message);
    void errorOccurred(QString message);

public Q_SLOTS:
    void onBarcodeScanned(const QString& barcode);
    void onSlotPhysicalChange(int slotIndex, bool occupied);
    void cancelCurrentOperation();
    void reload();

private Q_SLOTS:
    void onStateChanged(smartcart::application::AppState newState);
    void onSlotHighlighted(int slotIndex, QColor color);
    void onOperationFinished(int                              operationId,
                             smartcart::domain::OperationStatus status);
    void onError(smartcart::domain::ErrorCode code, QString message);

private:
    smartcart::application::ports::IReelRepository&      reelRepo_;
    smartcart::application::ports::IOperationRepository& opRepo_;
    smartcart::application::AppStateMachine&             stateMachine_;

    QVector<SlotCellData> slots_;

    SlotCellData*  findSlot(int slotIndex);
    void           rebuildSlots();
    static QString errorMessage(smartcart::domain::ErrorCode code);
};
