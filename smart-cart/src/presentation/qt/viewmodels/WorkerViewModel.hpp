// ===== src/presentation/qt/viewmodels/WorkerViewModel.hpp =====
// Исправлено:
//   - SlotItem теперь использует SlotCellData из SlotGridWidget.hpp
//     (разрыв цикла: WorkerViewModel больше не тянет SlotGridWidget)
//   - slotsUpdated эмитирует QVector<SlotCellData>
//   - убран лишний #include <unordered_map> из .hpp
#pragma once

#include "application/state/AppStateMachine.hpp"
#include "application/ports/IReelRepository.hpp"
#include "application/ports/IOperationRepository.hpp"
#include "domain/entities/Slot.hpp"
#include "domain/entities/Operation.hpp"
#include "domain/errors/ErrorCode.hpp"
#include "presentation/qt/widgets/SlotGridWidget.hpp"  // SlotCellData

#include <QObject>
#include <QString>
#include <QVector>
#include <QColor>

// SlotItem — псевдоним для обратной совместимости внутри presentation слоя
using SlotItem = SlotCellData;

class WorkerViewModel : public QObject {
    Q_OBJECT

public:
    explicit WorkerViewModel(
        smartcart::application::ports::IReelRepository&      reelRepo,
        smartcart::application::ports::IOperationRepository& opRepo,
        smartcart::application::AppStateMachine&             stateMachine,
        QObject*                                             parent = nullptr
    );

    const QVector<SlotCellData>& slots()      const noexcept { return slots_; }
    QString                      stateLabel() const;

    smartcart::application::AppStateMachine& stateMachine() noexcept {
        return stateMachine_;
    }

signals:
    void slotsUpdated(QVector<SlotCellData> slots);
    void operationStateChanged(QString state, QString message);
    void errorOccurred(QString message);

public slots:
    void onBarcodeScanned(const QString& barcode);
    void onSlotPhysicalChange(int slotIndex, bool occupied);
    void cancelCurrentOperation();
    void reload();

private slots:
    void onStateChanged(smartcart::application::AppState newState);
    void onSlotHighlighted(int slotIndex, QColor color);
    void onOperationFinished(int operationId,
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
