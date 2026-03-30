// ===== src/presentation/qt/widgets/SlotGridWidget.hpp =====
// Исправлено:
//   - SlotItem вынесен из WorkerViewModel в отдельный заголовок SlotItem.hpp
//     (разрыв циклической зависимости SlotGridWidget ↔ WorkerViewModel)
//   - updateSlots принимает QVector<SlotCellData> вместо QVector<SlotItem>
//     (SlotGridWidget не должен знать о WorkerViewModel)
#pragma once

#include "domain/entities/Slot.hpp"

#include <QWidget>
#include <QVector>
#include <QColor>
#include <QGridLayout>
#include <QPushButton>
#include <QString>

/// Данные одной ячейки сетки — не зависит от ViewModel
struct SlotCellData {
    int                          slotIndex   = 0;
    smartcart::domain::SlotState state       = smartcart::domain::SlotState::Free;
    bool                         highlighted = false;
    QColor                       color       { 128, 128, 128 };
    QString                      barcode;
};

/// Виджет сетки 6×4 для визуализации 24 слотов модуля.
/// Каждая ячейка: номер слота + цветовой индикатор + tooltip со штрихкодом.
class SlotGridWidget : public QWidget {
    Q_OBJECT

public:
    static constexpr int kRows  = 4;
    static constexpr int kCols  = 6;
    static constexpr int kSlots = kRows * kCols; // 24

    explicit SlotGridWidget(QWidget* parent = nullptr);

    /// Обновить все слоты разом (вызывается из WorkerViewModel::slotsUpdated).
    void updateSlots(const QVector<SlotCellData>& items);

    /// Обновить один слот.
    void updateSlot(int slotIndex,
                    smartcart::domain::SlotState state,
                    bool highlighted,
                    QColor color = {},
                    const QString& barcode = {});

signals:
    /// Оператор кликнул по ячейке слота.
    void slotClicked(int slotIndex);

private:
    QGridLayout*          layout_ = nullptr;
    QVector<QPushButton*> cells_;  // индекс = slotIndex - 1

    QPushButton* createCell(int slotIndex);

    void applyStyle(QPushButton* cell,
                    smartcart::domain::SlotState state,
                    bool highlighted,
                    QColor color);

    static QColor colorForState(smartcart::domain::SlotState state);
};
