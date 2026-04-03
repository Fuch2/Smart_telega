#pragma once

#include "domain/entities/Slot.hpp"

#include <QColor>
#include <QString>

/// Данные одной ячейки сетки слотов.
/// Не зависит ни от QWidget, ни от ViewModel —
/// используется как чистый DTO между WorkerViewModel и SlotGridWidget.
struct SlotCellData {
    int                          slotIndex   = 0;
    smartcart::domain::SlotState state
        = smartcart::domain::SlotState::Free;
    bool                         highlighted = false;
    QColor                       color       { 128, 128, 128 };
    QString                      barcode;
};
