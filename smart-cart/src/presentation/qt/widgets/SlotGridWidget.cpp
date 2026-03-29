#include "SlotGridWidget.hpp"
#include <QGridLayout>
#include <QPushButton>

SlotGridWidget::SlotGridWidget(QWidget* parent) : QWidget(parent) {
    auto* grid = new QGridLayout(this);
    grid->setSpacing(8);

    slots_.reserve(24);
    for (int i = 0; i < 24; ++i) {
        const int slot = i + 1;
        auto* b = new QPushButton(QString("S%1").arg(slot, 2, 10, QChar('0')), this);
        b->setMinimumSize(90, 64);
        b->setProperty("occupied", false);
        connect(b, &QPushButton::clicked, this, [this, slot]() { emit slotClicked(slot); });
        slots_.push_back(b);

        const int row = i / 6;
        const int col = i % 6;
        grid->addWidget(b, row, col);
    }

    for (int i = 1; i <= 24; ++i) repaintSlot(i);
}

void SlotGridWidget::setSlotOccupied(int slotIndex, bool occupied) {
    if (slotIndex < 1 || slotIndex > 24) return;
    slots_[slotIndex - 1]->setProperty("occupied", occupied);
    repaintSlot(slotIndex);
}

void SlotGridWidget::setTargetSlot(int slotIndex) {
    if (slotIndex < 0 || slotIndex > 24) return;
    target_ = slotIndex;
    for (int i = 1; i <= 24; ++i) repaintSlot(i);
}

void SlotGridWidget::repaintSlot(int idx1) {
    auto* b = slots_[idx1 - 1];
    const bool occupied = b->property("occupied").toBool();
    const bool target = (idx1 == target_);

    QString style = "font-weight:700; border-radius:8px; border:2px solid #445;";
    if (target) style += "background:#F39C12; color:white;";       // target
    else if (occupied) style += "background:#2ECC71; color:#102A12;"; // occupied
    else style += "background:#E74C3C; color:white;";              // free

    b->setStyleSheet(style);
}
