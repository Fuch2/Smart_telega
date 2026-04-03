// ===== src/presentation/qt/widgets/SlotGridWidget.cpp =====
// Исправлено:
//   - include path относительный
//   - updateSlots принимает QVector<SlotCellData>
#include "SlotGridWidget.hpp"

#include <QToolTip>
#include <QSizePolicy>
#include <QFont>

using namespace smartcart::domain;

SlotGridWidget::SlotGridWidget(QWidget* parent)
    : QWidget(parent)
{
    layout_ = new QGridLayout(this);
    layout_->setSpacing(10);
    layout_->setContentsMargins(4, 4, 4, 4);

    cells_.resize(kSlots);

    for (int i = 0; i < kSlots; ++i) {
        const int row     = i / kCols;
        const int col     = i % kCols;
        const int slotIdx = i + 1;

        QPushButton* cell = createCell(slotIdx);
        cells_[i] = cell;
        layout_->addWidget(cell, row, col);
        layout_->setRowStretch(row, 1);
        layout_->setColumnStretch(col, 1);
    }

    setLayout(layout_);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

QPushButton* SlotGridWidget::createCell(int slotIndex) {
    auto* btn = new QPushButton(
        QString::fromUtf8("Слот\n%1").arg(slotIndex), this);
    btn->setMinimumSize(118, 96);
    btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    btn->setCheckable(false);
    btn->setFont(QFont("Segoe UI", 16, QFont::Bold));

    applyStyle(btn, SlotState::Free, false, colorForState(SlotState::Free));

    connect(btn, &QPushButton::clicked, this, [this, slotIndex]() {
        emit slotClicked(slotIndex);
    });

    return btn;
}

void SlotGridWidget::updateSlots(const QVector<SlotCellData>& items) {
    for (const auto& item : items) {
        if (item.slotIndex < 1 || item.slotIndex > kSlots) continue;
        updateSlot(item.slotIndex, item.state,
                   item.highlighted, item.color, item.barcode);
    }
}

void SlotGridWidget::updateSlot(int slotIndex,
                                SlotState state,
                                bool highlighted,
                                QColor color,
                                const QString& barcode)
{
    if (slotIndex < 1 || slotIndex > kSlots) return;

    QPushButton* cell = cells_[slotIndex - 1];
    if (!cell) return;

    const QColor effectiveColor = color.isValid() ? color : colorForState(state);
    applyStyle(cell, state, highlighted, effectiveColor);

    if (!barcode.isEmpty()) {
        cell->setToolTip(
            QString::fromUtf8("Слот %1\n%2").arg(slotIndex).arg(barcode));
    } else {
        cell->setToolTip(
            QString::fromUtf8("Слот %1 — свободен").arg(slotIndex));
    }
}

void SlotGridWidget::applyStyle(QPushButton* cell,
                                SlotState state,
                                bool highlighted,
                                QColor color)
{
    Q_UNUSED(state)

    const QString border = highlighted
        ? "border: 3px solid #2E7D4F;"
        : "border: 1px solid #BFCBC2;";

    const QString bg = QString("background-color: rgb(%1,%2,%3);")
        .arg(color.red())
        .arg(color.green())
        .arg(color.blue());

    const int luminance =
        (color.red() * 299 + color.green() * 587 + color.blue() * 114) / 1000;
    const QString textColor =
        (luminance < 128) ? "color: #FFFFFF;" : "color: #000000;";

    cell->setStyleSheet(QString(
        "QPushButton { %1 %2 %3 border-radius: 8px; padding: 10px; "
        "font-weight: bold; }"
        "QPushButton:hover { border-color: #2F8F57; }"
    ).arg(bg, border, textColor));
}

QColor SlotGridWidget::colorForState(SlotState state) {
    switch (state) {
        case SlotState::Free:     return QColor(222, 229, 224);
        case SlotState::Occupied: return QColor(47,  143,  87);
        case SlotState::Reserved: return QColor(225, 177,  79);
        case SlotState::Error:    return QColor(184,  92,  92);
    }
    return QColor(222, 229, 224);
}
