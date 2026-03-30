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
    layout_->setSpacing(6);
    layout_->setContentsMargins(8, 8, 8, 8);

    cells_.resize(kSlots);

    for (int i = 0; i < kSlots; ++i) {
        const int row     = i / kCols;
        const int col     = i % kCols;
        const int slotIdx = i + 1;

        QPushButton* cell = createCell(slotIdx);
        cells_[i] = cell;
        layout_->addWidget(cell, row, col);
    }

    setLayout(layout_);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

QPushButton* SlotGridWidget::createCell(int slotIndex) {
    auto* btn = new QPushButton(
        QString::fromUtf8("Слот\n%1").arg(slotIndex), this);
    btn->setFixedSize(90, 70);
    btn->setCheckable(false);
    btn->setFont(QFont("Segoe UI", 9));

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
        ? "border: 3px solid #FFFFFF;"
        : "border: 1px solid #555555;";

    const QString bg = QString("background-color: rgb(%1,%2,%3);")
        .arg(color.red())
        .arg(color.green())
        .arg(color.blue());

    const int luminance =
        (color.red() * 299 + color.green() * 587 + color.blue() * 114) / 1000;
    const QString textColor =
        (luminance < 128) ? "color: #FFFFFF;" : "color: #000000;";

    cell->setStyleSheet(QString(
        "QPushButton { %1 %2 %3 border-radius: 6px; padding: 4px; }"
        "QPushButton:hover { border-color: #AAAAAA; }"
    ).arg(bg, border, textColor));
}

QColor SlotGridWidget::colorForState(SlotState state) {
    switch (state) {
        case SlotState::Free:     return QColor(80,  80,  80);
        case SlotState::Occupied: return QColor(30,  80,  200);
        case SlotState::Reserved: return QColor(200, 160,   0);
        case SlotState::Error:    return QColor(200,  30,  30);
    }
    return QColor(80, 80, 80);
}
