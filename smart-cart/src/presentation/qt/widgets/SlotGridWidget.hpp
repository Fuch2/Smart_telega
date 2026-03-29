#pragma once
#include <QWidget>
#include <QVector>

class QPushButton;

class SlotGridWidget final : public QWidget {
    Q_OBJECT
public:
    explicit SlotGridWidget(QWidget* parent = nullptr);

signals:
    void slotClicked(int slotIndex); // 1..24

public slots:
    void setSlotOccupied(int slotIndex, bool occupied);
    void setTargetSlot(int slotIndex); // 1..24, 0 = none

private:
    QVector<QPushButton*> slots_;
    int target_{0};
    void repaintSlot(int idx1);
};
