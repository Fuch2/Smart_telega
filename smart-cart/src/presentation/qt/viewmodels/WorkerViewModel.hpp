#pragma once
#include <QObject>
#include <array>

class QTimer;

class WorkerViewModel final : public QObject {
    Q_OBJECT
public:
    explicit WorkerViewModel(QObject* parent = nullptr);

    void start();
    void stop();

signals:
    void rfidStatusChanged(const QString& v);
    void uartStatusChanged(const QString& v);
    void scannerStatusChanged(const QString& v);
    void moduleSerialChanged(const QString& v);

    void slotOccupiedChanged(int slotIndex, bool occupied); // 1..24
    void targetSlotChanged(int slotIndex);                  // 1..24, 0 = none

private slots:
    void onTick();

private:
    QTimer* timer_{nullptr};
    std::array<bool, 24> occupied_{};
    int tick_{0};
    int target_{0};

    static int wrap1to24(int v);
};
