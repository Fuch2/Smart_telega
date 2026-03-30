#pragma once
#include <QObject>
#include <array>
#include "../../../infrastructure/hw/stm32/UartStm32Link.hpp"


class WorkerViewModel final : public QObject {
    Q_OBJECT
public:
    explicit WorkerViewModel(QObject* parent = nullptr);
    void setStm32Link(application::ports::IStm32Link* link);

    void start();
    void stop();

signals:
    void rfidStatusChanged(const QString& v);
    void uartStatusChanged(const QString& v);
    void scannerStatusChanged(const QString& v);
    void moduleSerialChanged(const QString& v);

    void slotOccupiedChanged(int slotIndex, bool occupied); // 1..24
    void targetSlotChanged(int slotIndex);                  // 1..24, 0 = none

private:
    std::array<bool, 24> occupied_{};
    int target_{0};
    void onStm32Event(const infrastructure::hw::stm32::Frame& frame);
    application::ports::IStm32Link* stm32_ = nullptr;

};
