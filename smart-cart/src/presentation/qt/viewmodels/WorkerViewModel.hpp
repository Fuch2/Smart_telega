#pragma once
#include <QObject>
#include <array>

// Forward declarations — MOC не нужны полные типы
namespace smartcart::application::ports { class IStm32Link; }
namespace smartcart::infrastructure::hw::stm32 { struct Frame; }

class WorkerViewModel final : public QObject {
    Q_OBJECT
public:
    explicit WorkerViewModel(QObject* parent = nullptr);

    void setStm32Link(smartcart::application::ports::IStm32Link* link);
    void start();
    void stop();

signals:
    void rfidStatusChanged(const QString& v);
    void uartStatusChanged(const QString& v);
    void scannerStatusChanged(const QString& v);
    void moduleSerialChanged(const QString& v);
    void slotOccupiedChanged(int slotIndex, bool occupied);
    void targetSlotChanged(int slotIndex);

private:
    std::array<bool, 24> occupied_{};
    int target_{0};
    void onStm32Event(const smartcart::infrastructure::hw::stm32::Frame& frame);
    smartcart::application::ports::IStm32Link* stm32_ = nullptr;
};
