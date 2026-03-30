// ===== src/infrastructure/hw/rfid/Rc522RfidProvider.hpp =====
#pragma once

#include "application/ports/IRfidProvider.hpp"
#include <atomic>
#include <string>
#include <thread>

namespace smartcart::infrastructure::hw::rfid {

/// RC522 через SPI (/dev/spidevX.Y) — заглушка для будущей реализации.
/// В MVP не используется, но компилируется.
class Rc522RfidProvider final
    : public application::ports::IRfidProvider
{
public:
    explicit Rc522RfidProvider(std::string spiDevice);
    ~Rc522RfidProvider() override;

    void setRfidCallback(RfidCallback cb) override;
    void start()  override;
    void stop()   override;
    bool isActive() const override;

private:
    void pollLoop();

    std::string       spiDevice_;
    RfidCallback      cb_;
    std::atomic<bool> active_{false};
    std::thread       thread_;
};

} // namespace smartcart::infrastructure::hw::rfid
