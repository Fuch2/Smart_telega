// ===== src/infrastructure/hw/rfid/MockRfidProvider.hpp =====
#pragma once

#include "application/ports/IRfidProvider.hpp"

namespace smartcart::infrastructure::hw::rfid {

class MockRfidProvider final
    : public application::ports::IRfidProvider
{
public:
    void setRfidCallback(RfidCallback cb) override { cb_ = std::move(cb); }
    void start()  override { active_ = true;  }
    void stop()   override { active_ = false; }
    bool isActive() const override { return active_; }

    void inject(const std::string& uid) {
        if (active_ && cb_) cb_(uid);
    }

private:
    RfidCallback cb_;
    bool         active_ = false;
};

} // namespace smartcart::infrastructure::hw::rfid
