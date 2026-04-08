// ===== src/infrastructure/hw/rfid/MockRfidProvider.hpp =====
#pragma once

#include "application/ports/IRfidProvider.hpp"

#include <chrono>
#include <mutex>
#include <optional>
#include <thread>

namespace smartcart::infrastructure::hw::rfid {

class MockRfidProvider final
    : public application::ports::IRfidProvider
{
public:
    void setRfidCallback(RfidCallback cb) override { cb_ = std::move(cb); }
    void start()  override { active_ = true;  }
    void stop()   override { active_ = false; }
    bool isActive() const override { return active_; }
    std::optional<std::string> readOnce(int timeoutMs) override {
        const auto deadline =
            std::chrono::steady_clock::now() +
            std::chrono::milliseconds(timeoutMs > 0 ? timeoutMs : 1);

        while (std::chrono::steady_clock::now() < deadline) {
            {
                std::lock_guard lock(mtx_);
                if (pendingUid_.has_value()) {
                    const auto uid = pendingUid_;
                    pendingUid_.reset();
                    return uid;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return std::nullopt;
    }

    void inject(const std::string& uid) {
        {
            std::lock_guard lock(mtx_);
            pendingUid_ = uid;
        }
        if (active_ && cb_) cb_(uid);
    }

private:
    RfidCallback cb_;
    bool         active_ = false;
    std::mutex   mtx_;
    std::optional<std::string> pendingUid_;
};

} // namespace smartcart::infrastructure::hw::rfid
