// ===== src/infrastructure/hw/rfid/MultiRc522RfidProvider.hpp =====
#pragma once

#include "application/ports/IRfidProvider.hpp"
#include "infrastructure/hw/rfid/Rc522RfidProvider.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace smartcart::infrastructure::hw::rfid {

/// Набор RC522-считывателей. Каждый считыватель должен быть доступен как
/// отдельный spidev, например /dev/spidev0.0 ... /dev/spidev0.5.
class MultiRc522RfidProvider final
    : public application::ports::IRfidProvider
{
public:
    explicit MultiRc522RfidProvider(std::vector<std::string> spiDevices);
    ~MultiRc522RfidProvider() override;

    void setRfidCallback(RfidCallback cb) override;
    void start() override;
    void stop() override;
    bool isActive() const override;

    std::optional<std::string> readOnce(int timeoutMs) override;
    std::vector<std::string> readAllOnce(int timeoutMs) override;

    const std::vector<std::string>& spiDevices() const noexcept {
        return spiDevices_;
    }

private:
    void pollLoop();

    std::vector<std::string> spiDevices_;
    std::vector<std::unique_ptr<Rc522RfidProvider>> readers_;
    RfidCallback cb_;
    std::atomic<bool> active_{false};
    std::thread thread_;
    std::mutex mtx_;
};

} // namespace smartcart::infrastructure::hw::rfid
