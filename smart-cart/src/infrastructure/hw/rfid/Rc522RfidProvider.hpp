// ===== src/infrastructure/hw/rfid/Rc522RfidProvider.hpp =====
#pragma once

#include "application/ports/IRfidProvider.hpp"
#include <atomic>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace smartcart::infrastructure::hw::rfid {

/// RC522 через SPI (/dev/spidevX.Y).
/// Используется для чтения UID RFID-метки модуля на Raspberry Pi.
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

    /// Синхронно прочитать UID метки за ограниченное время.
    /// Возвращает hex-строку UID или std::nullopt, если метка не найдена.
    std::optional<std::string> readOnce(int timeoutMs);

private:
    void pollLoop();
    bool openDevice();
    void closeDevice();
    bool initializeChip();
    std::optional<std::string> tryReadUid();

    std::string       spiDevice_;
    RfidCallback      cb_;
    std::atomic<bool> active_{false};
    std::thread       thread_;
    std::mutex        ioMtx_;
    int               fd_{-1};
};

} // namespace smartcart::infrastructure::hw::rfid
