// ===== src/infrastructure/hw/scanner/SerialScannerProvider.hpp =====
#pragma once

#include "application/ports/IScannerProvider.hpp"
#include <atomic>
#include <string>
#include <thread>

namespace smartcart::infrastructure::hw::scanner {

/// Читает штрихкоды со сканера, подключённого через UART (/dev/ttyUSBx).
/// Каждая строка (до '\n') — один штрихкод.
class SerialScannerProvider final
    : public application::ports::IScannerProvider
{
public:
    explicit SerialScannerProvider(std::string device,
                                   uint32_t    baudRate = 9600);
    ~SerialScannerProvider() override;

    void setBarcodeCallback(BarcodeCallback cb) override;
    void start()  override;
    void stop()   override;
    bool isActive() const override;

private:
    void readLoop();

    std::string       device_;
    uint32_t          baudRate_;
    BarcodeCallback   cb_;
    std::atomic<bool> active_{false};
    std::thread       thread_;
    int               fd_{-1};
};

} // namespace smartcart::infrastructure::hw::scanner
