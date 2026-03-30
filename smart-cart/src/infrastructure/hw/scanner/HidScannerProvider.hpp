// ===== src/infrastructure/hw/scanner/HidScannerProvider.hpp =====
#pragma once

#include "application/ports/IScannerProvider.hpp"
#include <atomic>
#include <thread>

namespace smartcart::infrastructure::hw::scanner {

/// Читает штрихкоды с HID-сканера через /dev/input/eventX (Linux evdev).
class HidScannerProvider final
    : public application::ports::IScannerProvider
{
public:
    explicit HidScannerProvider(std::string devicePath);
    ~HidScannerProvider() override;

    void setBarcodeCallback(BarcodeCallback cb) override;
    void start()  override;
    void stop()   override;
    bool isActive() const override;

private:
    void readLoop();

    std::string     devicePath_;
    BarcodeCallback cb_;
    std::atomic<bool> active_{false};
    std::thread     thread_;
    int             fd_{-1};
};

} // namespace smartcart::infrastructure::hw::scanner
