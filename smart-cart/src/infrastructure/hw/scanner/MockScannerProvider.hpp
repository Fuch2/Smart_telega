// ===== src/infrastructure/hw/scanner/MockScannerProvider.hpp =====
// Новый файл: был пустым
#pragma once

#include "application/ports/IScannerProvider.hpp"

namespace smartcart::infrastructure::hw::scanner {

/// Тестовый mock-сканер: штрихкоды инжектируются программно через inject().
class MockScannerProvider final
    : public application::ports::IScannerProvider
{
public:
    void setBarcodeCallback(BarcodeCallback cb) override {
        cb_ = std::move(cb);
    }

    void start()  override { active_ = true;  }
    void stop()   override { active_ = false; }
    bool isActive() const override { return active_; }

    /// Эмулировать сканирование штрихкода.
    void inject(const std::string& barcode) {
        if (active_ && cb_) cb_(barcode);
    }

private:
    BarcodeCallback cb_;
    bool            active_ = false;
};

} // namespace smartcart::infrastructure::hw::scanner
