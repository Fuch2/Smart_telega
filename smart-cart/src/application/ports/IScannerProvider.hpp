#pragma once

#include <functional>
#include <string>

namespace smartcart::application::ports {

/// Порт для получения штрихкода от сканера.
/// Реализации: USB HID сканер, эмулятор клавиатуры, тестовая заглушка.
class IScannerProvider {
public:
    virtual ~IScannerProvider() = default;

    /// Установить callback, вызываемый при успешном сканировании.
    /// Вызывается из потока сканера — реализация должна быть thread-safe.
    using BarcodeCallback = std::function<void(std::string barcode)>;
    virtual void setBarcodeCallback(BarcodeCallback cb) = 0;

    /// Запустить прослушивание сканера.
    virtual void start() = 0;

    /// Остановить прослушивание.
    virtual void stop() = 0;

    /// Проверить, активен ли сканер.
    virtual bool isActive() const = 0;
};

} // namespace smartcart::application::ports
