#pragma once

#include <functional>
#include <optional>
#include <string>

namespace smartcart::application::ports {

/// Порт для чтения RFID-меток (альтернатива штрихкоду).
/// Реализации: RFID-ридер по UART/USB, тестовая заглушка.
class IRfidProvider {
public:
    virtual ~IRfidProvider() = default;

    /// Callback при успешном чтении метки.
    /// uid — уникальный идентификатор метки.
    using RfidCallback = std::function<void(std::string uid)>;
    virtual void setRfidCallback(RfidCallback cb) = 0;

    /// Запустить опрос ридера.
    virtual void start() = 0;

    /// Остановить опрос.
    virtual void stop() = 0;

    /// Проверить, активен ли ридер.
    virtual bool isActive() const = 0;

    /// Синхронно прочитать UID метки.
    /// Возвращает UID либо std::nullopt, если метка не обнаружена.
    virtual std::optional<std::string> readOnce(int timeoutMs) = 0;
};

} // namespace smartcart::application::ports
