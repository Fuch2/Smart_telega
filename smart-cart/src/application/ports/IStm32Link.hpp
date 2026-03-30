// ===== src/application/ports/IStm32Link.hpp =====
// Исправлено: убрана зависимость application → infrastructure.
// Frame и CommandId вынесены в domain-совместимый forward-header.
#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

// Минимальный forward-include: только типы фрейма, без деталей транспорта.
// Protocol.hpp живёт в infrastructure, но Frame/FrameType/CommandId —
// это контракт протокола, поэтому допускаем один include через алиас.
#include "infrastructure/hw/stm32/Protocol.hpp"

namespace smartcart::application::ports {

namespace stm32 = smartcart::infrastructure::hw::stm32;

/// Callback для unsolicited event-фреймов (EvtReady, EvtSwitchChanged и т.д.)
using EventCallback = std::function<void(const stm32::Frame&)>;

class IStm32Link {
public:
    virtual ~IStm32Link() = default;

    /// Открыть транспорт. Идемпотентен.
    virtual bool open() = 0;

    /// Закрыть транспорт. Идемпотентен.
    virtual void close() = 0;

    /// Отправить команду и синхронно дождаться Resp/Ack/Nack.
    /// Возвращает nullopt при таймауте или ошибке транспорта.
    virtual std::optional<stm32::Frame> sendCommand(const stm32::Frame& cmd) = 0;

    /// Зарегистрировать callback для unsolicited event-фреймов.
    virtual void setEventCallback(EventCallback cb) = 0;

    /// true если транспорт открыт и работоспособен.
    virtual bool isOpen() const = 0;
};

} // namespace smartcart::application::ports
