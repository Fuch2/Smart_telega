#pragma once

#include "infrastructure/hw/stm32/Protocol.hpp"

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

namespace smartcart::application::ports {

using namespace smartcart::infrastructure::hw::stm32;

// Callback invoked when an unsolicited event frame arrives (e.g. EvtReady, EvtSwitchChanged)
using EventCallback = std::function<void(const Frame&)>;

class IStm32Link {
public:
    virtual ~IStm32Link() = default;

    // Open the underlying transport (UART / mock). Idempotent.
    virtual bool open() = 0;

    // Close the transport. Idempotent.
    virtual void close() = 0;

    // Send a command and wait synchronously for the matching Resp/Ack/Nack.
    // Returns std::nullopt on timeout or transport error.
    virtual std::optional<Frame> sendCommand(const Frame& cmd) = 0;

    // Register a callback for unsolicited event frames.
    virtual void setEventCallback(EventCallback cb) = 0;

    // True if the transport is open and healthy.
    virtual bool isOpen() const = 0;
};

} // namespace smartcart::application::ports
