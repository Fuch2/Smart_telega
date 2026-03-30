#pragma once

#include "application/ports/IStm32Link.hpp"
#include "Protocol.hpp"

#include <functional>
#include <optional>

namespace smartcart::infrastructure::hw::stm32 {

// Synchronous mock: caller installs a handler that maps cmd → reply.
// Useful for unit/integration tests and demoMode.
class MockStm32Link final : public application::ports::IStm32Link {
public:
    using Handler = std::function<std::optional<Frame>(const Frame&)>;

    explicit MockStm32Link(Handler handler = nullptr);

    bool open()  override;
    void close() override;
    bool isOpen() const override;

    std::optional<Frame> sendCommand(const Frame& cmd) override;
    void setEventCallback(application::ports::EventCallback cb) override;

    // Inject an unsolicited event from test code
    void injectEvent(const Frame& evt);

private:
    Handler   handler_;
    application::ports::EventCallback eventCb_;
    bool      open_ {false};
};

} // namespace smartcart::infrastructure::hw::stm32
