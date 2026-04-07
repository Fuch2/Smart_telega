// ===== src/infrastructure/hw/stm32/MockStm32Link.hpp =====
#pragma once

#include "application/ports/IStm32Link.hpp"
#include "Protocol.hpp"

#include <functional>
#include <optional>
#include <atomic>
#include <thread>
#include <mutex>

namespace smartcart::infrastructure::hw::stm32 {

class MockStm32Link final : public application::ports::IStm32Link {
public:
    using Handler = std::function<std::optional<Frame>(const Frame&)>;

    explicit MockStm32Link(Handler handler = nullptr);
    ~MockStm32Link() override;

    bool open()  override;
    void close() override;
    bool isOpen() const override;

    std::optional<Frame> sendCommand(const Frame& cmd) override;
    void setEventCallback(application::ports::EventCallback cb) override;

    // Эмулировать нажатие/отпускание кнопки (channel 0-based, occupied=true/false)
    void simulateSwitchEvent(int channel, bool occupied);

    // Inject произвольный event из тест-кода
    void injectEvent(const Frame& evt);

private:
    Handler   handler_;

    std::mutex                                commandMtx_;
    std::mutex                                eventCbMtx_;
    application::ports::EventCallback         eventCb_;

    bool      open_ {false};
};

} // namespace smartcart::infrastructure::hw::stm32
