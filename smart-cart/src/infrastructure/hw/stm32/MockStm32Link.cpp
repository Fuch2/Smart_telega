// ===== src/infrastructure/hw/stm32/MockStm32Link.cpp =====
#include "MockStm32Link.hpp"

namespace smartcart::infrastructure::hw::stm32 {

MockStm32Link::MockStm32Link(Handler handler)
    : handler_(std::move(handler))
{}

MockStm32Link::~MockStm32Link() { close(); }

bool MockStm32Link::open()  { open_ = true;  return true; }
void MockStm32Link::close() { open_ = false; }
bool MockStm32Link::isOpen() const { return open_; }

void MockStm32Link::setEventCallback(application::ports::EventCallback cb) {
    std::lock_guard lock(eventCbMtx_);
    eventCb_ = std::move(cb);
}

std::optional<Frame> MockStm32Link::sendCommand(const Frame& cmd) {
    std::lock_guard commandLock(commandMtx_);
    if (!open_) return std::nullopt;
    if (handler_) return handler_(cmd);

    Frame ack;
    ack.protocolVersion = cmd.protocolVersion;
    ack.type  = FrameType::Ack;
    ack.seq   = cmd.seq;
    ack.cmdId = cmd.cmdId;
    return ack;
}

void MockStm32Link::simulateSwitchEvent(int channel, bool occupied) {
    Frame evt;
    evt.type    = FrameType::Evt;
    evt.cmdId   = CommandId::EvtSwitchChanged;
    // payload[0] = channel 0-based, payload[1] = occupied
    evt.payload = {
        static_cast<uint8_t>(channel),
        static_cast<uint8_t>(occupied ? 0x01 : 0x00)
    };
    injectEvent(evt);
}

void MockStm32Link::injectEvent(const Frame& evt) {
    application::ports::EventCallback cb;
    {
        std::lock_guard lock(eventCbMtx_);
        cb = eventCb_;
    }
    if (cb) cb(evt);
}

} // namespace smartcart::infrastructure::hw::stm32
