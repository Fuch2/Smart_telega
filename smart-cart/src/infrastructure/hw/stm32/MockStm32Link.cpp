#include "MockStm32Link.hpp"

namespace smartcart::infrastructure::hw::stm32 {

MockStm32Link::MockStm32Link(Handler handler)
    : handler_(std::move(handler))
{}

bool MockStm32Link::open() {
    open_ = true;
    return true;
}

void MockStm32Link::close() {
    open_ = false;
}

bool MockStm32Link::isOpen() const {
    return open_;
}

void MockStm32Link::setEventCallback(application::ports::EventCallback cb) {
    eventCb_ = std::move(cb);
}

std::optional<Frame> MockStm32Link::sendCommand(const Frame& cmd) {
    if (!open_) return std::nullopt;
    if (handler_) return handler_(cmd);

    // Default: echo back an Ack with same seq/commandId
    Frame ack;
    ack.protocolVersion = cmd.protocolVersion;
    ack.frameType       = FrameType::Ack;
    ack.seq             = cmd.seq;
    ack.commandId       = cmd.commandId;
    return ack;
}

void MockStm32Link::injectEvent(const Frame& evt) {
    if (eventCb_) eventCb_(evt);
}

} // namespace smartcart::infrastructure::hw::stm32
