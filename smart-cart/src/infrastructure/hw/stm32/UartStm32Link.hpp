#pragma once

#include "application/ports/IStm32Link.hpp"
#include "FrameCodec.hpp"
#include "Protocol.hpp"

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace smartcart::infrastructure::hw::stm32 {

class UartStm32Link final : public application::ports::IStm32Link {
public:
    explicit UartStm32Link(std::string device,
                           uint32_t    baudRate      = 115200,
                           uint32_t    timeoutMs     = 500);
    ~UartStm32Link() override;

    bool open()  override;
    void close() override;

    std::optional<Frame> sendCommand(const Frame& cmd) override;
    void setEventCallback(application::ports::EventCallback cb) override;
    bool isOpen() const override;

private:
    void rxThreadFunc();
    void handleParsedFrame(Frame frame);

    std::string  device_;
    uint32_t     baudRate_;
    uint32_t     timeoutMs_;
    int          fd_    {-1};

    std::atomic<bool> running_ {false};
    std::thread       rxThread_;

    std::mutex                        eventCbMtx_;
    application::ports::EventCallback eventCb_;

    // Pending synchronous reply
    std::mutex              commandMtx_;
    std::mutex              replyMtx_;
    std::condition_variable replyCv_;
    std::optional<Frame>    pendingReply_;
    bool                    replyReady_ {false};
    uint8_t                 pendingSeq_ {0};

    uint8_t seqCounter_ {0};

    FrameCodec::StreamParser parser_;
};

} // namespace smartcart::infrastructure::hw::stm32
