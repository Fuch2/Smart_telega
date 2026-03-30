#include "UartStm32Link.hpp"

#include <cerrno>
#include <cstring>
#include <stdexcept>

// POSIX UART
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace smartcart::infrastructure::hw::stm32 {

namespace {

speed_t toSpeed(uint32_t baud) {
    switch (baud) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default:     return B115200;
    }
}

} // namespace

UartStm32Link::UartStm32Link(std::string device,
                             uint32_t    baudRate,
                             uint32_t    timeoutMs)
    : device_(std::move(device))
    , baudRate_(baudRate)
    , timeoutMs_(timeoutMs)
{}

UartStm32Link::~UartStm32Link() {
    close();
}

bool UartStm32Link::open() {
    if (running_.load()) return true;

    fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    cfsetispeed(&tty, toSpeed(baudRate_));
    cfsetospeed(&tty, toSpeed(baudRate_));

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_iflag  = IGNBRK;
    tty.c_oflag  = 0;
    tty.c_lflag  = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1; // 100 ms read timeout

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    parser_.reset();
    running_.store(true);
    rxThread_ = std::thread(&UartStm32Link::rxThreadFunc, this);
    return true;
}

void UartStm32Link::close() {
    if (!running_.load()) return;
    running_.store(false);
    if (rxThread_.joinable()) rxThread_.join();
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    // Wake any waiting sendCommand
    {
        std::lock_guard<std::mutex> lk(replyMtx_);
        replyReady_ = true;
        pendingReply_ = std::nullopt;
    }
    replyCv_.notify_all();
}

bool UartStm32Link::isOpen() const {
    return running_.load();
}

void UartStm32Link::setEventCallback(application::ports::EventCallback cb) {
    eventCb_ = std::move(cb);
}

std::optional<Frame> UartStm32Link::sendCommand(const Frame& cmd) {
    if (!running_.load()) return std::nullopt;

    Frame outFrame = cmd;
    outFrame.seq = ++seqCounter_;

    const auto raw = FrameCodec::encode(outFrame);

    {
        std::lock_guard<std::mutex> lk(replyMtx_);
        replyReady_  = false;
        pendingReply_ = std::nullopt;
        pendingSeq_  = outFrame.seq;
    }

    const ssize_t written = ::write(fd_, raw.data(), raw.size());
    if (written < 0 || static_cast<size_t>(written) != raw.size()) {
        return std::nullopt;
    }

    std::unique_lock<std::mutex> lk(replyMtx_);
    const bool ok = replyCv_.wait_for(
        lk,
        std::chrono::milliseconds(timeoutMs_),
        [this] { return replyReady_; }
    );

    if (!ok) return std::nullopt;
    return pendingReply_;
}

void UartStm32Link::rxThreadFunc() {
    uint8_t buf[256];
    while (running_.load()) {
        const ssize_t n = ::read(fd_, buf, sizeof(buf));
        if (n <= 0) continue;
        for (ssize_t i = 0; i < n; ++i) {
            auto ev = parser_.feed(buf[i]);
            if (ev.type == FrameCodec::ParseEventType::FrameReady && ev.frame) {
                handleParsedFrame(std::move(*ev.frame));
            }
        }
    }
}

void UartStm32Link::handleParsedFrame(Frame frame) {
    const bool isEvent = (frame.frameType == FrameType::Evt);

    if (isEvent) {
        if (eventCb_) eventCb_(frame);
        return;
    }

    // Resp / Ack / Nack — match to pending sendCommand
    std::lock_guard<std::mutex> lk(replyMtx_);
    if (!replyReady_ && frame.seq == pendingSeq_) {
        pendingReply_ = std::move(frame);
        replyReady_   = true;
        replyCv_.notify_one();
    }
}

} // namespace smartcart::infrastructure::hw::stm32
