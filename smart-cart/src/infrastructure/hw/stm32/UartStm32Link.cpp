// ===== src/infrastructure/hw/stm32/UartStm32Link.cpp =====
#include "UartStm32Link.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <fcntl.h>
#include <termios.h>
#include <thread>
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

UartStm32Link::~UartStm32Link() { close(); }

bool UartStm32Link::open() {
    if (running_.load()) return true;

    fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        ::close(fd_); fd_ = -1;
        return false;
    }

    cfsetispeed(&tty, toSpeed(baudRate_));
    cfsetospeed(&tty, toSpeed(baudRate_));
    tty.c_cflag  = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_iflag  = IGNBRK;
    tty.c_oflag  = 0;
    tty.c_lflag  = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        ::close(fd_); fd_ = -1;
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

    // 1. Разблокируем sendCommand если он ждёт ответа на CV.
    //    Без этого join ниже зависнет — sendCommand держит commandMtx_.
    {
        std::lock_guard<std::mutex> lk(replyMtx_);
        replyReady_   = true;
        pendingReply_ = std::nullopt;
        replyCv_.notify_all();
    }

    // 2. Дожидаемся завершения RX-потока.
    if (rxThread_.joinable()) rxThread_.join();

    // 3. Закрываем fd под commandMtx_ — это гарантирует, что любой
    //    параллельный sendCommand завершил ::write() прежде, чем мы закроем
    //    дескриптор. Без этой блокировки возможна гонка close vs ::write.
    std::lock_guard<std::mutex> commandLock(commandMtx_);
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

bool UartStm32Link::isOpen() const { return running_.load(); }

void UartStm32Link::healthCheck() {
    if (isOpen()) {
        // Already open, reset reconnect state
        reconnectAttempts_ = 0;
        return;
    }

    const auto now = std::chrono::steady_clock::now();

    // Exponential backoff: 5s, 10s, 20s, 40s, max 60s.
    // Сдвиг клиппируется до 13 (5000 * 2^13 = 40 960 000) — это безопасно
    // для 32-битного int. На большем количестве попыток backoff всё равно
    // упирается в std::min(..., 60000), но без клиппинга получали бы
    // переполнение и отрицательный результат → бесконечный flood-reconnect.
    const int kMaxShift = 13;
    const int safeShift = std::min(reconnectAttempts_, kMaxShift);
    const int backoffMs = std::min(5000 * (1 << safeShift), 60000);
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - lastReconnectAttempt_);

    if (elapsed.count() < backoffMs) {
        return; // Too soon to retry
    }

    lastReconnectAttempt_ = now;
    reconnectAttempts_++;

    if (open()) {
        reconnectAttempts_ = 0; // Success, reset counter
    }
}

void UartStm32Link::setEventCallback(
    application::ports::EventCallback cb)
{
    std::lock_guard lock(eventCbMtx_);
    eventCb_ = std::move(cb);
}

std::optional<Frame> UartStm32Link::sendCommand(const Frame& cmd) {
    std::lock_guard commandLock(commandMtx_);
    if (!running_.load()) return std::nullopt;

    Frame outFrame = cmd;
    outFrame.seq   = ++seqCounter_;

    const auto raw = FrameCodec::encode(outFrame);

    {
        std::lock_guard<std::mutex> lk(replyMtx_);
        replyReady_   = false;
        pendingReply_ = std::nullopt;
        pendingSeq_   = outFrame.seq;
    }

    const ssize_t written = ::write(fd_, raw.data(), raw.size());
    if (written < 0 || static_cast<size_t>(written) != raw.size())
        return std::nullopt;

    std::unique_lock<std::mutex> lk(replyMtx_);
    const bool ok = replyCv_.wait_for(
        lk,
        std::chrono::milliseconds(timeoutMs_),
        [this] { return replyReady_; }
    );

    return ok ? pendingReply_ : std::nullopt;
}

void UartStm32Link::rxThreadFunc() {
    uint8_t buf[256];
    while (running_.load()) {
        const ssize_t n = ::read(fd_, buf, sizeof(buf));
        if (n <= 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        for (ssize_t i = 0; i < n; ++i) {
            auto ev = parser_.feed(buf[i]);
            if (ev.type == FrameCodec::ParseEventType::FrameReady && ev.frame)
                handleParsedFrame(std::move(*ev.frame));
        }
    }
}

void UartStm32Link::handleParsedFrame(Frame frame) {
    const bool isEvent = (frame.type == FrameType::Evt);

    if (isEvent) {
        application::ports::EventCallback cb;
        {
            std::lock_guard lock(eventCbMtx_);
            cb = eventCb_;
        }
        if (cb) cb(frame);
        return;
    }

    std::lock_guard<std::mutex> lk(replyMtx_);
    if (!replyReady_ && frame.seq == pendingSeq_) {
        pendingReply_ = std::move(frame);
        replyReady_   = true;
        replyCv_.notify_one();
    }
}

} // namespace smartcart::infrastructure::hw::stm32
