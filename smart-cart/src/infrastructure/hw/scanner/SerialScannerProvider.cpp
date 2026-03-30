// ===== src/infrastructure/hw/scanner/SerialScannerProvider.cpp =====
#include "SerialScannerProvider.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace smartcart::infrastructure::hw::scanner {

static speed_t toSpeed(uint32_t baud) {
    switch (baud) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    default:     return B9600;
    }
}

SerialScannerProvider::SerialScannerProvider(std::string device,
                                             uint32_t    baudRate)
    : device_(std::move(device))
    , baudRate_(baudRate)
{}

SerialScannerProvider::~SerialScannerProvider() { stop(); }

void SerialScannerProvider::setBarcodeCallback(BarcodeCallback cb) {
    cb_ = std::move(cb);
}

void SerialScannerProvider::start() {
    if (active_.load()) return;

    fd_ = ::open(device_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return;

    termios tty{};
    tcgetattr(fd_, &tty);
    cfsetispeed(&tty, toSpeed(baudRate_));
    cfsetospeed(&tty, toSpeed(baudRate_));
    tty.c_cflag  = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_iflag  = IGNBRK | ICRNL;
    tty.c_oflag  = 0;
    tty.c_lflag  = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;
    tcsetattr(fd_, TCSANOW, &tty);

    active_.store(true);
    thread_ = std::thread(&SerialScannerProvider::readLoop, this);
}

void SerialScannerProvider::stop() {
    if (!active_.load()) return;
    active_.store(false);
    if (thread_.joinable()) thread_.join();
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

bool SerialScannerProvider::isActive() const { return active_.load(); }

void SerialScannerProvider::readLoop() {
    std::string buf;
    char        ch;

    while (active_.load()) {
        const ssize_t n = ::read(fd_, &ch, 1);
        if (n <= 0) continue;
        if (ch == '\n' || ch == '\r') {
            if (!buf.empty() && cb_) cb_(buf);
            buf.clear();
        } else {
            buf += ch;
        }
    }
}

} // namespace smartcart::infrastructure::hw::scanner
