// ===== src/infrastructure/hw/scanner/HidScannerProvider.cpp =====
#include "HidScannerProvider.hpp"

#include <fcntl.h>
#include <linux/input.h>
#include <unistd.h>

#include <cctype>
#include <string>

namespace smartcart::infrastructure::hw::scanner {

// Таблица: evdev keycode → ASCII символ (только цифры и буквы)
static char keycodeToChar(uint16_t code, bool shift) {
    if (code >= KEY_1 && code <= KEY_9) {
        const char digits[] = "1234567890";
        return shift ? "!@#$%^&*()"[code - KEY_1] : digits[code - KEY_1];
    }
    if (code == KEY_0) return shift ? ')' : '0';
    if (code >= KEY_Q && code <= KEY_P) {
        const char row[] = "qwertyuiop";
        const char c = row[code - KEY_Q];
        return shift ? static_cast<char>(std::toupper(c)) : c;
    }
    if (code >= KEY_A && code <= KEY_L) {
        const char row[] = "asdfghjkl";
        const char c = row[code - KEY_A];
        return shift ? static_cast<char>(std::toupper(c)) : c;
    }
    if (code >= KEY_Z && code <= KEY_M) {
        const char row[] = "zxcvbnm";
        const char c = row[code - KEY_Z];
        return shift ? static_cast<char>(std::toupper(c)) : c;
    }
    return '\0';
}

HidScannerProvider::HidScannerProvider(std::string devicePath)
    : devicePath_(std::move(devicePath))
{}

HidScannerProvider::~HidScannerProvider() { stop(); }

void HidScannerProvider::setBarcodeCallback(BarcodeCallback cb) {
    cb_ = std::move(cb);
}

void HidScannerProvider::start() {
    if (active_.load()) return;
    fd_ = ::open(devicePath_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd_ < 0) return;
    active_.store(true);
    thread_ = std::thread(&HidScannerProvider::readLoop, this);
}

void HidScannerProvider::stop() {
    if (!active_.load()) return;
    active_.store(false);
    if (thread_.joinable()) thread_.join();
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

bool HidScannerProvider::isActive() const { return active_.load(); }

void HidScannerProvider::readLoop() {
    std::string buf;
    bool shift = false;
    input_event ev{};

    while (active_.load()) {
        const ssize_t n = ::read(fd_, &ev, sizeof(ev));
        if (n != sizeof(ev)) continue;
        if (ev.type != EV_KEY) continue;

        if (ev.code == KEY_LEFTSHIFT || ev.code == KEY_RIGHTSHIFT) {
            shift = (ev.value != 0);
            continue;
        }

        if (ev.value == 1) { // key down
            if (ev.code == KEY_ENTER) {
                if (!buf.empty() && cb_) cb_(buf);
                buf.clear();
            } else {
                const char c = keycodeToChar(ev.code, shift);
                if (c != '\0') buf += c;
            }
        }
    }
}

} // namespace smartcart::infrastructure::hw::scanner
