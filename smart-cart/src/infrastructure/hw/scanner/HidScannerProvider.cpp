// ===== src/infrastructure/hw/scanner/HidScannerProvider.cpp =====
#include "HidScannerProvider.hpp"

#include <fcntl.h>
#ifdef __linux__
#include <linux/input.h>
#include <poll.h>
#endif
#include <unistd.h>

#include <cctype>
#include <cstdint>
#include <string>
#include <utility>

namespace smartcart::infrastructure::hw::scanner {

// Таблица: evdev keycode → ASCII символ.
// Покрывает символы, типичные для производственных штрихкодов:
// цифры, буквы (en-US QWERTY), -, =, [, ], \, ;, ', `, ,, ., /, пробел.
#ifdef __linux__
static char keycodeToChar(uint16_t code, bool shift) {
    if (code >= KEY_1 && code <= KEY_9) {
        const char digits[] = "1234567890";
        return shift ? "!@#$%^&*()"[code - KEY_1] : digits[code - KEY_1];
    }
    if (code == KEY_0)     return shift ? ')'  : '0';
    if (code == KEY_MINUS) return shift ? '_'  : '-';
    if (code == KEY_EQUAL) return shift ? '+'  : '=';
    if (code == KEY_LEFTBRACE)  return shift ? '{'  : '[';
    if (code == KEY_RIGHTBRACE) return shift ? '}'  : ']';
    if (code == KEY_BACKSLASH)  return shift ? '|'  : '\\';
    if (code == KEY_SEMICOLON)  return shift ? ':'  : ';';
    if (code == KEY_APOSTROPHE) return shift ? '"'  : '\'';
    if (code == KEY_GRAVE)      return shift ? '~'  : '`';
    if (code == KEY_COMMA)      return shift ? '<'  : ',';
    if (code == KEY_DOT)        return shift ? '>'  : '.';
    if (code == KEY_SLASH)      return shift ? '?'  : '/';
    if (code == KEY_SPACE)      return ' ';

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
#endif

HidScannerProvider::HidScannerProvider(std::string devicePath)
    : devicePath_(std::move(devicePath))
{}

HidScannerProvider::~HidScannerProvider() { stop(); }

void HidScannerProvider::setBarcodeCallback(BarcodeCallback cb) {
    cb_ = std::move(cb);
}

void HidScannerProvider::start() {
    if (active_.load()) return;
#ifdef __linux__
    fd_ = ::open(devicePath_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd_ < 0) return;
    active_.store(true);
    thread_ = std::thread(&HidScannerProvider::readLoop, this);
#endif
}

void HidScannerProvider::stop() {
    if (!active_.load()) return;
    active_.store(false);
    if (thread_.joinable()) thread_.join();
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

bool HidScannerProvider::isActive() const { return active_.load(); }

void HidScannerProvider::readLoop() {
#ifdef __linux__
    std::string buf;
    bool shift = false;
    input_event ev{};

    // poll() с таймаутом 200 мс: поток спит до прихода данных или истечения
    // таймаута — этим избегаем busy-wait с 100% CPU при O_NONBLOCK.
    // Таймаут также обеспечивает регулярную проверку active_ для корректной
    // остановки в stop().
    constexpr int kPollTimeoutMs = 200;

    while (active_.load()) {
        pollfd pfd{};
        pfd.fd = fd_;
        pfd.events = POLLIN;

        const int pr = ::poll(&pfd, 1, kPollTimeoutMs);
        if (pr <= 0) {
            // pr == 0 — таймаут (нет данных), pr < 0 — EINTR/ошибка.
            // В обоих случаях просто возвращаемся к проверке active_.
            continue;
        }
        if ((pfd.revents & POLLIN) == 0) {
            continue;
        }

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
#endif
}

} // namespace smartcart::infrastructure::hw::scanner
