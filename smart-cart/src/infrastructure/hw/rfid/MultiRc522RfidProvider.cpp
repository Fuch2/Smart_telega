// ===== src/infrastructure/hw/rfid/MultiRc522RfidProvider.cpp =====
#include "MultiRc522RfidProvider.hpp"

#include <algorithm>
#include <chrono>
#include <thread>
#include <unordered_set>
#include <utility>

namespace smartcart::infrastructure::hw::rfid {

MultiRc522RfidProvider::MultiRc522RfidProvider(
    std::vector<std::string> spiDevices)
    : spiDevices_(std::move(spiDevices))
{
    readers_.reserve(spiDevices_.size());
    for (const auto& device : spiDevices_) {
        if (!device.empty()) {
            readers_.push_back(std::make_unique<Rc522RfidProvider>(device));
        }
    }
}

MultiRc522RfidProvider::~MultiRc522RfidProvider() {
    stop();
}

void MultiRc522RfidProvider::setRfidCallback(RfidCallback cb) {
    std::lock_guard lock(mtx_);
    cb_ = std::move(cb);
}

void MultiRc522RfidProvider::start() {
    if (active_.exchange(true)) {
        return;
    }
    thread_ = std::thread(&MultiRc522RfidProvider::pollLoop, this);
}

void MultiRc522RfidProvider::stop() {
    if (!active_.exchange(false)) {
        return;
    }
    if (thread_.joinable()) {
        thread_.join();
    }
}

bool MultiRc522RfidProvider::isActive() const {
    return active_.load();
}

std::optional<std::string> MultiRc522RfidProvider::readOnce(int timeoutMs) {
    const auto all = readAllOnce(timeoutMs);
    if (all.empty()) {
        return std::nullopt;
    }
    return all.front();
}

std::vector<std::string> MultiRc522RfidProvider::readAllOnce(int timeoutMs) {
    std::vector<std::string> result;
    std::unordered_set<std::string> seen;

    if (readers_.empty()) {
        return result;
    }

    // timeoutMs — это бюджет на один физический считыватель. Его нельзя
    // делить между всеми spidev: при 6 настроенных CE реально подключённый
    // RC522 получает слишком короткое окно и нестабильно видит метку.
    const int perReaderTimeout = std::max(100, timeoutMs);

    for (auto& reader : readers_) {
        if (!reader) {
            continue;
        }

        if (auto uid = reader->readOnce(perReaderTimeout);
            uid.has_value() && !uid->empty() && seen.insert(*uid).second)
        {
            result.push_back(*uid);
        }
    }

    return result;
}

void MultiRc522RfidProvider::pollLoop() {
    std::unordered_set<std::string> lastSeen;

    while (active_.load()) {
        const auto current = readAllOnce(250);
        std::unordered_set<std::string> currentSet(current.begin(), current.end());

        RfidCallback cb;
        {
            std::lock_guard lock(mtx_);
            cb = cb_;
        }

        if (cb) {
            for (const auto& uid : current) {
                if (lastSeen.find(uid) == lastSeen.end()) {
                    cb(uid);
                }
            }
        }

        lastSeen = std::move(currentSet);
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }
}

} // namespace smartcart::infrastructure::hw::rfid
