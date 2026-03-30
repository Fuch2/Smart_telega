// ===== src/infrastructure/hw/rfid/Rc522RfidProvider.cpp =====
#include "Rc522RfidProvider.hpp"
#include <chrono>

namespace smartcart::infrastructure::hw::rfid {

Rc522RfidProvider::Rc522RfidProvider(std::string spiDevice)
    : spiDevice_(std::move(spiDevice))
{}

Rc522RfidProvider::~Rc522RfidProvider() { stop(); }

void Rc522RfidProvider::setRfidCallback(RfidCallback cb) {
    cb_ = std::move(cb);
}

void Rc522RfidProvider::start() {
    if (active_.load()) return;
    active_.store(true);
    thread_ = std::thread(&Rc522RfidProvider::pollLoop, this);
}

void Rc522RfidProvider::stop() {
    if (!active_.load()) return;
    active_.store(false);
    if (thread_.joinable()) thread_.join();
}

bool Rc522RfidProvider::isActive() const { return active_.load(); }

void Rc522RfidProvider::pollLoop() {
    // TODO: реализовать через librc522 или прямой SPI
    while (active_.load())
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

} // namespace smartcart::infrastructure::hw::rfid
