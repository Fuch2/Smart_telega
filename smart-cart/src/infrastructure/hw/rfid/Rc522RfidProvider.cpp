#include "Rc522RfidProvider.hpp"

#include <chrono>
#include <thread>
#include <utility>

#ifdef __linux__
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <array>
#include <cstdint>
#include <sstream>
#endif

namespace smartcart::infrastructure::hw::rfid {

namespace {

#ifdef __linux__

constexpr uint8_t kCommandReg      = 0x01;
constexpr uint8_t kComIEnReg       = 0x02;
constexpr uint8_t kComIrqReg       = 0x04;
constexpr uint8_t kErrorReg        = 0x06;
constexpr uint8_t kFIFODataReg     = 0x09;
constexpr uint8_t kFIFOLevelReg    = 0x0A;
constexpr uint8_t kControlReg      = 0x0C;
constexpr uint8_t kBitFramingReg   = 0x0D;
constexpr uint8_t kModeReg         = 0x11;
constexpr uint8_t kTxControlReg    = 0x14;
constexpr uint8_t kTxASKReg        = 0x15;
constexpr uint8_t kRFCfgReg        = 0x26;
constexpr uint8_t kTModeReg        = 0x2A;
constexpr uint8_t kTPrescalerReg   = 0x2B;
constexpr uint8_t kTReloadRegH     = 0x2C;
constexpr uint8_t kTReloadRegL     = 0x2D;
constexpr uint8_t kVersionReg      = 0x37;

constexpr uint8_t kCmdIdle         = 0x00;
constexpr uint8_t kCmdTransceive   = 0x0C;
constexpr uint8_t kCmdSoftReset    = 0x0F;

constexpr uint8_t kPiccReqA        = 0x26;
constexpr uint8_t kPiccAntiColl    = 0x93;
constexpr uint8_t kPiccAntiCollNvb = 0x20;

constexpr uint32_t kSpiSpeedHz     = 1000000;

std::string uidToHex(const std::array<uint8_t, 4>& uid) {
    static const char* digits = "0123456789ABCDEF";
    std::string text;
    text.reserve(uid.size() * 2);
    for (const auto byte : uid) {
        text.push_back(digits[(byte >> 4) & 0x0F]);
        text.push_back(digits[byte & 0x0F]);
    }
    return text;
}

bool spiTransfer(int fd, const uint8_t* tx, uint8_t* rx, std::size_t size) {
    spi_ioc_transfer transfer{};
    transfer.tx_buf = reinterpret_cast<unsigned long>(tx);
    transfer.rx_buf = reinterpret_cast<unsigned long>(rx);
    transfer.len = static_cast<uint32_t>(size);
    transfer.speed_hz = kSpiSpeedHz;
    transfer.bits_per_word = 8;
    return ioctl(fd, SPI_IOC_MESSAGE(1), &transfer) >= 0;
}

uint8_t makeWriteAddress(uint8_t reg) {
    return static_cast<uint8_t>((reg << 1) & 0x7E);
}

uint8_t makeReadAddress(uint8_t reg) {
    return static_cast<uint8_t>(((reg << 1) & 0x7E) | 0x80);
}

#endif

} // namespace

Rc522RfidProvider::Rc522RfidProvider(std::string spiDevice)
    : spiDevice_(std::move(spiDevice))
{}

Rc522RfidProvider::~Rc522RfidProvider() {
    stop();
}

void Rc522RfidProvider::setRfidCallback(RfidCallback cb) {
    cb_ = std::move(cb);
}

void Rc522RfidProvider::start() {
    // lifecycleMtx_ сериализует start()/stop() из любых потоков.
    std::lock_guard lifecycleLock(lifecycleMtx_);

    if (active_.load()) {
        return;
    }

    {
        std::lock_guard ioLock(ioMtx_);
        if (!openDevice()) {
            return;
        }
    }

    active_.store(true);
    thread_ = std::thread(&Rc522RfidProvider::pollLoop, this);
}

void Rc522RfidProvider::stop() {
    // Сериализация stop() из любых потоков. Без неё конкурентный двойной stop()
    // мог попасть на closeDevice() без join, что давало гонку с pollLoop по fd_.
    std::lock_guard lifecycleLock(lifecycleMtx_);

    active_.store(false);
    if (thread_.joinable()) {
        thread_.join();
    }
    // После join ни один поток не работает с SPI — безопасно закрывать fd.
    std::lock_guard ioLock(ioMtx_);
    closeDevice();
}

bool Rc522RfidProvider::isActive() const {
    return active_.load();
}

std::optional<std::string> Rc522RfidProvider::readOnce(int timeoutMs) {
    if (timeoutMs <= 0) {
        timeoutMs = 1000;
    }

    std::lock_guard lock(ioMtx_);
    if (!openDevice()) {
        return std::nullopt;
    }

    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(timeoutMs);

    std::string previousUid;
    int stableReads = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        if (auto uid = tryReadUid(); uid.has_value() && !uid->empty()) {
            if (*uid == previousUid) {
                ++stableReads;
            } else {
                previousUid = *uid;
                stableReads = 1;
            }

            if (stableReads >= 2) {
                return uid;
            }
        } else {
            previousUid.clear();
            stableReads = 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    return std::nullopt;
}

void Rc522RfidProvider::pollLoop() {
    std::string lastUid;
    std::string pendingUid;
    int stableReads = 0;

    while (active_.load()) {
        std::optional<std::string> uid;
        {
            std::lock_guard lock(ioMtx_);
            uid = tryReadUid();
        }

        if (uid.has_value() && !uid->empty()) {
            if (*uid == pendingUid) {
                ++stableReads;
            } else {
                pendingUid = *uid;
                stableReads = 1;
            }

            if (stableReads >= 2 && *uid != lastUid) {
                lastUid = *uid;
                if (cb_) {
                    cb_(*uid);
                }
            }
        } else {
            pendingUid.clear();
            stableReads = 0;
            lastUid.clear();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }
}

bool Rc522RfidProvider::openDevice() {
#ifdef __linux__
    if (fd_ >= 0) {
        return true;
    }

    fd_ = ::open(spiDevice_.c_str(), O_RDWR);
    if (fd_ < 0) {
        return false;
    }

    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t speed = kSpiSpeedHz;

    if (ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0 ||
        ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
        ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0)
    {
        closeDevice();
        return false;
    }

    if (!initializeChip()) {
        closeDevice();
        return false;
    }

    return true;
#else
    return false;
#endif
}

void Rc522RfidProvider::closeDevice() {
#ifdef __linux__
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
#endif
}

bool Rc522RfidProvider::initializeChip() {
#ifdef __linux__
    auto writeRegister = [this](uint8_t reg, uint8_t value) -> bool {
        const uint8_t tx[2] = {makeWriteAddress(reg), value};
        uint8_t rx[2] = {0, 0};
        return spiTransfer(fd_, tx, rx, sizeof(tx));
    };

    auto readRegister = [this](uint8_t reg) -> std::optional<uint8_t> {
        const uint8_t tx[2] = {makeReadAddress(reg), 0x00};
        uint8_t rx[2] = {0, 0};
        if (!spiTransfer(fd_, tx, rx, sizeof(tx))) {
            return std::nullopt;
        }
        return rx[1];
    };

    auto setBitMask = [&writeRegister, &readRegister](uint8_t reg,
                                                      uint8_t mask) -> bool {
        const auto value = readRegister(reg);
        return value.has_value() && writeRegister(reg, static_cast<uint8_t>(*value | mask));
    };

    if (!writeRegister(kCommandReg, kCmdSoftReset)) {
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (!writeRegister(kTModeReg, 0x8D) ||
        !writeRegister(kTPrescalerReg, 0x3E) ||
        !writeRegister(kTReloadRegL, 30) ||
        !writeRegister(kTReloadRegH, 0) ||
        !writeRegister(kTxASKReg, 0x40) ||
        !writeRegister(kModeReg, 0x3D) ||
        !writeRegister(kRFCfgReg, 0x70))
    {
        return false;
    }

    if (!setBitMask(kTxControlReg, 0x03)) {
        return false;
    }

    const auto version = readRegister(kVersionReg);
    return version.has_value() && *version != 0x00 && *version != 0xFF;
#else
    return false;
#endif
}

std::optional<std::string> Rc522RfidProvider::tryReadUid() {
#ifdef __linux__
    if (fd_ < 0) {
        return std::nullopt;
    }

    auto writeRegister = [this](uint8_t reg, uint8_t value) -> bool {
        const uint8_t tx[2] = {makeWriteAddress(reg), value};
        uint8_t rx[2] = {0, 0};
        return spiTransfer(fd_, tx, rx, sizeof(tx));
    };

    auto readRegister = [this](uint8_t reg) -> std::optional<uint8_t> {
        const uint8_t tx[2] = {makeReadAddress(reg), 0x00};
        uint8_t rx[2] = {0, 0};
        if (!spiTransfer(fd_, tx, rx, sizeof(tx))) {
            return std::nullopt;
        }
        return rx[1];
    };

    auto setBitMask = [&writeRegister, &readRegister](uint8_t reg,
                                                      uint8_t mask) -> bool {
        const auto value = readRegister(reg);
        return value.has_value() &&
               writeRegister(reg, static_cast<uint8_t>(*value | mask));
    };

    auto clearBitMask = [&writeRegister, &readRegister](uint8_t reg,
                                                        uint8_t mask) -> bool {
        const auto value = readRegister(reg);
        return value.has_value() &&
               writeRegister(reg, static_cast<uint8_t>(*value & ~mask));
    };

    auto readFifo = [&readRegister](int count) -> std::optional<std::vector<uint8_t>> {
        std::vector<uint8_t> data;
        data.reserve(static_cast<std::size_t>(count));
        for (int i = 0; i < count; ++i) {
            const auto value = readRegister(kFIFODataReg);
            if (!value.has_value()) {
                return std::nullopt;
            }
            data.push_back(*value);
        }
        return data;
    };

    // Сбрасываем рабочее состояние чипа перед каждым чтением, чтобы не тащить
    // "хвост" от предыдущей метки. Полный initializeChip() (с soft-reset и
    // 50ms sleep) не нужен на каждой итерации — он выполняется один раз в
    // openDevice(). Здесь только лёгкий per-read reset рабочих регистров.
    if (!writeRegister(kCommandReg, kCmdIdle) ||
        !writeRegister(kComIrqReg, 0x7F) ||
        !writeRegister(kFIFOLevelReg, 0x80) ||
        !writeRegister(kBitFramingReg, 0x00) ||
        !writeRegister(kControlReg, 0x00))
    {
        return std::nullopt;
    }

    auto transceive = [&](const std::vector<uint8_t>& data,
                          uint8_t bitFraming) -> std::optional<std::vector<uint8_t>> {
        if (!writeRegister(kCommandReg, kCmdIdle) ||
            !writeRegister(kComIrqReg, 0x7F) ||
            !writeRegister(kFIFOLevelReg, 0x80))
        {
            return std::nullopt;
        }

        for (const auto byte : data) {
            if (!writeRegister(kFIFODataReg, byte)) {
                return std::nullopt;
            }
        }

        if (!writeRegister(kBitFramingReg, bitFraming) ||
            !writeRegister(kCommandReg, kCmdTransceive) ||
            !setBitMask(kBitFramingReg, 0x80))
        {
            return std::nullopt;
        }

        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
        while (std::chrono::steady_clock::now() < deadline) {
            const auto irq = readRegister(kComIrqReg);
            if (!irq.has_value()) {
                return std::nullopt;
            }
            if ((*irq & 0x01U) != 0U) {
                return std::nullopt;
            }
            if ((*irq & 0x30U) != 0U) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        clearBitMask(kBitFramingReg, 0x80);

        const auto error = readRegister(kErrorReg);
        if (!error.has_value() || (*error & 0x13U) != 0U) {
            return std::nullopt;
        }

        const auto fifoLevel = readRegister(kFIFOLevelReg);
        if (!fifoLevel.has_value() || *fifoLevel == 0) {
            return std::nullopt;
        }

        return readFifo(*fifoLevel);
    };

    const auto atqa = transceive({kPiccReqA}, 0x07);
    if (!atqa.has_value() || atqa->size() < 2) {
        return std::nullopt;
    }

    const auto anticoll =
        transceive({kPiccAntiColl, kPiccAntiCollNvb}, 0x00);
    if (!anticoll.has_value() || anticoll->size() < 5) {
        return std::nullopt;
    }

    std::array<uint8_t, 4> uid{
        (*anticoll)[0],
        (*anticoll)[1],
        (*anticoll)[2],
        (*anticoll)[3]
    };

    const uint8_t bcc = static_cast<uint8_t>(
        uid[0] ^ uid[1] ^ uid[2] ^ uid[3]
    );
    if (bcc != (*anticoll)[4]) {
        return std::nullopt;
    }

    return uidToHex(uid);
#else
    return std::nullopt;
#endif
}

} // namespace smartcart::infrastructure::hw::rfid
