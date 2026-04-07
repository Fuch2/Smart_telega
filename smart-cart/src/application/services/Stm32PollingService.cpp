#include "application/services/Stm32PollingService.hpp"
#include "infrastructure/hw/stm32/Protocol.hpp"
#include "domain/entities/Slot.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <sstream>
#include <string>
#include <utility>

namespace smartcart::application::services {

namespace domain = smartcart::domain;
namespace stm32  = smartcart::infrastructure::hw::stm32;

namespace {

std::vector<bool> parseSnapshotPayload(const std::vector<uint8_t>& payload,
                                       int slotCount) {
    std::vector<bool> result(slotCount, false);
    if (payload.size() < 3) {
        return {};
    }

    for (int channel = 0; channel < slotCount; ++channel) {
        const int byteIdx = channel / 8;
        const int bitIdx  = channel % 8;
        result[channel] = ((payload[byteIdx] >> bitIdx) & 0x01) != 0;
    }

    return result;
}

} // namespace

Stm32PollingService::Stm32PollingService(
    ports::IStm32Link&      link,
    ports::IReelRepository& reelRepo,
    ports::IEventLogger&    eventLogger,
    Stm32PollingConfig      config)
    : link_(link)
    , reelRepo_(reelRepo)
    , eventLogger_(eventLogger)
    , config_(std::move(config))
{}

Stm32PollingService::~Stm32PollingService() {
    stop();
}

void Stm32PollingService::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        return;
    }

    thread_ = std::thread(&Stm32PollingService::pollLoop, this);
}

void Stm32PollingService::stop() {
    if (!running_.exchange(false)) {
        return;
    }

    if (thread_.joinable()) {
        thread_.join();
    }
}

void Stm32PollingService::pollLoop() {
    logSafe("INFO", "Stm32PollingStarted", "Опрос STM32 запущен");

    while (running_.load()) {
        pollOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(config_.pollMs));
    }

    logSafe("INFO", "Stm32PollingStopped", "Опрос STM32 остановлен");
}

void Stm32PollingService::pollOnce() {
    try {
        const auto snapshot = requestSnapshot();
        if (!snapshot.has_value()) {
            logSafe("WARN",
                    "Stm32SnapshotUnavailable",
                    "Не удалось получить snapshot STM32");
            return;
        }

        applySnapshot(*snapshot);
    } catch (const std::exception& ex) {
        logSafe("ERROR", "Stm32PollingError", ex.what());
    } catch (...) {
        logSafe("ERROR", "Stm32PollingError", "Неизвестная ошибка polling");
    }
}

std::optional<std::vector<bool>> Stm32PollingService::requestSnapshot() {
    auto request = [this](stm32::CommandId cmdId)
        -> std::optional<std::vector<bool>>
    {
        stm32::Frame cmd;
        cmd.type  = stm32::FrameType::Cmd;
        cmd.cmdId = cmdId;

        const auto resp = link_.sendCommand(cmd);
        if (!resp.has_value() ||
            resp->type != stm32::FrameType::Resp ||
            resp->payload.size() < 3)
        {
            return std::nullopt;
        }

        auto snapshot = parseSnapshotPayload(resp->payload, config_.slotCount);
        if (static_cast<int>(snapshot.size()) != config_.slotCount) {
            return std::nullopt;
        }

        return snapshot;
    };

    if (auto snapshot = request(stm32::CommandId::GetSwitchSnapshot)) {
        return snapshot;
    }

    // Совместимость с текущим stm_query.py / ранней прошивкой.
    return request(static_cast<stm32::CommandId>(0x04));
}

void Stm32PollingService::applySnapshot(const std::vector<bool>& snapshot) {
    for (int channel = 0; channel < static_cast<int>(snapshot.size()); ++channel) {
        if (!isTrackedChannel(channel) || isIgnoredChannel(channel)) {
            continue;
        }

        const bool occupied = snapshot[channel];
        const bool changed =
            !lastSnapshot_.has_value() ||
            channel >= static_cast<int>(lastSnapshot_->size()) ||
            occupied != (*lastSnapshot_)[channel];

        if (!changed) {
            continue;
        }

        const int slotIndex = channel + 1;
        const bool saved = reelRepo_.setSlotState(
            config_.moduleId,
            slotIndex,
            occupied ? domain::SlotState::Occupied : domain::SlotState::Free
        );

        std::ostringstream msg;
        msg << "channel=" << channel
            << " slot=" << slotIndex
            << " occupied=" << (occupied ? "true" : "false");

        logSafe(saved ? "INFO" : "ERROR",
                saved ? "SwitchChanged" : "SwitchStateSaveFailed",
                msg.str());
    }

    lastSnapshot_ = snapshot;
}

bool Stm32PollingService::isIgnoredChannel(int channel) const {
    return std::find(config_.ignoredChannels.begin(),
                     config_.ignoredChannels.end(),
                     channel) != config_.ignoredChannels.end();
}

bool Stm32PollingService::isTrackedChannel(int channel) const {
    return std::find(config_.trackedChannels.begin(),
                     config_.trackedChannels.end(),
                     channel) != config_.trackedChannels.end();
}

void Stm32PollingService::logSafe(std::string_view level,
                                  std::string_view code,
                                  std::string_view message) const {
    try {
        eventLogger_.log(level, code, message);
    } catch (...) {
        // Логирование не должно останавливать обмен с STM32.
    }
}

} // namespace smartcart::application::services
