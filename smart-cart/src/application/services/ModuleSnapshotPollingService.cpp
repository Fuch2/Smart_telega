#include "application/services/ModuleSnapshotPollingService.hpp"

#include "domain/entities/ModuleInfo.hpp"
#include "domain/entities/Slot.hpp"
#include "infrastructure/hw/stm32/Protocol.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <sstream>
#include <utility>

namespace smartcart::application::services {

namespace domain = smartcart::domain;
namespace stm32 = smartcart::infrastructure::hw::stm32;

namespace {

std::vector<bool> parseSnapshotPayload(const std::vector<uint8_t>& payload,
                                       int slotCount) {
    std::vector<bool> result(slotCount, false);
    if (payload.size() < 3) {
        return {};
    }

    for (int channel = 0; channel < slotCount; ++channel) {
        const int byteIdx = channel / 8;
        const int bitIdx = channel % 8;
        result[channel] = ((payload[byteIdx] >> bitIdx) & 0x01) != 0;
    }

    return result;
}

} // namespace

ModuleSnapshotPollingService::ModuleSnapshotPollingService(
    ports::IStm32Link& link,
    ports::IModuleRepository& moduleRepo,
    ports::IReelRepository& reelRepo,
    ports::IEventLogger& eventLogger,
    ModuleSnapshotPollingConfig config)
    : link_(link)
    , moduleRepo_(moduleRepo)
    , reelRepo_(reelRepo)
    , eventLogger_(eventLogger)
    , config_(std::move(config))
{}

ModuleSnapshotPollingService::~ModuleSnapshotPollingService() {
    stop();
}

void ModuleSnapshotPollingService::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        return;
    }

    thread_ = std::thread(&ModuleSnapshotPollingService::pollLoop, this);
}

void ModuleSnapshotPollingService::stop() {
    if (!running_.exchange(false)) {
        return;
    }

    if (thread_.joinable()) {
        thread_.join();
    }
}

void ModuleSnapshotPollingService::pollLoop() {
    logSafe("INFO",
            "ModuleStm32PollingStarted",
            "module_id=" + std::to_string(config_.moduleId) +
                " channel=" + config_.channelName +
                " device=" + config_.stm32Device);

    while (running_.load()) {
        pollOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(config_.pollMs));
    }

    logSafe("INFO",
            "ModuleStm32PollingStopped",
            "module_id=" + std::to_string(config_.moduleId) +
                " channel=" + config_.channelName);
}

void ModuleSnapshotPollingService::pollOnce() {
    try {
        const auto snapshot = requestSnapshot();
        if (!snapshot.has_value()) {
            ++failedSnapshots_;
            if (failedSnapshots_ >= 3) {
                setModuleOnline(false);
            }
            logSafe("WARN",
                    "ModuleSnapshotUnavailable",
                    "module_id=" + std::to_string(config_.moduleId) +
                        " channel=" + config_.channelName);
            return;
        }

        failedSnapshots_ = 0;
        setModuleOnline(true);
        applySnapshot(*snapshot);
    } catch (const std::exception& ex) {
        ++failedSnapshots_;
        if (failedSnapshots_ >= 3) {
            setModuleOnline(false);
        }
        logSafe("ERROR", "ModuleSnapshotPollingError", ex.what());
    } catch (...) {
        ++failedSnapshots_;
        if (failedSnapshots_ >= 3) {
            setModuleOnline(false);
        }
        logSafe("ERROR",
                "ModuleSnapshotPollingError",
                "Неизвестная ошибка polling");
    }
}

void ModuleSnapshotPollingService::handleEventFrame(const stm32::Frame& frame) {
    if (frame.type != stm32::FrameType::Evt ||
        frame.cmdId != stm32::CommandId::EvtSwitchChanged ||
        frame.payload.size() < 2)
    {
        return;
    }

    const int channel = static_cast<int>(frame.payload[0]);
    const bool occupied = frame.payload[1] != 0;

    if (!isTrackedChannel(channel) ||
        isIgnoredChannel(channel) ||
        !slotIndexForChannel(channel).has_value())
    {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    std::lock_guard lock(stateMtx_);
    if (!stableSnapshot_.has_value() ||
        static_cast<int>(stableSnapshot_->size()) != config_.slotCount)
    {
        stableSnapshot_ = std::vector<bool>(config_.slotCount, false);
        rawSnapshot_ = *stableSnapshot_;
        rawChangedAt_.assign(static_cast<size_t>(config_.slotCount), now);
    }

    if ((*stableSnapshot_)[channel] == occupied) {
        return;
    }

    (*stableSnapshot_)[channel] = occupied;
    if (!rawSnapshot_.has_value() ||
        static_cast<int>(rawSnapshot_->size()) != config_.slotCount)
    {
        rawSnapshot_ = *stableSnapshot_;
    }
    (*rawSnapshot_)[channel] = occupied;
    if (rawChangedAt_.size() != static_cast<size_t>(config_.slotCount)) {
        rawChangedAt_.assign(static_cast<size_t>(config_.slotCount), now);
    }
    rawChangedAt_[channel] = now;

    setModuleOnline(true);
    applyChannelStateLocked(channel, occupied, "event");
}

std::optional<std::vector<bool>> ModuleSnapshotPollingService::requestSnapshot() {
    auto request = [this](stm32::CommandId cmdId)
        -> std::optional<std::vector<bool>>
    {
        stm32::Frame cmd;
        cmd.type = stm32::FrameType::Cmd;
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

    return request(static_cast<stm32::CommandId>(0x04));
}

void ModuleSnapshotPollingService::applySnapshot(const std::vector<bool>& snapshot) {
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard lock(stateMtx_);

    if (!stableSnapshot_.has_value() || stableSnapshot_->size() != snapshot.size()) {
        stableSnapshot_ = snapshot;
        rawSnapshot_ = snapshot;
        rawChangedAt_.assign(snapshot.size(), now);

        for (int channel = 0; channel < static_cast<int>(snapshot.size()); ++channel) {
            if (!isTrackedChannel(channel) ||
                isIgnoredChannel(channel) ||
                !slotIndexForChannel(channel).has_value())
            {
                continue;
            }
            applyChannelStateLocked(channel, snapshot[channel], "snapshot");
        }
        return;
    }

    if (!rawSnapshot_.has_value() || rawSnapshot_->size() != snapshot.size()) {
        rawSnapshot_ = *stableSnapshot_;
        rawChangedAt_.assign(snapshot.size(), now);
    }
    if (rawChangedAt_.size() != snapshot.size()) {
        rawChangedAt_.assign(snapshot.size(), now);
    }

    for (int channel = 0; channel < static_cast<int>(snapshot.size()); ++channel) {
        if (!isTrackedChannel(channel) ||
            isIgnoredChannel(channel) ||
            !slotIndexForChannel(channel).has_value())
        {
            continue;
        }

        const bool occupied = snapshot[channel];
        if (occupied != (*rawSnapshot_)[channel]) {
            (*rawSnapshot_)[channel] = occupied;
            rawChangedAt_[channel] = now;
        }

        if (occupied == (*stableSnapshot_)[channel]) {
            continue;
        }

        const auto stableFor = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - rawChangedAt_[channel]).count();
        if (config_.debounceMs > 0 && stableFor < config_.debounceMs) {
            continue;
        }

        (*stableSnapshot_)[channel] = occupied;
        applyChannelStateLocked(channel, occupied, "snapshot");
    }
}

void ModuleSnapshotPollingService::applyChannelStateLocked(int channel,
                                                           bool occupied,
                                                           std::string_view source) {
    const auto mappedSlot = slotIndexForChannel(channel);
    if (!mappedSlot.has_value()) {
        return;
    }

    const int slotIndex = *mappedSlot;
    const bool saved = reelRepo_.setSlotState(
        config_.moduleId,
        slotIndex,
        occupied ? domain::SlotState::Occupied : domain::SlotState::Free
    );

    std::ostringstream msg;
    msg << "module_id=" << config_.moduleId
        << " channel=" << config_.channelName
        << " device=" << config_.stm32Device
        << " input_channel=" << channel
        << " slot=" << slotIndex
        << " occupied=" << (occupied ? "true" : "false")
        << " source=" << source;

    logSafe(saved ? "INFO" : "ERROR",
            saved ? "ModuleSwitchChanged" : "ModuleSwitchStateSaveFailed",
            msg.str());
}

bool ModuleSnapshotPollingService::isIgnoredChannel(int channel) const {
    return std::find(config_.ignoredChannels.begin(),
                     config_.ignoredChannels.end(),
                     channel) != config_.ignoredChannels.end();
}

bool ModuleSnapshotPollingService::isTrackedChannel(int channel) const {
    return std::find(config_.trackedChannels.begin(),
                     config_.trackedChannels.end(),
                     channel) != config_.trackedChannels.end();
}

std::optional<int> ModuleSnapshotPollingService::slotIndexForChannel(int channel) const {
    if (channel < 0 || channel >= config_.slotCount) {
        return std::nullopt;
    }

    if (channel < static_cast<int>(config_.channelToSlotMap.size())) {
        const int slotIndex = config_.channelToSlotMap[channel];
        if (slotIndex > 0 && slotIndex <= config_.slotCount) {
            return slotIndex;
        }
        return std::nullopt;
    }

    return channel + 1;
}

void ModuleSnapshotPollingService::setModuleOnline(bool online) {
    auto module = moduleRepo_.getById(config_.moduleId);
    if (!module.has_value()) {
        return;
    }

    const auto newStatus =
        online ? domain::ModuleStatus::Online : domain::ModuleStatus::Offline;
    if (module->status == newStatus) {
        return;
    }

    module->status = newStatus;
    if (!moduleRepo_.update(*module)) {
        return;
    }

    logSafe(online ? "INFO" : "WARN",
            online ? "ModuleStm32Online" : "ModuleStm32Offline",
            "module_id=" + std::to_string(config_.moduleId) +
                " channel=" + config_.channelName +
                " device=" + config_.stm32Device);
}

void ModuleSnapshotPollingService::logSafe(std::string_view level,
                                           std::string_view code,
                                           std::string_view message) const {
    try {
        eventLogger_.log(level, code, message);
    } catch (...) {
    }
}

} // namespace smartcart::application::services
