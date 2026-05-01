#include "application/services/SwitchEventHandler.hpp"
#include "domain/entities/Slot.hpp"

#include <algorithm>
#include <sstream>

namespace smartcart::application::services {

namespace domain = smartcart::domain;
namespace stm32 = smartcart::infrastructure::hw::stm32;

SwitchEventHandler::SwitchEventHandler(
    ports::IReelRepository& reelRepo,
    ports::IEventLogger& eventLogger,
    SwitchEventConfig config)
    : reelRepo_(reelRepo)
    , eventLogger_(eventLogger)
    , config_(std::move(config))
{}

void SwitchEventHandler::handleEvent(
    const stm32::Frame& frame,
    std::vector<bool>& lastSnapshot,
    std::vector<bool>& rawSnapshot,
    std::vector<std::chrono::steady_clock::time_point>& rawChangedAt)
{
    if (frame.type != stm32::FrameType::Evt ||
        frame.cmdId != stm32::CommandId::EvtSwitchChanged)
    {
        return;
    }

    if (frame.payload.size() < 2) {
        logSafe("WARN",
                "Stm32EventBadPayload",
                "EvtSwitchChanged payload слишком короткий");
        return;
    }

    const int channel = static_cast<int>(frame.payload[0]);
    const bool occupied = frame.payload[1] != 0;

    if (channel < 0 || channel >= config_.slotCount) {
        std::ostringstream msg;
        msg << "channel=" << channel;
        logSafe("WARN", "Stm32EventInvalidChannel", msg.str());
        return;
    }

    if (!isTrackedChannel(channel) ||
        isIgnoredChannel(channel) ||
        !slotIndexForChannel(channel).has_value())
    {
        return;
    }

    const auto now = std::chrono::steady_clock::now();

    // Initialize snapshots if needed
    if (lastSnapshot.size() != static_cast<size_t>(config_.slotCount)) {
        lastSnapshot.assign(config_.slotCount, false);
        rawSnapshot.assign(config_.slotCount, false);
        rawChangedAt.assign(config_.slotCount, now);
    }

    if (lastSnapshot[channel] == occupied) {
        return; // No change
    }

    lastSnapshot[channel] = occupied;
    rawSnapshot[channel] = occupied;
    rawChangedAt[channel] = now;

    applyChannelState(channel, occupied);
}

bool SwitchEventHandler::isTrackedChannel(int channel) const {
    if (config_.trackedChannels.empty()) {
        return true;
    }
    return std::find(config_.trackedChannels.begin(),
                     config_.trackedChannels.end(),
                     channel) != config_.trackedChannels.end();
}

bool SwitchEventHandler::isIgnoredChannel(int channel) const {
    return std::find(config_.ignoredChannels.begin(),
                     config_.ignoredChannels.end(),
                     channel) != config_.ignoredChannels.end();
}

std::optional<int> SwitchEventHandler::slotIndexForChannel(int channel) const {
    if (config_.channelToSlotMap.empty()) {
        return channel;
    }
    if (channel < 0 || channel >= static_cast<int>(config_.channelToSlotMap.size())) {
        return std::nullopt;
    }
    return config_.channelToSlotMap[channel];
}

void SwitchEventHandler::applyChannelState(int channel, bool occupied) {
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

    if (!saved) {
        return;
    }

    if (occupied) {
        handleOccupiedSlot(slotIndex);
    } else {
        handleFreedSlot(slotIndex);
    }
}

void SwitchEventHandler::handleOccupiedSlot(int slotIndex) {
    std::ostringstream msg;
    msg << "slot=" << slotIndex << " state=occupied";
    logSafe("INFO", "SlotOccupied", msg.str());
}

void SwitchEventHandler::handleFreedSlot(int slotIndex) {
    std::ostringstream msg;
    msg << "slot=" << slotIndex << " state=free";
    logSafe("INFO", "SlotFreed", msg.str());
}

void SwitchEventHandler::logSafe(std::string_view level,
                                  std::string_view code,
                                  std::string_view message) const {
    try {
        eventLogger_.log(level, code, message);
    } catch (...) {
        // Ошибка логирования не должна ломать обработку событий
    }
}

} // namespace smartcart::application::services
