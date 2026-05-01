#pragma once

#include "application/ports/IEventLogger.hpp"
#include "application/ports/IReelRepository.hpp"
#include "infrastructure/hw/stm32/Protocol.hpp"

#include <chrono>
#include <optional>
#include <vector>

namespace smartcart::application::services {

struct SwitchEventConfig {
    int moduleId = 1;
    int slotCount = 24;
    std::vector<int> trackedChannels;
    std::vector<int> ignoredChannels;
    std::vector<int> channelToSlotMap;
};

class SwitchEventHandler {
public:
    SwitchEventHandler(ports::IReelRepository& reelRepo,
                       ports::IEventLogger& eventLogger,
                       SwitchEventConfig config);

    void handleEvent(const smartcart::infrastructure::hw::stm32::Frame& frame,
                     std::vector<bool>& lastSnapshot,
                     std::vector<bool>& rawSnapshot,
                     std::vector<std::chrono::steady_clock::time_point>& rawChangedAt);

private:
    ports::IReelRepository& reelRepo_;
    ports::IEventLogger& eventLogger_;
    SwitchEventConfig config_;

    bool isTrackedChannel(int channel) const;
    bool isIgnoredChannel(int channel) const;
    std::optional<int> slotIndexForChannel(int channel) const;
    void applyChannelState(int channel, bool occupied);
    void handleOccupiedSlot(int slotIndex);
    void handleFreedSlot(int slotIndex);
    void logSafe(std::string_view level,
                 std::string_view code,
                 std::string_view message) const;
};

} // namespace smartcart::application::services
