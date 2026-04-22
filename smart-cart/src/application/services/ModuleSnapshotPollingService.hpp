#pragma once

#include "application/ports/IEventLogger.hpp"
#include "application/ports/IModuleRepository.hpp"
#include "application/ports/IReelRepository.hpp"
#include "application/ports/IStm32Link.hpp"

#include <atomic>
#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace smartcart::application::services {

struct ModuleSnapshotPollingConfig {
    int moduleId{1};
    int slotCount{24};
    int pollMs{500};
    int debounceMs{50};
    std::string channelName;
    std::string stm32Device;

    std::vector<int> trackedChannels{1, 3};
    std::vector<int> ignoredChannels{11};
    std::vector<int> channelToSlotMap;
};

class ModuleSnapshotPollingService {
public:
    ModuleSnapshotPollingService(
        ports::IStm32Link& link,
        ports::IModuleRepository& moduleRepo,
        ports::IReelRepository& reelRepo,
        ports::IEventLogger& eventLogger,
        ModuleSnapshotPollingConfig config
    );
    ~ModuleSnapshotPollingService();

    ModuleSnapshotPollingService(const ModuleSnapshotPollingService&) = delete;
    ModuleSnapshotPollingService& operator=(const ModuleSnapshotPollingService&) = delete;

    void start();
    void stop();
    void pollOnce();
    void handleEventFrame(const smartcart::infrastructure::hw::stm32::Frame& frame);
    bool isRunning() const noexcept { return running_.load(); }

private:
    ports::IStm32Link& link_;
    ports::IModuleRepository& moduleRepo_;
    ports::IReelRepository& reelRepo_;
    ports::IEventLogger& eventLogger_;
    ModuleSnapshotPollingConfig config_;

    std::atomic<bool> running_{false};
    std::thread thread_;
    std::mutex stateMtx_;
    std::optional<std::vector<bool>> stableSnapshot_;
    std::optional<std::vector<bool>> rawSnapshot_;
    std::vector<std::chrono::steady_clock::time_point> rawChangedAt_;
    int failedSnapshots_{0};

    void pollLoop();
    std::optional<std::vector<bool>> requestSnapshot();
    void applySnapshot(const std::vector<bool>& snapshot);
    void applyChannelStateLocked(int channel,
                                 bool occupied,
                                 std::string_view source);
    bool isIgnoredChannel(int channel) const;
    bool isTrackedChannel(int channel) const;
    std::optional<int> slotIndexForChannel(int channel) const;
    void setModuleOnline(bool online);
    void logSafe(std::string_view level,
                 std::string_view code,
                 std::string_view message) const;
};

} // namespace smartcart::application::services
