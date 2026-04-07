#pragma once

#include "application/ports/IEventLogger.hpp"
#include "application/ports/IReelRepository.hpp"
#include "application/ports/IStm32Link.hpp"

#include <atomic>
#include <optional>
#include <string_view>
#include <thread>
#include <vector>

namespace smartcart::application::services {

struct Stm32PollingConfig {
    int moduleId  = 1;
    int slotCount = 24;
    int pollMs    = 500;

    // На текущей плате реально подключены PA1/PA2.
    std::vector<int> trackedChannels {1, 3};
    std::vector<int> ignoredChannels {11};
};

class Stm32PollingService {
public:
    Stm32PollingService(
        ports::IStm32Link&      link,
        ports::IReelRepository& reelRepo,
        ports::IEventLogger&    eventLogger,
        Stm32PollingConfig      config
    );
    ~Stm32PollingService();

    Stm32PollingService(const Stm32PollingService&) = delete;
    Stm32PollingService& operator=(const Stm32PollingService&) = delete;

    void start();
    void stop();
    bool isRunning() const noexcept { return running_.load(); }

private:
    ports::IStm32Link&      link_;
    ports::IReelRepository& reelRepo_;
    ports::IEventLogger&    eventLogger_;
    Stm32PollingConfig      config_;

    std::atomic<bool> running_ {false};
    std::thread       thread_;

    std::optional<std::vector<bool>> lastSnapshot_;

    void pollLoop();
    void pollOnce();
    std::optional<std::vector<bool>> requestSnapshot();
    void applySnapshot(const std::vector<bool>& snapshot);

    bool isIgnoredChannel(int channel) const;
    bool isTrackedChannel(int channel) const;
    void logSafe(std::string_view level,
                 std::string_view code,
                 std::string_view message) const;
};

} // namespace smartcart::application::services
