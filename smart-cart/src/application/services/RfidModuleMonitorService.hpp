#pragma once

#include "application/ports/IEventLogger.hpp"
#include "application/ports/IModuleRepository.hpp"
#include "application/ports/IRfidProvider.hpp"

#include <atomic>
#include <chrono>
#include <functional>
#include <string>
#include <thread>

namespace smartcart::application::services {

struct RfidModuleMonitorConfig {
    int moduleId{1};
    int pollMs{500};
    int readTimeoutMs{350};
    int offlineTimeoutMs{3000};
    std::string expectedUid;
};

class RfidModuleMonitorService {
public:
    using ModuleSwitchCallback = std::function<void(std::string uid)>;

    RfidModuleMonitorService(ports::IRfidProvider& rfidProvider,
                             ports::IModuleRepository& moduleRepo,
                             ports::IEventLogger& eventLogger,
                             RfidModuleMonitorConfig config);
    ~RfidModuleMonitorService();

    RfidModuleMonitorService(const RfidModuleMonitorService&) = delete;
    RfidModuleMonitorService& operator=(const RfidModuleMonitorService&) = delete;

    void start();
    void stop();
    bool isRunning() const noexcept { return running_.load(); }
    void setModuleSwitchCallback(ModuleSwitchCallback cb);

    // Health status
    bool isOnline() const noexcept { return moduleOnline_; }
    std::chrono::system_clock::time_point lastSeen() const;
    std::string lastError() const;

private:
    ports::IRfidProvider& rfidProvider_;
    ports::IModuleRepository& moduleRepo_;
    ports::IEventLogger& eventLogger_;
    RfidModuleMonitorConfig config_;

    std::atomic<bool> running_{false};
    std::thread thread_;
    bool moduleOnline_{false};
    std::chrono::steady_clock::time_point lastSeen_{};
    std::string lastUnexpectedUid_;
    std::chrono::steady_clock::time_point lastUnexpectedSeen_{};
    std::string notifiedSwitchUid_;
    std::chrono::steady_clock::time_point lastSwitchNotifyAt_{};
    ModuleSwitchCallback switchCb_;
    mutable std::mutex statusMtx_;
    std::string lastError_;

    void monitorLoop();
    void markOfflineIfExpectedUidTimedOut(
        std::chrono::steady_clock::time_point now);
    void setModuleOnline(bool online);
    void logSafe(std::string_view level,
                 std::string_view code,
                 std::string_view message) const;
};

} // namespace smartcart::application::services
