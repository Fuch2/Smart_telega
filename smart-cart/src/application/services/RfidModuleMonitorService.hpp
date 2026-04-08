#pragma once

#include "application/ports/IEventLogger.hpp"
#include "application/ports/IModuleRepository.hpp"
#include "application/ports/IRfidProvider.hpp"

#include <atomic>
#include <chrono>
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

    void monitorLoop();
    void setModuleOnline(bool online);
    void logSafe(std::string_view level,
                 std::string_view code,
                 std::string_view message) const;
};

} // namespace smartcart::application::services
