#pragma once

#include "application/ports/IEventLogger.hpp"
#include "application/ports/IModuleRepository.hpp"
#include "application/ports/IRfidProvider.hpp"

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
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
    bool isOnline() const noexcept { return moduleOnline_.load(std::memory_order_acquire); }
    std::chrono::system_clock::time_point lastSeen() const;
    std::string lastError() const;

private:
    ports::IRfidProvider& rfidProvider_;
    ports::IModuleRepository& moduleRepo_;
    ports::IEventLogger& eventLogger_;
    RfidModuleMonitorConfig config_;

    std::atomic<bool> running_{false};
    std::thread thread_;

    // moduleOnline_ — atomic, доступен из monitorLoop и из UI без блокировок.
    std::atomic<bool> moduleOnline_{false};

    // Поля, доступные одновременно из monitorLoop и из публичных геттеров,
    // защищены statusMtx_. Поля, видимые только monitorLoop, синхронизации
    // не требуют (lastUnexpectedUid_, notifiedSwitchUid_, и т.п.).
    mutable std::mutex statusMtx_;
    std::chrono::steady_clock::time_point lastSeen_{};
    std::string lastError_;

    // Только monitorLoop, без синхронизации.
    std::string lastUnexpectedUid_;
    std::chrono::steady_clock::time_point lastUnexpectedSeen_{};
    std::string notifiedSwitchUid_;
    std::chrono::steady_clock::time_point lastSwitchNotifyAt_{};

    // switchCb_ может ставиться из UI-потока и читаться из monitorLoop.
    // Отдельный мьютекс, чтобы не держать statusMtx_ во время вызова callback.
    mutable std::mutex   switchCbMtx_;
    ModuleSwitchCallback switchCb_;

    void monitorLoop();
    void markOfflineIfExpectedUidTimedOut(
        std::chrono::steady_clock::time_point now);
    void setModuleOnline(bool online);
    void logSafe(std::string_view level,
                 std::string_view code,
                 std::string_view message) const;
};

} // namespace smartcart::application::services
