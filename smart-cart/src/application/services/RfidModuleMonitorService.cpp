#include "application/services/RfidModuleMonitorService.hpp"

#include "domain/entities/ModuleInfo.hpp"

#include <algorithm>
#include <sstream>
#include <utility>

namespace smartcart::application::services {

namespace domain = smartcart::domain;

RfidModuleMonitorService::RfidModuleMonitorService(
    ports::IRfidProvider& rfidProvider,
    ports::IModuleRepository& moduleRepo,
    ports::IEventLogger& eventLogger,
    RfidModuleMonitorConfig config)
    : rfidProvider_(rfidProvider)
    , moduleRepo_(moduleRepo)
    , eventLogger_(eventLogger)
    , config_(std::move(config))
{}

RfidModuleMonitorService::~RfidModuleMonitorService() {
    stop();
}

void RfidModuleMonitorService::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        return;
    }

    {
        std::lock_guard lock(statusMtx_);
        lastSeen_ = std::chrono::steady_clock::now();
    }

    const auto module = moduleRepo_.getById(config_.moduleId);
    moduleOnline_.store(
        module.has_value() && module->status == domain::ModuleStatus::Online,
        std::memory_order_release);

    thread_ = std::thread(&RfidModuleMonitorService::monitorLoop, this);
}

void RfidModuleMonitorService::setModuleSwitchCallback(ModuleSwitchCallback cb) {
    std::lock_guard lock(switchCbMtx_);
    switchCb_ = std::move(cb);
}

void RfidModuleMonitorService::stop() {
    if (!running_.exchange(false)) {
        return;
    }

    if (thread_.joinable()) {
        thread_.join();
    }
}

void RfidModuleMonitorService::monitorLoop() {
    // Получаем текущий callback через защищённый геттер.
    // Локальная переменная — мы не держим switchCbMtx_ во время вызова callback,
    // чтобы не блокировать setModuleSwitchCallback из UI-потока.
    auto invokeSwitchCb = [this](const std::string& uid) {
        ModuleSwitchCallback cb;
        {
            std::lock_guard lock(switchCbMtx_);
            cb = switchCb_;
        }
        if (cb) cb(uid);
    };
    auto hasSwitchCb = [this]() {
        std::lock_guard lock(switchCbMtx_);
        return static_cast<bool>(switchCb_);
    };

    while (running_.load()) {
        const auto uids = rfidProvider_.readAllOnce(config_.readTimeoutMs);
        const auto now = std::chrono::steady_clock::now();

        if (!uids.empty()) {
            if (config_.expectedUid.empty()) {
                const auto& uid = uids.front();
                const auto retryFor =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - lastSwitchNotifyAt_);
                if (hasSwitchCb() &&
                    (uid != notifiedSwitchUid_ || retryFor.count() >= 2500))
                {
                    notifiedSwitchUid_ = uid;
                    lastSwitchNotifyAt_ = now;
                    logSafe("INFO",
                            "RfidModuleSwitchRequested",
                            "uid=" + uid + " reason=no_active_uid");
                    invokeSwitchCb(uid);
                }
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(config_.pollMs));
                continue;
            }

            const auto matchingUid = std::find(
                uids.begin(),
                uids.end(),
                config_.expectedUid);

            if (matchingUid != uids.end()) {
                {
                    std::lock_guard lock(statusMtx_);
                    lastSeen_ = now;
                }
                lastUnexpectedUid_.clear();
                notifiedSwitchUid_.clear();
                if (!moduleOnline_.load(std::memory_order_acquire)) {
                    setModuleOnline(true);
                }
            } else if (const auto& uid = uids.front(); uid != lastUnexpectedUid_) {
                markOfflineIfExpectedUidTimedOut(now);
                lastUnexpectedUid_ = uid;
                lastUnexpectedSeen_ = now;
                logSafe("WARN",
                        "RfidUnexpectedModule",
                        "Обнаружена чужая RFID-метка: uid=" + uid +
                        ", ожидается uid=" + config_.expectedUid);
            } else {
                markOfflineIfExpectedUidTimedOut(now);
                const auto seenFor =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - lastUnexpectedSeen_);
                const auto retryFor =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - lastSwitchNotifyAt_);
                if (hasSwitchCb() &&
                    (lastUnexpectedUid_ != notifiedSwitchUid_ ||
                     retryFor.count() >= 2500) &&
                    seenFor.count() >= 800)
                {
                    notifiedSwitchUid_ = lastUnexpectedUid_;
                    lastSwitchNotifyAt_ = now;
                    logSafe("INFO",
                            "RfidModuleSwitchRequested",
                            "uid=" + lastUnexpectedUid_ +
                                " reason=unexpected_uid");
                    invokeSwitchCb(lastUnexpectedUid_);
                }
            }
        } else if (config_.expectedUid.empty()) {
            notifiedSwitchUid_.clear();
            lastUnexpectedUid_.clear();
        } else {
            markOfflineIfExpectedUidTimedOut(now);
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(config_.pollMs));
    }
}

void RfidModuleMonitorService::markOfflineIfExpectedUidTimedOut(
    std::chrono::steady_clock::time_point now)
{
    if (!moduleOnline_.load(std::memory_order_acquire) ||
        config_.expectedUid.empty())
    {
        return;
    }

    std::chrono::steady_clock::time_point lastSeenCopy;
    {
        std::lock_guard lock(statusMtx_);
        lastSeenCopy = lastSeen_;
    }

    const auto offlineFor =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            now - lastSeenCopy);
    if (offlineFor.count() >= config_.offlineTimeoutMs) {
        setModuleOnline(false);
    }
}

void RfidModuleMonitorService::setModuleOnline(bool online) {
    auto module = moduleRepo_.getById(config_.moduleId);
    if (!module.has_value()) {
        return;
    }

    const auto newStatus =
        online ? domain::ModuleStatus::Online : domain::ModuleStatus::Offline;
    if (module->status == newStatus) {
        moduleOnline_.store(online, std::memory_order_release);
        return;
    }

    module->status = newStatus;
    if (!moduleRepo_.update(*module)) {
        return;
    }

    moduleOnline_.store(online, std::memory_order_release);

    // Update health status
    {
        std::lock_guard lock(statusMtx_);
        if (online) {
            lastError_.clear();
        } else {
            lastError_ = "RFID module offline (timeout)";
        }
    }

    if (online) {
        logSafe("INFO",
                "RfidModuleOnline",
                "module_id=" + std::to_string(config_.moduleId) +
                " uid=" + config_.expectedUid);
    } else {
        logSafe("WARN",
                "RfidModuleOffline",
                "module_id=" + std::to_string(config_.moduleId) +
                " uid=" + config_.expectedUid);
    }
}

std::chrono::system_clock::time_point RfidModuleMonitorService::lastSeen() const {
    // Convert steady_clock to system_clock (approximate).
    // lastSeen_ читается под statusMtx_, чтобы исключить data race с monitorLoop.
    std::chrono::steady_clock::time_point lastSeenCopy;
    {
        std::lock_guard lock(statusMtx_);
        lastSeenCopy = lastSeen_;
    }
    const auto steadyNow = std::chrono::steady_clock::now();
    const auto systemNow = std::chrono::system_clock::now();
    const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(steadyNow - lastSeenCopy);
    return systemNow - elapsed;
}

std::string RfidModuleMonitorService::lastError() const {
    std::lock_guard lock(statusMtx_);
    return lastError_;
}

void RfidModuleMonitorService::logSafe(std::string_view level,
                                       std::string_view code,
                                       std::string_view message) const {
    try {
        eventLogger_.log(level, code, message);
    } catch (...) {
    }
}

} // namespace smartcart::application::services
