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

    lastSeen_ = std::chrono::steady_clock::now();
    const auto module = moduleRepo_.getById(config_.moduleId);
    moduleOnline_ =
        module.has_value() &&
        module->status == domain::ModuleStatus::Online;
    thread_ = std::thread(&RfidModuleMonitorService::monitorLoop, this);
}

void RfidModuleMonitorService::setModuleSwitchCallback(ModuleSwitchCallback cb) {
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
    while (running_.load()) {
        const auto uids = rfidProvider_.readAllOnce(config_.readTimeoutMs);
        const auto now = std::chrono::steady_clock::now();

        if (!uids.empty()) {
            if (config_.expectedUid.empty()) {
                const auto& uid = uids.front();
                const auto retryFor =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - lastSwitchNotifyAt_);
                if (switchCb_ &&
                    (uid != notifiedSwitchUid_ || retryFor.count() >= 2500))
                {
                    notifiedSwitchUid_ = uid;
                    lastSwitchNotifyAt_ = now;
                    logSafe("INFO",
                            "RfidModuleSwitchRequested",
                            "uid=" + uid + " reason=no_active_uid");
                    switchCb_(uid);
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
                lastSeen_ = now;
                lastUnexpectedUid_.clear();
                notifiedSwitchUid_.clear();
                if (!moduleOnline_) {
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
                if (switchCb_ &&
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
                    switchCb_(lastUnexpectedUid_);
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
    if (!moduleOnline_ || config_.expectedUid.empty()) {
        return;
    }

    const auto offlineFor =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            now - lastSeen_);
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
        moduleOnline_ = online;
        return;
    }

    module->status = newStatus;
    if (!moduleRepo_.update(*module)) {
        return;
    }

    moduleOnline_ = online;

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
    // Convert steady_clock to system_clock (approximate)
    const auto steadyNow = std::chrono::steady_clock::now();
    const auto systemNow = std::chrono::system_clock::now();
    const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(steadyNow - lastSeen_);
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
