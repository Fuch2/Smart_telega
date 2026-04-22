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
                lastUnexpectedUid_ = uid;
                lastUnexpectedSeen_ = now;
                logSafe("WARN",
                        "RfidUnexpectedModule",
                        "Обнаружена чужая RFID-метка: uid=" + uid +
                        ", ожидается uid=" + config_.expectedUid);
            } else {
                const auto seenFor =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - lastUnexpectedSeen_);
                if (switchCb_ &&
                    lastUnexpectedUid_ != notifiedSwitchUid_ &&
                    seenFor.count() >= 800)
                {
                    notifiedSwitchUid_ = lastUnexpectedUid_;
                    switchCb_(lastUnexpectedUid_);
                }
            }
        } else if (moduleOnline_) {
            const auto offlineFor =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - lastSeen_);
            if (offlineFor.count() >= config_.offlineTimeoutMs) {
                setModuleOnline(false);
            }
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(config_.pollMs));
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

void RfidModuleMonitorService::logSafe(std::string_view level,
                                       std::string_view code,
                                       std::string_view message) const {
    try {
        eventLogger_.log(level, code, message);
    } catch (...) {
    }
}

} // namespace smartcart::application::services
