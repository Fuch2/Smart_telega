#include "application/services/AdminDiagnosticsService.hpp"

#include <algorithm>
#include <utility>

namespace smartcart::application::services {

AdminDiagnosticsService::AdminDiagnosticsService(
    Stm32PollingService& pollingSvc,
    ports::IDiagnosticsRepository& diagnosticsRepo,
    AdminDiagnosticsConfig config)
    : pollingSvc_(pollingSvc)
    , diagnosticsRepo_(diagnosticsRepo)
    , config_(std::move(config))
{}

AdminDiagnosticsSnapshot AdminDiagnosticsService::snapshot(int eventLimit) {
    AdminDiagnosticsSnapshot result;
    result.stm32 = pollingSvc_.connectionStatus();
    result.trackedChannels = config_.trackedChannels;
    result.ignoredChannels = config_.ignoredChannels;
    result.channelToSlotMap = config_.channelToSlotMap;
    result.recentEvents = diagnosticsRepo_.recentEvents(std::clamp(eventLimit, 1, 200));
    return result;
}

bool AdminDiagnosticsService::resetTestCart() {
    return diagnosticsRepo_.resetTestCart(config_.moduleId);
}

} // namespace smartcart::application::services
