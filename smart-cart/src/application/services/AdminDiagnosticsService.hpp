#pragma once

#include "application/ports/IDiagnosticsRepository.hpp"
#include "application/services/Stm32PollingService.hpp"

#include <string>
#include <vector>

namespace smartcart::application::services {

struct AdminDiagnosticsConfig {
    int moduleId{1};
    std::vector<int> trackedChannels;
    std::vector<int> ignoredChannels;
    std::vector<int> channelToSlotMap;
};

struct AdminDiagnosticsSnapshot {
    Stm32ConnectionStatus stm32;
    std::vector<int> trackedChannels;
    std::vector<int> ignoredChannels;
    std::vector<int> channelToSlotMap;
    std::vector<ports::EventLogRecord> recentEvents;
};

class AdminDiagnosticsService {
public:
    AdminDiagnosticsService(Stm32PollingService& pollingSvc,
                            ports::IDiagnosticsRepository& diagnosticsRepo,
                            AdminDiagnosticsConfig config);

    AdminDiagnosticsSnapshot snapshot(int eventLimit = 30);
    bool resetTestCart();

private:
    Stm32PollingService& pollingSvc_;
    ports::IDiagnosticsRepository& diagnosticsRepo_;
    AdminDiagnosticsConfig config_;
};

} // namespace smartcart::application::services
