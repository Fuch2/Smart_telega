#pragma once

#include "application/ports/IEventLogger.hpp"
#include "application/ports/IOrderRepository.hpp"
#include "application/ports/IReelRepository.hpp"
#include "application/ports/IWorkflowRepository.hpp"
#include "domain/errors/ErrorCode.hpp"

#include <string>
#include <string_view>
#include <vector>

namespace smartcart::application::services {

struct OrderImportConfig {
    int moduleId = 1;
};

struct OrderImportResult {
    bool success{false};
    int orderId{0};
    domain::ErrorCode error{domain::ErrorCode::None};
    std::string message;

    explicit operator bool() const noexcept { return success; }
};

class OrderImportService {
public:
    OrderImportService(ports::IOrderRepository& orderRepo,
                       ports::IWorkflowRepository& workflowRepo,
                       ports::IReelRepository& reelRepo,
                       ports::IEventLogger& eventLogger,
                       OrderImportConfig config);

    OrderImportResult importFromFile(const std::string& jsonPath);

private:
    ports::IOrderRepository& orderRepo_;
    ports::IWorkflowRepository& workflowRepo_;
    ports::IReelRepository& reelRepo_;
    ports::IEventLogger& eventLogger_;
    OrderImportConfig config_;

    OrderImportResult reject(domain::ErrorCode code,
                             std::string message,
                             std::string_view logCode) const;
    void logSafe(std::string_view level,
                 std::string_view code,
                 std::string_view message) const;
};

} // namespace smartcart::application::services
