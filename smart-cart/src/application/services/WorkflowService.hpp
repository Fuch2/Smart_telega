#pragma once

#include "application/ports/IEventLogger.hpp"
#include "application/ports/IOrderRepository.hpp"
#include "application/ports/IReelRepository.hpp"
#include "application/ports/IWorkflowRepository.hpp"
#include "domain/errors/ErrorCode.hpp"

#include <optional>
#include <string>
#include <string_view>

namespace smartcart::application::services {

struct WorkflowActionResult {
    bool success{false};
    domain::ErrorCode error{domain::ErrorCode::None};
    std::string message;

    explicit operator bool() const noexcept { return success; }
};

class WorkflowService {
public:
    WorkflowService(ports::IOrderRepository& orderRepo,
                    ports::IWorkflowRepository& workflowRepo,
                    ports::IReelRepository& reelRepo,
                    ports::IEventLogger& eventLogger,
                    int moduleId = 1);

    WorkflowActionResult notifyMaterialPlaced();

    WorkflowActionResult startFeederLoading();
    WorkflowActionResult completeFeederLoading();
    WorkflowActionResult skipFeederLoading();

    WorkflowActionResult markCartArrivedToFeederPrep();
    WorkflowActionResult startFeederPrep();
    WorkflowActionResult markFeederPrepCompleted();

    WorkflowActionResult markCartArrivedToLine();
    WorkflowActionResult startIssuingToLine();
    WorkflowActionResult markItemIssued(const std::string& barcode);
    WorkflowActionResult completeIssuing();

    WorkflowActionResult inspectLeftoversAfterOrderCompleted();
    WorkflowActionResult startReturningLeftovers();
    WorkflowActionResult markLeftoverReturnedByBarcode(const std::string& barcode);
    WorkflowActionResult markLeftoverReturnedBySlot(int slotIndex);

private:
    ports::IOrderRepository& orderRepo_;
    ports::IWorkflowRepository& workflowRepo_;
    ports::IReelRepository& reelRepo_;
    ports::IEventLogger& eventLogger_;
    int moduleId_{1};

    std::optional<int> currentOrderId() const;
    WorkflowActionResult ok(std::string message) const;
    WorkflowActionResult reject(domain::ErrorCode code,
                                std::string message,
                                std::string_view logCode) const;
    WorkflowActionResult requireState(domain::CartWorkflowState expected,
                                      std::string_view action) const;
    void logSafe(std::string_view level,
                 std::string_view code,
                 std::string_view message) const;
};

} // namespace smartcart::application::services
