#include "application/services/WorkflowService.hpp"

#include <algorithm>
#include <optional>
#include <sstream>
#include <utility>
#include <vector>

namespace smartcart::application::services {

namespace domain = smartcart::domain;

namespace {

struct ItemStats {
    int total{0};
    int pending{0};
    int placed{0};
    int issued{0};
    int returned{0};
    int missing{0};
    int wrongSlot{0};
};

ItemStats countItems(const std::vector<domain::OrderItem>& items) {
    ItemStats stats;
    stats.total = static_cast<int>(items.size());

    for (const auto& item : items) {
        switch (item.status) {
            case domain::OrderItemStatus::Pending:   ++stats.pending; break;
            case domain::OrderItemStatus::Placed:    ++stats.placed; break;
            case domain::OrderItemStatus::Issued:    ++stats.issued; break;
            case domain::OrderItemStatus::Returned:  ++stats.returned; break;
            case domain::OrderItemStatus::Missing:   ++stats.missing; break;
            case domain::OrderItemStatus::WrongSlot: ++stats.wrongSlot; break;
        }
    }

    return stats;
}

std::string describeLeftovers(const std::vector<domain::ReelRecord>& leftovers) {
    if (leftovers.empty()) {
        return "leftovers=[]";
    }

    std::ostringstream msg;
    msg << "leftovers=[";
    for (size_t i = 0; i < leftovers.size(); ++i) {
        if (i != 0) {
            msg << "; ";
        }
        msg << leftovers[i].barcode << " slot=" << leftovers[i].slotIndex;
    }
    msg << "]";
    return msg.str();
}

std::string buildOrderReport(int orderId,
                             const std::vector<domain::OrderItem>& items,
                             const std::vector<domain::ReelRecord>& leftovers) {
    const auto stats = countItems(items);

    std::ostringstream msg;
    msg << "order_id=" << orderId
        << " total=" << stats.total
        << " issued=" << stats.issued
        << " returned=" << stats.returned
        << " placed_not_issued=" << stats.placed
        << " pending=" << stats.pending
        << " missing=" << stats.missing
        << " wrong_slot=" << stats.wrongSlot
        << " leftovers_count=" << leftovers.size()
        << " " << describeLeftovers(leftovers);
    return msg.str();
}

} // namespace

WorkflowService::WorkflowService(ports::IOrderRepository& orderRepo,
                                 ports::IWorkflowRepository& workflowRepo,
                                 ports::IReelRepository& reelRepo,
                                 ports::IEventLogger& eventLogger,
                                 int moduleId)
    : orderRepo_(orderRepo)
    , workflowRepo_(workflowRepo)
    , reelRepo_(reelRepo)
    , eventLogger_(eventLogger)
    , moduleId_(moduleId)
{}

WorkflowActionResult WorkflowService::notifyMaterialPlaced() {
    const auto orderId = currentOrderId();
    if (!orderId.has_value()) {
        return reject(domain::ErrorCode::Unknown,
                      "Нет активного заказа",
                      "InvalidWorkflowTransition");
    }

    const auto items = orderRepo_.getItems(*orderId);
    if (items.empty()) {
        return reject(domain::ErrorCode::Unknown,
                      "В заказе нет позиций",
                      "InvalidWorkflowTransition");
    }

    const bool allPlaced = std::all_of(items.begin(), items.end(), [](const auto& item) {
        return item.status == domain::OrderItemStatus::Placed ||
               item.status == domain::OrderItemStatus::Issued;
    });

    if (!allPlaced) {
        workflowRepo_.setState(domain::CartWorkflowState::PickingMaterials);
        return ok("Подбор материалов продолжается");
    }

    orderRepo_.updateOrderStatus(*orderId, domain::OrderStatus::InProgress);
    workflowRepo_.setState(domain::CartWorkflowState::ReadyForFeederPrep);
    logSafe("INFO", "PickingCompleted", "Подбор материалов завершён");
    logSafe("INFO", "NextStep", "Перевезите тележку в зону подготовки питателей");
    return ok("Подбор завершён. Перевезите тележку в зону подготовки питателей");
}

WorkflowActionResult WorkflowService::startFeederLoading() {
    const auto workflow = workflowRepo_.get();
    // Allow also FREE+order (order adopted from another module, no workflow started yet).
    if (workflow.state == domain::CartWorkflowState::Free &&
        workflow.currentOrderId.has_value())
    {
        workflowRepo_.setState(domain::CartWorkflowState::OrderLoaded);
    }

    if (auto guard = requireState(domain::CartWorkflowState::OrderLoaded,
                                  "startFeederLoading");
        !guard)
    {
        return guard;
    }

    workflowRepo_.setState(domain::CartWorkflowState::LoadingFeeders);
    logSafe("INFO", "FeederLoadingStarted", "Загрузка питателей в тележку начата");
    return ok("Загрузите питатели в тележку");
}

WorkflowActionResult WorkflowService::completeFeederLoading() {
    if (auto guard = requireState(domain::CartWorkflowState::LoadingFeeders,
                                  "completeFeederLoading");
        !guard)
    {
        return guard;
    }

    workflowRepo_.setState(domain::CartWorkflowState::PickingMaterials);
    logSafe("INFO", "FeederLoadingCompleted", "Питатели загружены в тележку");
    logSafe("INFO", "NextStep", "Начните подбор материалов по заказу");
    return ok("Питатели загружены. Начните подбор материалов");
}

WorkflowActionResult WorkflowService::skipFeederLoading() {
    if (auto guard = requireState(domain::CartWorkflowState::OrderLoaded,
                                  "skipFeederLoading");
        !guard)
    {
        return guard;
    }

    workflowRepo_.setState(domain::CartWorkflowState::PickingMaterials);
    logSafe("INFO", "FeederLoadingSkipped",
            "Тележка была в работе, загрузка питателей пропущена");
    logSafe("INFO", "NextStep", "Начните подбор материалов по заказу");
    return ok("Тележка была в работе. Начните подбор материалов");
}

WorkflowActionResult WorkflowService::markCartArrivedToFeederPrep() {
    if (auto guard = requireState(domain::CartWorkflowState::ReadyForFeederPrep,
                                  "markCartArrivedToFeederPrep");
        !guard)
    {
        return guard;
    }

    logSafe("INFO", "CartArrivedToFeederPrep", "Тележка прибыла в зону подготовки питателей");
    return ok("Тележка прибыла в зону подготовки питателей");
}

WorkflowActionResult WorkflowService::startFeederPrep() {
    if (auto guard = requireState(domain::CartWorkflowState::ReadyForFeederPrep,
                                  "startFeederPrep");
        !guard)
    {
        return guard;
    }

    workflowRepo_.setState(domain::CartWorkflowState::FeederPrep);
    logSafe("INFO", "FeederPrepStarted", "Подготовка питателей начата");
    return ok("Подготовка питателей начата");
}

WorkflowActionResult WorkflowService::markFeederPrepCompleted() {
    if (auto guard = requireState(domain::CartWorkflowState::FeederPrep,
                                  "markFeederPrepCompleted");
        !guard)
    {
        return guard;
    }

    workflowRepo_.setState(domain::CartWorkflowState::ReadyForLine);
    logSafe("INFO", "FeederPrepCompleted", "Подготовка питателей завершена");
    logSafe("INFO", "NextStep", "Перевезите тележку к линии SMT");
    return ok("Подготовка питателей завершена. Перевезите тележку к линии SMT");
}

WorkflowActionResult WorkflowService::markCartArrivedToLine() {
    if (auto guard = requireState(domain::CartWorkflowState::ReadyForLine,
                                  "markCartArrivedToLine");
        !guard)
    {
        return guard;
    }

    logSafe("INFO", "CartArrivedToLine", "Тележка прибыла на линию SMT");
    return ok("Тележка прибыла на линию SMT");
}

WorkflowActionResult WorkflowService::startIssuingToLine() {
    const auto workflow = workflowRepo_.get();
    // Allow also FREE+order (REEL module with adopted order, skipping feeder stages).
    if (workflow.state == domain::CartWorkflowState::Free &&
        workflow.currentOrderId.has_value())
    {
        workflowRepo_.setState(domain::CartWorkflowState::ReadyForLine);
    }

    if (auto guard = requireState(domain::CartWorkflowState::ReadyForLine,
                                  "startIssuingToLine");
        !guard)
    {
        return guard;
    }

    workflowRepo_.setState(domain::CartWorkflowState::IssuingToLine);
    logSafe("INFO", "IssuingToLineStarted", "Выдача материалов на линию начата");
    return ok("Выдача материалов на линию начата");
}

WorkflowActionResult WorkflowService::markItemIssued(const std::string& barcode) {
    if (auto guard = requireState(domain::CartWorkflowState::IssuingToLine,
                                  "markItemIssued");
        !guard)
    {
        return guard;
    }

    const auto orderId = currentOrderId();
    if (!orderId.has_value()) {
        return reject(domain::ErrorCode::Unknown,
                      "Нет активного заказа",
                      "MaterialIssueFailed");
    }

    const auto item = orderRepo_.findItemByScannedBarcode(*orderId, barcode);
    if (!item.has_value()) {
        return reject(domain::ErrorCode::ReelNotFound,
                      "Материал не найден в текущем заказе: " + barcode,
                      "MaterialIssueFailed");
    }

    if (item->currentSlot.has_value()) {
        reelRepo_.markRemovedBySlot(moduleId_, *item->currentSlot);
        reelRepo_.setSlotState(moduleId_, *item->currentSlot, domain::SlotState::Free);
    } else if (auto active = reelRepo_.findActiveByBarcode(barcode)) {
        reelRepo_.markRemovedBySlot(active->moduleId, active->slotIndex);
        reelRepo_.setSlotState(active->moduleId, active->slotIndex, domain::SlotState::Free);
    }

    orderRepo_.updateItemStatus(item->id, domain::OrderItemStatus::Issued);

    std::ostringstream msg;
    msg << "barcode=" << barcode;
    if (item->currentSlot.has_value()) {
        msg << " slot=" << *item->currentSlot;
    }
    logSafe("INFO", "MaterialIssued", msg.str());
    return ok("Материал выдан на линию: " + barcode);
}

WorkflowActionResult WorkflowService::completeIssuing() {
    if (auto guard = requireState(domain::CartWorkflowState::IssuingToLine,
                                  "completeIssuing");
        !guard)
    {
        return guard;
    }

    const auto orderId = currentOrderId();
    if (!orderId.has_value()) {
        return reject(domain::ErrorCode::Unknown,
                      "Нет активного заказа",
                      "IssuingCompleteFailed");
    }

    const auto items = orderRepo_.getItems(*orderId);
    const auto stats = countItems(items);
    const bool allIssued = !items.empty() &&
        std::all_of(items.begin(), items.end(), [](const auto& item) {
            return item.status == domain::OrderItemStatus::Issued ||
                   item.status == domain::OrderItemStatus::Returned;
        });

    if (!allIssued) {
        std::ostringstream msg;
        msg << "Не все материалы выданы на линию: выдано "
            << stats.issued << "/" << stats.total
            << ", ожидает=" << stats.pending
            << ", размещено не выдано=" << stats.placed
            << ", ошибок=" << stats.wrongSlot;
        return reject(domain::ErrorCode::Unknown,
                      msg.str(),
                      "IssuingCompleteFailed");
    }

    const auto active = reelRepo_.getActiveByModule(moduleId_);
    logSafe("INFO", "OrderReport", buildOrderReport(*orderId, items, active));

    orderRepo_.updateOrderStatus(*orderId, domain::OrderStatus::Completed);
    logSafe("INFO", "OrderCompleted", "Заказ завершён");

    if (active.empty()) {
        workflowRepo_.clearCurrentOrder(domain::CartWorkflowState::Free);
        logSafe("INFO", "CartFree", "Остатков нет, тележка свободна");
        return ok("Заказ завершён. Отчёт записан. Остатков нет");
    }

    workflowRepo_.setState(domain::CartWorkflowState::ReturningLeftovers);
    std::ostringstream msg;
    msg << "count=" << active.size()
        << " " << describeLeftovers(active);
    logSafe("WARN", "LeftoversDetected", msg.str());
    logSafe("INFO", "ReturningLeftoversStarted", "Возврат остатков начат автоматически");
    return ok("Заказ завершён. Отчёт записан. Верните остатки на склад");
}

WorkflowActionResult WorkflowService::inspectLeftoversAfterOrderCompleted() {
    const auto workflow = workflowRepo_.get();
    if (workflow.state != domain::CartWorkflowState::OrderCompleted &&
        workflow.state != domain::CartWorkflowState::LeftoversDetected &&
        workflow.state != domain::CartWorkflowState::ReturningLeftovers)
    {
        return reject(domain::ErrorCode::Unknown,
                      "Проверка остатков доступна только после завершения заказа",
                      "InvalidWorkflowTransition");
    }

    const auto active = reelRepo_.getActiveByModule(moduleId_);
    if (active.empty()) {
        workflowRepo_.clearCurrentOrder(domain::CartWorkflowState::Free);
        logSafe("INFO", "CartFree", "Остатков нет, тележка свободна");
        return ok("Остатков нет. Тележка свободна");
    }

    workflowRepo_.setState(domain::CartWorkflowState::LeftoversDetected);
    std::ostringstream msg;
    msg << "count=" << active.size()
        << " " << describeLeftovers(active);
    logSafe("WARN", "LeftoversDetected", msg.str());
    return ok("Обнаружены остатки. Верните их на склад");
}

WorkflowActionResult WorkflowService::startReturningLeftovers() {
    if (auto guard = requireState(domain::CartWorkflowState::LeftoversDetected,
                                  "startReturningLeftovers");
        !guard)
    {
        return guard;
    }

    workflowRepo_.setState(domain::CartWorkflowState::ReturningLeftovers);
    logSafe("INFO", "ReturningLeftoversStarted", "Возврат остатков начат");
    return ok("Возврат остатков начат");
}

WorkflowActionResult WorkflowService::markLeftoverReturnedByBarcode(
    const std::string& barcode)
{
    if (auto active = reelRepo_.findActiveByBarcode(barcode)) {
        return markLeftoverReturnedBySlot(active->slotIndex);
    }
    return reject(domain::ErrorCode::ReelNotFound,
                  "Остаток не найден: " + barcode,
                  "LeftoverReturnFailed");
}

WorkflowActionResult WorkflowService::markLeftoverReturnedBySlot(int slotIndex) {
    const auto workflow = workflowRepo_.get();
    if (workflow.state != domain::CartWorkflowState::ReturningLeftovers &&
        workflow.state != domain::CartWorkflowState::LeftoversDetected)
    {
        return reject(domain::ErrorCode::Unknown,
                      "Возврат остатка сейчас недоступен",
                      "InvalidWorkflowTransition");
    }

    const auto active = reelRepo_.getBySlot(moduleId_, slotIndex);
    if (!active.has_value()) {
        return reject(domain::ErrorCode::ReelNotFound,
                      "В слоте нет активного остатка: " + std::to_string(slotIndex),
                      "LeftoverReturnFailed");
    }

    reelRepo_.markRemovedBySlot(moduleId_, slotIndex);
    reelRepo_.setSlotState(moduleId_, slotIndex, domain::SlotState::Free);

    std::ostringstream msg;
    msg << "barcode=" << active->barcode
        << " slot=" << slotIndex
        << " destination=stock";
    logSafe("INFO", "LeftoverReturned", msg.str());

    if (reelRepo_.getActiveByModule(moduleId_).empty()) {
        workflowRepo_.clearCurrentOrder(domain::CartWorkflowState::Free);
        logSafe("INFO", "CartFree", "Все остатки возвращены, тележка свободна");
        return ok("Все остатки возвращены. Тележка свободна");
    }

    workflowRepo_.setState(domain::CartWorkflowState::ReturningLeftovers);
    return ok("Остаток возвращён");
}

std::optional<int> WorkflowService::currentOrderId() const {
    return workflowRepo_.get().currentOrderId;
}

WorkflowActionResult WorkflowService::ok(std::string message) const {
    return WorkflowActionResult{true, domain::ErrorCode::None, std::move(message)};
}

WorkflowActionResult WorkflowService::reject(domain::ErrorCode code,
                                             std::string message,
                                             std::string_view logCode) const {
    logSafe("ERROR", logCode, message);
    return WorkflowActionResult{false, code, std::move(message)};
}

WorkflowActionResult WorkflowService::requireState(
    domain::CartWorkflowState expected,
    std::string_view action) const
{
    const auto workflow = workflowRepo_.get();
    if (workflow.state == expected) {
        return ok("OK");
    }

    std::ostringstream msg;
    msg << "action=" << action
        << " expected=" << domain::toString(expected)
        << " actual=" << domain::toString(workflow.state);
    return reject(domain::ErrorCode::Unknown,
                  msg.str(),
                  "InvalidWorkflowTransition");
}

void WorkflowService::logSafe(std::string_view level,
                              std::string_view code,
                              std::string_view message) const {
    try {
        eventLogger_.log(level, code, message);
    } catch (...) {
        // Ошибка логирования не должна ломать бизнес-переход.
    }
}

} // namespace smartcart::application::services
