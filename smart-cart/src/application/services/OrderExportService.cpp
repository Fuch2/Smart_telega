#include "application/services/OrderExportService.hpp"

#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace smartcart::application::services {

namespace domain = smartcart::domain;
using json = nlohmann::json;
namespace fs = std::filesystem;

namespace {

std::string itemStatusToString(domain::OrderItemStatus s) {
    return std::string(domain::toString(s));
}

} // namespace

OrderExportService::OrderExportService(ports::IOrderRepository& orderRepo,
                                       ports::IEventLogger& eventLogger,
                                       std::string completedOrdersDir)
    : orderRepo_(orderRepo)
    , eventLogger_(eventLogger)
    , completedOrdersDir_(std::move(completedOrdersDir))
{}

void OrderExportService::exportOrder(int orderId) {
    if (completedOrdersDir_.empty()) {
        return; // экспорт отключён
    }

    try {
        const auto orderOpt = orderRepo_.getOrderById(orderId);
        if (!orderOpt.has_value()) {
            throw std::runtime_error("OrderExportService: order not found id=" +
                                     std::to_string(orderId));
        }
        const auto& order = *orderOpt;
        const auto items = orderRepo_.getItems(orderId);

        // --- Собираем JSON ---
        json j;
        j["order_id"]        = order.externalOrderId;
        j["title"]           = order.title;
        j["priority"]        = order.priority;
        j["duration_minutes"]= order.durationMinutes;
        j["deadline"]        = order.deadline;
        j["route"]           = order.route;
        j["status"]          = std::string(domain::toString(order.status));
        j["created_at"]      = order.createdAt;
        j["completed_at"]    = order.updatedAt;

        json itemsArr = json::array();
        int issuedCount = 0;
        int returnedCount = 0;
        int missingCount = 0;

        for (const auto& item : items) {
            json ji;
            ji["part_number"]      = item.partNumber;
            ji["barcode"]          = item.barcode;
            ji["material_type"]    = item.materialType;
            ji["required_quantity"]= item.requiredQuantity;
            ji["target_slot"]      = item.targetSlot;
            ji["status"]           = itemStatusToString(item.status);

            if (item.currentSlot.has_value()) {
                ji["current_slot"] = *item.currentSlot;
            } else {
                ji["current_slot"] = nullptr;
            }

            switch (item.status) {
                case domain::OrderItemStatus::Issued:   ++issuedCount;   break;
                case domain::OrderItemStatus::Returned: ++returnedCount; break;
                case domain::OrderItemStatus::Missing:  ++missingCount;  break;
                default: break;
            }

            itemsArr.push_back(std::move(ji));
        }
        j["items"] = std::move(itemsArr);

        // Итоговая сводка
        json summary;
        summary["total"]    = static_cast<int>(items.size());
        summary["issued"]   = issuedCount;
        summary["returned"] = returnedCount;
        summary["missing"]  = missingCount;
        j["completion_result"] = summary;

        // --- Пишем файл ---
        fs::create_directories(fs::path(completedOrdersDir_));

        const std::string filename =
            order.externalOrderId + "_result.json";
        // Заменяем символы, недопустимые в именах файлов
        std::string safeName;
        safeName.reserve(filename.size());
        for (char c : filename) {
            safeName += (c == '/' || c == '\\' || c == ':' || c == '*' ||
                         c == '?' || c == '"' || c == '<' || c == '>' ||
                         c == '|') ? '_' : c;
        }

        const fs::path outPath = fs::path(completedOrdersDir_) / safeName;
        std::ofstream out(outPath);
        if (!out.is_open()) {
            throw std::runtime_error(
                "OrderExportService: cannot write " + outPath.string());
        }
        out << j.dump(2) << '\n';

        std::ostringstream msg;
        msg << "order_id=" << order.externalOrderId
            << " file=" << outPath.string()
            << " issued=" << issuedCount
            << " returned=" << returnedCount
            << " missing=" << missingCount;
        try { eventLogger_.log("INFO", "OrderExported", msg.str()); } catch (...) {}

    } catch (const std::exception& ex) {
        try { eventLogger_.log("ERROR", "OrderExportFailed", ex.what()); } catch (...) {}
    } catch (...) {
        try { eventLogger_.log("ERROR", "OrderExportFailed", "unknown error"); } catch (...) {}
    }
}

} // namespace smartcart::application::services
