#include "application/services/OrderImportService.hpp"

#include <nlohmann/json.hpp>

#include <exception>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace smartcart::application::services {

namespace domain = smartcart::domain;
using json = nlohmann::json;

namespace {

std::string readString(const json& j,
                       const char* key,
                       const std::string& fallback = {}) {
    if (!j.contains(key) || j.at(key).is_null()) {
        return fallback;
    }
    if (!j.at(key).is_string()) {
        throw std::runtime_error(std::string("Поле должно быть строкой: ") + key);
    }
    return j.at(key).get<std::string>();
}

int readInt(const json& j, const char* key, int fallback = 0) {
    if (!j.contains(key) || j.at(key).is_null()) {
        return fallback;
    }
    if (!j.at(key).is_number_integer()) {
        throw std::runtime_error(std::string("Поле должно быть целым числом: ") + key);
    }
    return j.at(key).get<int>();
}

domain::OrderInfo parseOrder(const json& j) {
    domain::OrderInfo order;
    order.externalOrderId = readString(j, "order_id");
    order.title = readString(j, "title");
    order.priority = readString(j, "priority", "normal");
    order.durationMinutes = readInt(j, "duration_minutes", 0);
    order.status = domain::OrderStatus::Loaded;

    if (order.externalOrderId.empty()) {
        throw std::runtime_error("order_id не должен быть пустым");
    }
    if (order.durationMinutes < 0) {
        throw std::runtime_error("duration_minutes не должен быть отрицательным");
    }

    return order;
}

std::vector<domain::OrderItem> parseItems(const json& j) {
    if (!j.contains("items") || !j.at("items").is_array()) {
        throw std::runtime_error("items должен быть массивом");
    }

    std::vector<domain::OrderItem> items;
    for (const auto& raw : j.at("items")) {
        if (!raw.is_object()) {
            throw std::runtime_error("Элемент items должен быть объектом");
        }

        domain::OrderItem item;
        item.barcode = readString(raw, "barcode");
        item.materialType = readString(raw, "material_type", "reel");
        item.targetSlot = readInt(raw, "target_slot", 0);
        item.status = domain::OrderItemStatus::Pending;

        if (item.barcode.empty()) {
            throw std::runtime_error("barcode не должен быть пустым");
        }
        if (item.materialType.empty()) {
            throw std::runtime_error("material_type не должен быть пустым");
        }
        if (item.targetSlot < 1) {
            throw std::runtime_error("target_slot должен быть >= 1");
        }

        items.push_back(std::move(item));
    }

    if (items.empty()) {
        throw std::runtime_error("items не должен быть пустым");
    }

    return items;
}

json readJsonFile(const std::string& jsonPath) {
    std::ifstream in(jsonPath);
    if (!in.is_open()) {
        throw std::runtime_error("Не удалось открыть JSON заказа: " + jsonPath);
    }

    json j;
    in >> j;
    if (!j.is_object()) {
        throw std::runtime_error("Корень JSON заказа должен быть объектом");
    }
    return j;
}

} // namespace

OrderImportService::OrderImportService(ports::IOrderRepository& orderRepo,
                                       ports::IWorkflowRepository& workflowRepo,
                                       ports::IReelRepository& reelRepo,
                                       ports::IEventLogger& eventLogger,
                                       OrderImportConfig config)
    : orderRepo_(orderRepo)
    , workflowRepo_(workflowRepo)
    , reelRepo_(reelRepo)
    , eventLogger_(eventLogger)
    , config_(config)
{}

OrderImportResult OrderImportService::importFromFile(const std::string& jsonPath) {
    try {
        const auto workflow = workflowRepo_.get();
        if (workflow.state != domain::CartWorkflowState::Free) {
            return reject(domain::ErrorCode::Unknown,
                          "Тележка не свободна для загрузки нового заказа",
                          "OrderImportRejected");
        }

        const json j = readJsonFile(jsonPath);
        domain::OrderInfo order = parseOrder(j);
        std::vector<domain::OrderItem> items = parseItems(j);

        std::unordered_set<std::string> orderBarcodes;
        for (const auto& item : items) {
            orderBarcodes.insert(item.barcode);
        }

        std::unordered_map<std::string, int> activeSlotByBarcode;
        for (const auto& reel : reelRepo_.getActiveByModule(config_.moduleId)) {
            if (orderBarcodes.find(reel.barcode) == orderBarcodes.end()) {
                std::ostringstream msg;
                msg << "В тележке есть остаток не из нового заказа: "
                    << reel.barcode << " slot=" << reel.slotIndex;
                return reject(domain::ErrorCode::Unknown,
                              msg.str(),
                              "OrderImportRejected");
            }
            activeSlotByBarcode.emplace(reel.barcode, reel.slotIndex);
        }

        if (orderRepo_.findOrderByExternalId(order.externalOrderId).has_value()) {
            return reject(domain::ErrorCode::PersistenceError,
                          "Заказ уже был импортирован: " + order.externalOrderId,
                          "OrderImportRejected");
        }

        const int orderId = orderRepo_.addOrder(order);
        for (auto& item : items) {
            item.orderId = orderId;
            if (const auto it = activeSlotByBarcode.find(item.barcode);
                it != activeSlotByBarcode.end())
            {
                item.currentSlot = it->second;
                item.status = domain::OrderItemStatus::Placed;

                std::ostringstream msg;
                msg << "barcode=" << item.barcode
                    << " current_slot=" << it->second
                    << " target_slot=" << item.targetSlot;
                logSafe("INFO", "OrderLeftoverReused", msg.str());
            }
            orderRepo_.addItem(item);
        }

        if (!workflowRepo_.setCurrentOrder(
                orderId,
                domain::CartWorkflowState::OrderLoaded))
        {
            return reject(domain::ErrorCode::PersistenceError,
                          "Не удалось обновить workflow заказа",
                          "OrderImportFailed");
        }

        std::ostringstream msg;
        msg << "order_id=" << order.externalOrderId
            << " db_id=" << orderId
            << " items=" << items.size();
        logSafe("INFO", "OrderLoaded", msg.str());

        return OrderImportResult{
            true,
            orderId,
            domain::ErrorCode::None,
            "Заказ загружен"
        };
    } catch (const std::exception& ex) {
        return reject(domain::ErrorCode::InvalidConfig,
                      ex.what(),
                      "OrderImportFailed");
    } catch (...) {
        return reject(domain::ErrorCode::Unknown,
                      "Неизвестная ошибка импорта заказа",
                      "OrderImportFailed");
    }
}

OrderImportResult OrderImportService::reject(domain::ErrorCode code,
                                             std::string message,
                                             std::string_view logCode) const {
    logSafe("ERROR", logCode, message);
    return OrderImportResult{false, 0, code, std::move(message)};
}

void OrderImportService::logSafe(std::string_view level,
                                 std::string_view code,
                                 std::string_view message) const {
    try {
        eventLogger_.log(level, code, message);
    } catch (...) {
        // Ошибка логирования не должна маскировать результат импорта.
    }
}

} // namespace smartcart::application::services
