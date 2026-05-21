#pragma once

#include "domain/entities/CartWorkflow.hpp"

#include <optional>
#include <string>
#include <vector>

namespace smartcart::application::ports {

class IOrderRepository {
public:
    virtual ~IOrderRepository() = default;

    virtual int addOrder(const domain::OrderInfo& order) = 0;
    virtual int addItem(const domain::OrderItem& item) = 0;

    virtual std::optional<domain::OrderInfo> getOrderById(int id) = 0;
    virtual std::optional<domain::OrderInfo> findOrderByExternalId(
        const std::string& externalOrderId) = 0;
    virtual std::optional<domain::OrderInfo> getActiveOrder() = 0;

    virtual std::vector<domain::OrderItem> getItems(int orderId) = 0;
    virtual std::optional<domain::OrderItem> findItemByBarcode(
        int orderId,
        const std::string& barcode) = 0;
    virtual std::optional<domain::OrderItem> findItemByScannedBarcode(
        int orderId,
        const std::string& scannedBarcode) = 0;

    virtual bool updateOrderStatus(int id, domain::OrderStatus status) = 0;
    virtual bool updateItemStatus(int id, domain::OrderItemStatus status) = 0;
    virtual bool updateItemPlacement(int id,
                                     int currentSlot,
                                     domain::OrderItemStatus status) = 0;

    /// Переносит активный заказ из fromModuleId в текущий модуль.
    /// Старые активные заказы текущего модуля переводятся в CANCELLED, чтобы
    /// экран после смены RFID-модуля всегда показывал переносимый заказ.
    virtual bool adoptActiveOrderFrom(int fromModuleId) = 0;
};

} // namespace smartcart::application::ports
