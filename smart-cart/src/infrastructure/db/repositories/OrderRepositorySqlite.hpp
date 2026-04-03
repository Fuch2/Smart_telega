#pragma once

#include "application/ports/IOrderRepository.hpp"
#include "infrastructure/db/SqliteConnection.hpp"

struct sqlite3_stmt;

namespace smartcart::infrastructure::db {

class OrderRepositorySqlite final
    : public application::ports::IOrderRepository
{
public:
    explicit OrderRepositorySqlite(SqliteConnection& conn, int moduleId = 1);

    int addOrder(const domain::OrderInfo& order) override;
    int addItem(const domain::OrderItem& item) override;

    std::optional<domain::OrderInfo> getOrderById(int id) override;
    std::optional<domain::OrderInfo> findOrderByExternalId(
        const std::string& externalOrderId) override;
    std::optional<domain::OrderInfo> getActiveOrder() override;

    std::vector<domain::OrderItem> getItems(int orderId) override;
    std::optional<domain::OrderItem> findItemByBarcode(
        int orderId,
        const std::string& barcode) override;

    bool updateOrderStatus(int id, domain::OrderStatus status) override;
    bool updateItemStatus(int id, domain::OrderItemStatus status) override;
    bool updateItemPlacement(int id,
                             int currentSlot,
                             domain::OrderItemStatus status) override;

private:
    SqliteConnection& conn_;
    int moduleId_{1};

    static domain::OrderInfo rowToOrder(sqlite3_stmt* stmt);
    static domain::OrderItem rowToItem(sqlite3_stmt* stmt);
    void ensureSchema();
};

} // namespace smartcart::infrastructure::db
