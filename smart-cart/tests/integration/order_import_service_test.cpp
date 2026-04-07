#include "application/services/OrderImportService.hpp"
#include "infrastructure/db/SqliteConnection.hpp"
#include "infrastructure/db/repositories/OrderRepositorySqlite.hpp"
#include "infrastructure/db/repositories/ReelRepositorySqlite.hpp"
#include "infrastructure/db/repositories/WorkflowRepositorySqlite.hpp"
#include "infrastructure/logging/SqliteEventLogger.hpp"
#include "domain/entities/CartWorkflow.hpp"

#include <sqlite3.h>

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

using namespace smartcart;
using namespace smartcart::application::services;

namespace {

std::filesystem::path writeOrderJson(const std::string& name,
                                     const std::string& text) {
    const auto path = std::filesystem::temp_directory_path() / name;
    std::ofstream out(path);
    out << text;
    return path;
}

int countEvents(sqlite3* db, const std::string& code) {
    const char* sql = "SELECT COUNT(1) FROM event_log WHERE code = ?;";
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr);
    sqlite3_bind_text(stmt, 1, code.c_str(), -1, SQLITE_TRANSIENT);

    int count = 0;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        count = sqlite3_column_int(stmt, 0);
    }
    sqlite3_finalize(stmt);
    return count;
}

} // namespace

TEST(OrderImportServiceTest, ImportValidJson_CreatesOrderItemsAndWorkflow) {
    infrastructure::db::SqliteConnection conn(":memory:");
    conn.runMigrations(std::string{MIGRATIONS_DIR});

    infrastructure::db::OrderRepositorySqlite orderRepo(conn);
    infrastructure::db::WorkflowRepositorySqlite workflowRepo(conn);
    infrastructure::db::ReelRepositorySqlite reelRepo(conn);
    infrastructure::logging::SqliteEventLogger logger(conn.handle());

    const auto path = writeOrderJson(
        "smartcart_order_import_valid.json",
        R"({
            "order_id": "ORDER-IMPORT-001",
            "title": "Demo order",
            "priority": "normal",
            "duration_minutes": 120,
            "items": [
                {
                    "barcode": "R-150 10%",
                    "material_type": "reel",
                    "target_slot": 2
                },
                {
                    "barcode": "R-250 20%",
                    "material_type": "reel",
                    "target_slot": 4
                }
            ]
        })"
    );

    OrderImportService svc(orderRepo, workflowRepo, reelRepo, logger, {});
    const auto result = svc.importFromFile(path.string());

    ASSERT_TRUE(result.success) << result.message;
    ASSERT_GT(result.orderId, 0);

    const auto workflow = workflowRepo.get();
    ASSERT_TRUE(workflow.currentOrderId.has_value());
    EXPECT_EQ(*workflow.currentOrderId, result.orderId);
    EXPECT_EQ(workflow.state, domain::CartWorkflowState::OrderLoaded);

    const auto order = orderRepo.getOrderById(result.orderId);
    ASSERT_TRUE(order.has_value());
    EXPECT_EQ(order->externalOrderId, "ORDER-IMPORT-001");
    EXPECT_EQ(order->status, domain::OrderStatus::Loaded);

    const auto items = orderRepo.getItems(result.orderId);
    ASSERT_EQ(items.size(), 2U);
    EXPECT_EQ(items[0].barcode, "R-150 10%");
    EXPECT_EQ(items[0].targetSlot, 2);
    EXPECT_EQ(items[1].barcode, "R-250 20%");
    EXPECT_EQ(items[1].targetSlot, 4);

    EXPECT_EQ(countEvents(conn.handle(), "OrderLoaded"), 1);
}

TEST(OrderImportServiceTest, ImportRejected_WhenCartHasLeftovers) {
    infrastructure::db::SqliteConnection conn(":memory:");
    conn.runMigrations(std::string{MIGRATIONS_DIR});

    infrastructure::db::OrderRepositorySqlite orderRepo(conn);
    infrastructure::db::WorkflowRepositorySqlite workflowRepo(conn);
    infrastructure::db::ReelRepositorySqlite reelRepo(conn);
    infrastructure::logging::SqliteEventLogger logger(conn.handle());

    conn.execute(
        "INSERT INTO modules(id, serial, slot_count, firmware, status) "
        "VALUES(1, 'TEST-MODULE', 24, '', 'ONLINE');"
    );
    reelRepo.addRecord(1, 2, "LEFTOVER-REEL");

    const auto path = writeOrderJson(
        "smartcart_order_import_leftover.json",
        R"({
            "order_id": "ORDER-IMPORT-002",
            "title": "Blocked order",
            "items": [
                {
                    "barcode": "R-150 10%",
                    "material_type": "reel",
                    "target_slot": 2
                }
            ]
        })"
    );

    OrderImportService svc(orderRepo, workflowRepo, reelRepo, logger, {});
    const auto result = svc.importFromFile(path.string());

    EXPECT_FALSE(result.success);
    EXPECT_EQ(orderRepo.getActiveOrder(), std::nullopt);
    EXPECT_EQ(workflowRepo.get().state, domain::CartWorkflowState::Free);
    EXPECT_EQ(countEvents(conn.handle(), "OrderImportRejected"), 1);
}

TEST(OrderImportServiceTest, ImportAllowsLeftover_WhenBarcodeBelongsToOrder) {
    infrastructure::db::SqliteConnection conn(":memory:");
    conn.runMigrations(std::string{MIGRATIONS_DIR});

    infrastructure::db::OrderRepositorySqlite orderRepo(conn);
    infrastructure::db::WorkflowRepositorySqlite workflowRepo(conn);
    infrastructure::db::ReelRepositorySqlite reelRepo(conn);
    infrastructure::logging::SqliteEventLogger logger(conn.handle());

    conn.execute(
        "INSERT INTO modules(id, serial, slot_count, firmware, status) "
        "VALUES(1, 'TEST-MODULE', 24, '', 'ONLINE');"
    );
    reelRepo.addRecord(1, 1, "R-150 10%");

    const auto path = writeOrderJson(
        "smartcart_order_import_reuse_leftover.json",
        R"({
            "order_id": "ORDER-IMPORT-003",
            "title": "Reuse leftover order",
            "items": [
                {
                    "barcode": "R-150 10%",
                    "material_type": "reel",
                    "target_slot": 2
                },
                {
                    "barcode": "R-250 20%",
                    "material_type": "reel",
                    "target_slot": 4
                }
            ]
        })"
    );

    OrderImportService svc(orderRepo, workflowRepo, reelRepo, logger, {});
    const auto result = svc.importFromFile(path.string());

    ASSERT_TRUE(result.success) << result.message;
    ASSERT_GT(result.orderId, 0);

    const auto reused = orderRepo.findItemByBarcode(result.orderId, "R-150 10%");
    ASSERT_TRUE(reused.has_value());
    ASSERT_TRUE(reused->currentSlot.has_value());
    EXPECT_EQ(*reused->currentSlot, 1);
    EXPECT_EQ(reused->status, domain::OrderItemStatus::Placed);

    const auto pending = orderRepo.findItemByBarcode(result.orderId, "R-250 20%");
    ASSERT_TRUE(pending.has_value());
    EXPECT_FALSE(pending->currentSlot.has_value());
    EXPECT_EQ(pending->status, domain::OrderItemStatus::Pending);

    EXPECT_EQ(workflowRepo.get().state, domain::CartWorkflowState::OrderLoaded);
    EXPECT_EQ(countEvents(conn.handle(), "OrderLeftoverReused"), 1);
}
