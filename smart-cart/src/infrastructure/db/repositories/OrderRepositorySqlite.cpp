#include "infrastructure/db/repositories/OrderRepositorySqlite.hpp"

#include <sqlite3.h>

#include <stdexcept>
#include <string>

namespace smartcart::infrastructure::db {

namespace domain = smartcart::domain;

namespace {

const char* kOrderColumns =
    "SELECT id, module_id, external_order_id, title, priority, duration_minutes,"
    "       status, created_at, updated_at "
    "FROM orders";

const char* kItemColumns =
    "SELECT id, order_id, barcode, part_number, material_type,"
    "       required_quantity, usage_priority, target_slot,"
    "       current_slot, status, updated_at "
    "FROM order_items";

void throwOnPrepareError(sqlite3* db, int rc, const char* where) {
    if (rc != SQLITE_OK) {
        throw std::runtime_error(std::string(where) + ": " + sqlite3_errmsg(db));
    }
}

bool columnExists(sqlite3* db,
                  const std::string& tableName,
                  const std::string& columnName) {
    const std::string sql = "PRAGMA table_info(" + tableName + ");";
    sqlite3_stmt* stmt = nullptr;
    throwOnPrepareError(db,
                        sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr),
                        "OrderRepositorySqlite::columnExists prepare");

    bool found = false;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        const unsigned char* text = sqlite3_column_text(stmt, 1);
        if (text != nullptr &&
            columnName == reinterpret_cast<const char*>(text))
        {
            found = true;
            break;
        }
    }

    sqlite3_finalize(stmt);
    return found;
}

void addColumnIfMissing(SqliteConnection& conn,
                        const std::string& tableName,
                        const std::string& columnName,
                        const std::string& definition) {
    if (columnExists(conn.handle(), tableName, columnName)) {
        return;
    }

    conn.execute(
        "ALTER TABLE " + tableName +
        " ADD COLUMN " + columnName + " " + definition + ";"
    );
}

} // namespace

OrderRepositorySqlite::OrderRepositorySqlite(SqliteConnection& conn,
                                             int moduleId)
    : conn_(conn)
    , moduleId_(moduleId)
{
    ensureSchema();
}

void OrderRepositorySqlite::ensureSchema() {
    conn_.execute(
        "CREATE TABLE IF NOT EXISTS orders ("
        "  id                INTEGER PRIMARY KEY AUTOINCREMENT,"
        "  module_id         INTEGER NOT NULL DEFAULT 1,"
        "  external_order_id TEXT    NOT NULL,"
        "  title             TEXT    NOT NULL DEFAULT '',"
        "  priority          TEXT    NOT NULL DEFAULT '',"
        "  duration_minutes  INTEGER NOT NULL DEFAULT 0,"
        "  status            TEXT    NOT NULL DEFAULT 'LOADED',"
        "  created_at        DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,"
        "  updated_at        DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,"
        "  UNIQUE(module_id, external_order_id)"
        ");"
    );

    conn_.execute(
        "CREATE TABLE IF NOT EXISTS order_items ("
        "  id            INTEGER PRIMARY KEY AUTOINCREMENT,"
        "  order_id      INTEGER NOT NULL,"
        "  barcode       TEXT    NOT NULL,"
        "  part_number   TEXT    NOT NULL DEFAULT '',"
        "  material_type TEXT    NOT NULL DEFAULT 'reel',"
        "  required_quantity INTEGER NOT NULL DEFAULT 1,"
        "  usage_priority INTEGER NOT NULL DEFAULT 3,"
        "  target_slot   INTEGER NOT NULL,"
        "  current_slot  INTEGER,"
        "  status        TEXT    NOT NULL DEFAULT 'PENDING',"
        "  updated_at    DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,"
        "  UNIQUE(order_id, barcode),"
        "  FOREIGN KEY(order_id) REFERENCES orders(id)"
        "    ON DELETE CASCADE ON UPDATE CASCADE"
        ");"
    );

    addColumnIfMissing(conn_, "order_items", "part_number",
                       "TEXT NOT NULL DEFAULT ''");
    addColumnIfMissing(conn_, "order_items", "required_quantity",
                       "INTEGER NOT NULL DEFAULT 1");
    addColumnIfMissing(conn_, "order_items", "usage_priority",
                       "INTEGER NOT NULL DEFAULT 3");

    conn_.execute(
        "CREATE INDEX IF NOT EXISTS idx_orders_status "
        "ON orders(module_id, status, updated_at);"
    );
    conn_.execute(
        "CREATE INDEX IF NOT EXISTS idx_order_items_part_number "
        "ON order_items(order_id, part_number);"
    );
}

domain::OrderInfo OrderRepositorySqlite::rowToOrder(sqlite3_stmt* stmt) {
    domain::OrderInfo order;
    order.id = sqlite3_column_int(stmt, 0);
    order.moduleId = sqlite3_column_int(stmt, 1);
    order.externalOrderId =
        reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2));
    order.title =
        reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3));
    order.priority =
        reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4));
    order.durationMinutes = sqlite3_column_int(stmt, 5);
    order.status = domain::orderStatusFromString(
        reinterpret_cast<const char*>(sqlite3_column_text(stmt, 6)));
    order.createdAt =
        reinterpret_cast<const char*>(sqlite3_column_text(stmt, 7));
    order.updatedAt =
        reinterpret_cast<const char*>(sqlite3_column_text(stmt, 8));
    return order;
}

domain::OrderItem OrderRepositorySqlite::rowToItem(sqlite3_stmt* stmt) {
    domain::OrderItem item;
    item.id = sqlite3_column_int(stmt, 0);
    item.orderId = sqlite3_column_int(stmt, 1);
    item.barcode =
        reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2));
    item.partNumber =
        reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3));
    item.materialType =
        reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4));
    item.requiredQuantity = sqlite3_column_int(stmt, 5);
    item.usagePriority = sqlite3_column_int(stmt, 6);
    item.targetSlot = sqlite3_column_int(stmt, 7);
    if (sqlite3_column_type(stmt, 8) != SQLITE_NULL) {
        item.currentSlot = sqlite3_column_int(stmt, 8);
    }
    item.status = domain::orderItemStatusFromString(
        reinterpret_cast<const char*>(sqlite3_column_text(stmt, 9)));
    item.updatedAt =
        reinterpret_cast<const char*>(sqlite3_column_text(stmt, 10));
    return item;
}

int OrderRepositorySqlite::addOrder(const domain::OrderInfo& order) {
    const char* sql =
        "INSERT INTO orders(module_id, external_order_id, title, priority,"
        "                   duration_minutes, status) "
        "VALUES(?, ?, ?, ?, ?, ?);";

    sqlite3_stmt* stmt = nullptr;
    throwOnPrepareError(conn_.handle(),
                        sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr),
                        "OrderRepositorySqlite::addOrder prepare");

    const std::string status{domain::toString(order.status)};
    sqlite3_bind_int(stmt, 1, moduleId_);
    sqlite3_bind_text(stmt, 2, order.externalOrderId.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 3, order.title.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 4, order.priority.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 5, order.durationMinutes);
    sqlite3_bind_text(stmt, 6, status.c_str(), -1, SQLITE_TRANSIENT);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    if (rc != SQLITE_DONE) {
        throw std::runtime_error("OrderRepositorySqlite::addOrder failed");
    }
    return static_cast<int>(sqlite3_last_insert_rowid(conn_.handle()));
}

int OrderRepositorySqlite::addItem(const domain::OrderItem& item) {
    const char* sql =
        "INSERT INTO order_items(order_id, barcode, part_number, material_type,"
        "                        required_quantity, usage_priority,"
        "                        target_slot, current_slot, status) "
        "VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?);";

    sqlite3_stmt* stmt = nullptr;
    throwOnPrepareError(conn_.handle(),
                        sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr),
                        "OrderRepositorySqlite::addItem prepare");

    const std::string status{domain::toString(item.status)};
    sqlite3_bind_int(stmt, 1, item.orderId);
    sqlite3_bind_text(stmt, 2, item.barcode.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 3, item.partNumber.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 4, item.materialType.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 5, item.requiredQuantity);
    sqlite3_bind_int(stmt, 6, item.usagePriority);
    sqlite3_bind_int(stmt, 7, item.targetSlot);
    if (item.currentSlot.has_value()) {
        sqlite3_bind_int(stmt, 8, *item.currentSlot);
    } else {
        sqlite3_bind_null(stmt, 8);
    }
    sqlite3_bind_text(stmt, 9, status.c_str(), -1, SQLITE_TRANSIENT);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    if (rc != SQLITE_DONE) {
        throw std::runtime_error("OrderRepositorySqlite::addItem failed");
    }
    return static_cast<int>(sqlite3_last_insert_rowid(conn_.handle()));
}

std::optional<domain::OrderInfo>
OrderRepositorySqlite::getOrderById(int id) {
    const std::string sql =
        std::string(kOrderColumns) + " WHERE id = ? AND module_id = ?;";
    sqlite3_stmt* stmt = nullptr;
    throwOnPrepareError(conn_.handle(),
                        sqlite3_prepare_v2(conn_.handle(), sql.c_str(), -1, &stmt, nullptr),
                        "OrderRepositorySqlite::getOrderById prepare");
    sqlite3_bind_int(stmt, 1, id);
    sqlite3_bind_int(stmt, 2, moduleId_);

    std::optional<domain::OrderInfo> result;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        result = rowToOrder(stmt);
    }
    sqlite3_finalize(stmt);
    return result;
}

std::optional<domain::OrderInfo>
OrderRepositorySqlite::findOrderByExternalId(const std::string& externalOrderId) {
    const std::string sql =
        std::string(kOrderColumns) +
        " WHERE external_order_id = ? AND module_id = ?;";
    sqlite3_stmt* stmt = nullptr;
    throwOnPrepareError(conn_.handle(),
                        sqlite3_prepare_v2(conn_.handle(), sql.c_str(), -1, &stmt, nullptr),
                        "OrderRepositorySqlite::findOrderByExternalId prepare");
    sqlite3_bind_text(stmt, 1, externalOrderId.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 2, moduleId_);

    std::optional<domain::OrderInfo> result;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        result = rowToOrder(stmt);
    }
    sqlite3_finalize(stmt);
    return result;
}

std::optional<domain::OrderInfo> OrderRepositorySqlite::getActiveOrder() {
    const std::string sql =
        std::string(kOrderColumns) +
        " WHERE module_id = ? AND status IN ('LOADED','IN_PROGRESS') "
        " ORDER BY id DESC LIMIT 1;";
    sqlite3_stmt* stmt = nullptr;
    throwOnPrepareError(conn_.handle(),
                        sqlite3_prepare_v2(conn_.handle(), sql.c_str(), -1, &stmt, nullptr),
                        "OrderRepositorySqlite::getActiveOrder prepare");

    sqlite3_bind_int(stmt, 1, moduleId_);

    std::optional<domain::OrderInfo> result;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        result = rowToOrder(stmt);
    }
    sqlite3_finalize(stmt);
    return result;
}

std::vector<domain::OrderItem> OrderRepositorySqlite::getItems(int orderId) {
    const std::string sql =
        std::string(kItemColumns) +
        " WHERE order_id = ? ORDER BY id ASC;";
    sqlite3_stmt* stmt = nullptr;
    throwOnPrepareError(conn_.handle(),
                        sqlite3_prepare_v2(conn_.handle(), sql.c_str(), -1, &stmt, nullptr),
                        "OrderRepositorySqlite::getItems prepare");
    sqlite3_bind_int(stmt, 1, orderId);

    std::vector<domain::OrderItem> result;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        result.push_back(rowToItem(stmt));
    }
    sqlite3_finalize(stmt);
    return result;
}

std::optional<domain::OrderItem>
OrderRepositorySqlite::findItemByBarcode(int orderId,
                                         const std::string& barcode) {
    const std::string sql =
        std::string(kItemColumns) +
        " WHERE order_id = ? AND barcode = ? LIMIT 1;";
    sqlite3_stmt* stmt = nullptr;
    throwOnPrepareError(conn_.handle(),
                        sqlite3_prepare_v2(conn_.handle(), sql.c_str(), -1, &stmt, nullptr),
                        "OrderRepositorySqlite::findItemByBarcode prepare");
    sqlite3_bind_int(stmt, 1, orderId);
    sqlite3_bind_text(stmt, 2, barcode.c_str(), -1, SQLITE_TRANSIENT);

    std::optional<domain::OrderItem> result;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        result = rowToItem(stmt);
    }
    sqlite3_finalize(stmt);
    return result;
}

std::optional<domain::OrderItem>
OrderRepositorySqlite::findItemByScannedBarcode(
    int orderId,
    const std::string& scannedBarcode) {
    const std::string sql =
        std::string(kItemColumns) +
        " WHERE order_id = ? AND ("
        "   barcode = ? OR"
        "   part_number = ? OR"
        "   (part_number <> '' AND instr(?, part_number) > 0)"
        " ) "
        " ORDER BY CASE "
        "   WHEN barcode = ? THEN 0 "
        "   WHEN part_number = ? THEN 1 "
        "   ELSE 2 "
        " END, required_quantity DESC, id ASC "
        " LIMIT 1;";

    sqlite3_stmt* stmt = nullptr;
    throwOnPrepareError(
        conn_.handle(),
        sqlite3_prepare_v2(conn_.handle(), sql.c_str(), -1, &stmt, nullptr),
        "OrderRepositorySqlite::findItemByScannedBarcode prepare");

    sqlite3_bind_int(stmt, 1, orderId);
    sqlite3_bind_text(stmt, 2, scannedBarcode.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 3, scannedBarcode.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 4, scannedBarcode.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 5, scannedBarcode.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 6, scannedBarcode.c_str(), -1, SQLITE_TRANSIENT);

    std::optional<domain::OrderItem> result;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        result = rowToItem(stmt);
    }
    sqlite3_finalize(stmt);
    return result;
}

bool OrderRepositorySqlite::updateOrderStatus(int id, domain::OrderStatus status) {
    const char* sql =
        "UPDATE orders SET status = ?, updated_at = datetime('now') "
        "WHERE id = ? AND module_id = ?;";
    sqlite3_stmt* stmt = nullptr;
    throwOnPrepareError(conn_.handle(),
                        sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr),
                        "OrderRepositorySqlite::updateOrderStatus prepare");

    const std::string statusStr{domain::toString(status)};
    sqlite3_bind_text(stmt, 1, statusStr.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 2, id);
    sqlite3_bind_int(stmt, 3, moduleId_);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return rc == SQLITE_DONE && sqlite3_changes(conn_.handle()) > 0;
}

bool OrderRepositorySqlite::updateItemStatus(int id,
                                             domain::OrderItemStatus status) {
    const char* sql =
        "UPDATE order_items SET status = ?, updated_at = datetime('now') "
        "WHERE id = ?;";
    sqlite3_stmt* stmt = nullptr;
    throwOnPrepareError(conn_.handle(),
                        sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr),
                        "OrderRepositorySqlite::updateItemStatus prepare");

    const std::string statusStr{domain::toString(status)};
    sqlite3_bind_text(stmt, 1, statusStr.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 2, id);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return rc == SQLITE_DONE && sqlite3_changes(conn_.handle()) > 0;
}

bool OrderRepositorySqlite::updateItemPlacement(
    int id,
    int currentSlot,
    domain::OrderItemStatus status)
{
    const char* sql =
        "UPDATE order_items "
        "SET current_slot = ?, status = ?, updated_at = datetime('now') "
        "WHERE id = ?;";
    sqlite3_stmt* stmt = nullptr;
    throwOnPrepareError(conn_.handle(),
                        sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr),
                        "OrderRepositorySqlite::updateItemPlacement prepare");

    const std::string statusStr{domain::toString(status)};
    sqlite3_bind_int(stmt, 1, currentSlot);
    sqlite3_bind_text(stmt, 2, statusStr.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 3, id);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return rc == SQLITE_DONE && sqlite3_changes(conn_.handle()) > 0;
}

bool OrderRepositorySqlite::adoptActiveOrderFrom(int fromModuleId) {
    // Если у текущего модуля уже есть активный заказ — ничего не делаем.
    if (getActiveOrder().has_value()) {
        return true;
    }

    // Переносим активный заказ из fromModuleId в наш moduleId_.
    const char* sql =
        "UPDATE orders SET module_id = ? "
        "WHERE module_id = ? "
        "  AND status IN ('LOADED', 'IN_PROGRESS') "
        "  AND id = (SELECT id FROM orders "
        "            WHERE module_id = ? "
        "              AND status IN ('LOADED', 'IN_PROGRESS') "
        "            ORDER BY updated_at DESC LIMIT 1);";

    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr) != SQLITE_OK) {
        return false;
    }
    sqlite3_bind_int(stmt, 1, moduleId_);
    sqlite3_bind_int(stmt, 2, fromModuleId);
    sqlite3_bind_int(stmt, 3, fromModuleId);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return rc == SQLITE_DONE && sqlite3_changes(conn_.handle()) > 0;
}

} // namespace smartcart::infrastructure::db
