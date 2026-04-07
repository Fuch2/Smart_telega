// ===== src/infrastructure/db/repositories/OperationRepositorySqlite.cpp =====
// Исправлено: updateStatus без лишнего параметра
#include "OperationRepositorySqlite.hpp"
#include <sqlite3.h>
#include <stdexcept>

namespace smartcart::infrastructure::db {

using namespace smartcart::domain;

static OperationType opTypeFromString(std::string_view s) {
    if (s == "ADD_REEL")     return OperationType::AddReel;
    if (s == "REMOVE_REEL")  return OperationType::RemoveReel;
    if (s == "REPLACE_REEL") return OperationType::ReplaceReel;
    return OperationType::AddReel;
}

static OperationStatus opStatusFromString(std::string_view s) {
    if (s == "IN_PROGRESS") return OperationStatus::InProgress;
    if (s == "COMPLETED")   return OperationStatus::Completed;
    if (s == "CANCELLED")   return OperationStatus::Cancelled;
    if (s == "FAILED")      return OperationStatus::Failed;
    return OperationStatus::Failed;
}

Operation OperationRepositorySqlite::rowToOp(sqlite3_stmt* stmt) {
    Operation op;
    op.id        = sqlite3_column_int(stmt, 0);
    op.type      = opTypeFromString(
        reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1)));
    op.status    = opStatusFromString(
        reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2)));
    op.moduleId  = sqlite3_column_int(stmt, 3);
    op.slotIndex = sqlite3_column_int(stmt, 4);
    op.barcode   = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 5));
    op.startedAt = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 6));
    if (sqlite3_column_type(stmt, 7) != SQLITE_NULL)
        op.finishedAt =
            reinterpret_cast<const char*>(sqlite3_column_text(stmt, 7));
    return op;
}

OperationRepositorySqlite::OperationRepositorySqlite(SqliteConnection& conn)
    : conn_(conn)
{
    ensureSchema();
}

void OperationRepositorySqlite::ensureSchema() {
    conn_.execute(
        "CREATE TABLE IF NOT EXISTS operations ("
        "  id          INTEGER  PRIMARY KEY AUTOINCREMENT,"
        "  type        TEXT     NOT NULL,"
        "  status      TEXT     NOT NULL DEFAULT 'IN_PROGRESS',"
        "  module_id   INTEGER  NOT NULL,"
        "  slot_index  INTEGER  NOT NULL,"
        "  barcode     TEXT     NOT NULL DEFAULT '',"
        "  started_at  DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,"
        "  finished_at DATETIME"
        ");"
    );
}

int OperationRepositorySqlite::add(const Operation& op) {
    const char* sql =
        "INSERT INTO operations(type, status, module_id, slot_index, barcode) "
        "VALUES(?, ?, ?, ?, ?);";

    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr);

    const std::string typeStr{toString(op.type)};
    const std::string statusStr{toString(op.status)};
    sqlite3_bind_text(stmt, 1, typeStr.c_str(),    -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, statusStr.c_str(),  -1, SQLITE_TRANSIENT);
    sqlite3_bind_int (stmt, 3, op.moduleId);
    sqlite3_bind_int (stmt, 4, op.slotIndex);
    sqlite3_bind_text(stmt, 5, op.barcode.c_str(), -1, SQLITE_TRANSIENT);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    if (rc != SQLITE_DONE)
        throw std::runtime_error("OperationRepositorySqlite::add failed");
    return static_cast<int>(sqlite3_last_insert_rowid(conn_.handle()));
}

bool OperationRepositorySqlite::updateStatus(int id, OperationStatus status) {
    const char* sql =
        "UPDATE operations "
        "SET status = ?,"
        "    finished_at = CASE"
        "        WHEN ? IN ('COMPLETED','CANCELLED','FAILED')"
        "        THEN datetime('now') ELSE finished_at END "
        "WHERE id = ?;";

    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr);

    const std::string statusStr{toString(status)};
    sqlite3_bind_text(stmt, 1, statusStr.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, statusStr.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int (stmt, 3, id);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return rc == SQLITE_DONE && sqlite3_changes(conn_.handle()) > 0;
}

bool OperationRepositorySqlite::updateSlot(int id, int moduleId, int slotIndex) {
    const char* sql =
        "UPDATE operations "
        "SET module_id = ?, slot_index = ? "
        "WHERE id = ?;";

    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr);
    sqlite3_bind_int(stmt, 1, moduleId);
    sqlite3_bind_int(stmt, 2, slotIndex);
    sqlite3_bind_int(stmt, 3, id);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return rc == SQLITE_DONE && sqlite3_changes(conn_.handle()) > 0;
}

std::vector<Operation> OperationRepositorySqlite::getUnfinished() {
    const char* sql =
        "SELECT id, type, status, module_id, slot_index, barcode,"
        "       started_at, finished_at "
        "FROM operations "
        "WHERE status = 'IN_PROGRESS' "
        "ORDER BY id ASC;";

    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr);

    std::vector<Operation> out;
    while (sqlite3_step(stmt) == SQLITE_ROW)
        out.push_back(rowToOp(stmt));
    sqlite3_finalize(stmt);
    return out;
}

std::optional<Operation> OperationRepositorySqlite::getById(int id) {
    const char* sql =
        "SELECT id, type, status, module_id, slot_index, barcode,"
        "       started_at, finished_at "
        "FROM operations WHERE id = ?;";

    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr);
    sqlite3_bind_int(stmt, 1, id);

    std::optional<Operation> out;
    if (sqlite3_step(stmt) == SQLITE_ROW)
        out = rowToOp(stmt);
    sqlite3_finalize(stmt);
    return out;
}

} // namespace smartcart::infrastructure::db
