#include "infrastructure/db/repositories/DiagnosticsRepositorySqlite.hpp"

#include <sqlite3.h>

#include <algorithm>
#include <stdexcept>
#include <string>

namespace smartcart::infrastructure::db {

namespace ports = smartcart::application::ports;

namespace {

void throwOnPrepareError(sqlite3* db, int rc, const char* where) {
    if (rc != SQLITE_OK) {
        throw std::runtime_error(std::string(where) + ": " + sqlite3_errmsg(db));
    }
}

std::string columnText(sqlite3_stmt* stmt, int column) {
    const auto* text = sqlite3_column_text(stmt, column);
    return text ? reinterpret_cast<const char*>(text) : std::string{};
}

} // namespace

DiagnosticsRepositorySqlite::DiagnosticsRepositorySqlite(SqliteConnection& conn)
    : conn_(conn)
{}

std::vector<ports::EventLogRecord>
DiagnosticsRepositorySqlite::recentEvents(int limit) {
    const int safeLimit = std::clamp(limit, 1, 200);
    const char* sql =
        "SELECT id, ts, level, code, COALESCE(message, '') "
        "FROM event_log "
        "ORDER BY id DESC "
        "LIMIT ?;";

    sqlite3_stmt* stmt = nullptr;
    throwOnPrepareError(conn_.handle(),
                        sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr),
                        "DiagnosticsRepositorySqlite::recentEvents prepare");
    sqlite3_bind_int(stmt, 1, safeLimit);

    std::vector<ports::EventLogRecord> result;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        ports::EventLogRecord record;
        record.id = sqlite3_column_int(stmt, 0);
        record.ts = columnText(stmt, 1);
        record.level = columnText(stmt, 2);
        record.code = columnText(stmt, 3);
        record.message = columnText(stmt, 4);
        result.push_back(std::move(record));
    }

    sqlite3_finalize(stmt);
    return result;
}

bool DiagnosticsRepositorySqlite::resetTestCart(int moduleId) {
    try {
        conn_.execute("BEGIN IMMEDIATE;");
        conn_.execute("DELETE FROM order_items;");
        conn_.execute("DELETE FROM orders;");
        conn_.execute("DELETE FROM reels;");
        conn_.execute("DELETE FROM operations;");
        conn_.execute(
            "UPDATE cart_workflow "
            "SET current_order_id = NULL, state = 'FREE', updated_at = datetime('now') "
            "WHERE id = 1;"
        );
        conn_.execute(
            "UPDATE slot_states "
            "SET state = 'FREE', updated_at = datetime('now') "
            "WHERE module_id = " + std::to_string(moduleId) + ";"
        );
        conn_.execute(
            "INSERT INTO event_log(ts, level, code, message) "
            "VALUES(datetime('now'), 'WARN', 'TestCartReset', 'module_id=" +
            std::to_string(moduleId) + "');"
        );
        conn_.execute("COMMIT;");
        return true;
    } catch (...) {
        try {
            conn_.execute("ROLLBACK;");
        } catch (...) {
        }
        return false;
    }
}

} // namespace smartcart::infrastructure::db
