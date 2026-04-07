#include "SqliteEventLogger.hpp"
#include <sqlite3.h>
#include <stdexcept>
#include <string>

namespace smartcart::infrastructure::logging {

SqliteEventLogger::SqliteEventLogger(sqlite3* db) : db_(db) {
    if (!db_) throw std::runtime_error("SqliteEventLogger: db is null");

    const char* sql =
        "CREATE TABLE IF NOT EXISTS event_log ("
        "  id      INTEGER PRIMARY KEY AUTOINCREMENT,"
        "  ts      DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,"
        "  level   TEXT     NOT NULL,"
        "  code    TEXT     NOT NULL,"
        "  message TEXT     NOT NULL"
        ");";
    char* errMsg = nullptr;
    const int rc = sqlite3_exec(db_, sql, nullptr, nullptr, &errMsg);
    if (rc != SQLITE_OK) {
        std::string err = errMsg ? errMsg : "unknown sqlite error";
        sqlite3_free(errMsg);
        throw std::runtime_error("SqliteEventLogger: schema failed: " + err);
    }
}

void SqliteEventLogger::log(std::string_view level, std::string_view code, std::string_view message) {
    static constexpr const char* kSql =
        "INSERT INTO event_log(level, code, message) VALUES(?, ?, ?);";

    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db_, kSql, -1, &stmt, nullptr) != SQLITE_OK) {
        throw std::runtime_error("SqliteEventLogger: prepare failed");
    }

    sqlite3_bind_text(stmt, 1, level.data(), static_cast<int>(level.size()), SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, code.data(), static_cast<int>(code.size()), SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 3, message.data(), static_cast<int>(message.size()), SQLITE_TRANSIENT);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);

    if (rc != SQLITE_DONE) {
        throw std::runtime_error("SqliteEventLogger: insert failed");
    }
}

} // namespace smartcart::infrastructure::logging
