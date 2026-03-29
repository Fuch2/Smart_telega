#include "SqliteEventLogger.hpp"
#include <sqlite3.h>
#include <stdexcept>

namespace smartcart::infrastructure::logging {

SqliteEventLogger::SqliteEventLogger(sqlite3* db) : db_(db) {
    if (!db_) throw std::runtime_error("SqliteEventLogger: db is null");
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
