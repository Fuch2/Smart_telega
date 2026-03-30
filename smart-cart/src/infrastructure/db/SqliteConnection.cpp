// ===== src/infrastructure/db/SqliteConnection.cpp =====
// Исправлено: убраны кириллица-мусор в сообщениях об ошибках
#include "infrastructure/db/SqliteConnection.hpp"

#include <sqlite3.h>

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace smartcart::infrastructure::db {

SqliteConnection::SqliteConnection(const std::string& path) {
    if (sqlite3_open(path.c_str(), &db_) != SQLITE_OK) {
        std::string err = sqlite3_errmsg(db_);
        sqlite3_close(db_);
        db_ = nullptr;
        throw std::runtime_error(
            "SqliteConnection: failed to open database: " + err);
    }
    execute("PRAGMA journal_mode=WAL;");
    execute("PRAGMA foreign_keys=ON;");
}

SqliteConnection::~SqliteConnection() {
    if (db_) {
        sqlite3_close(db_);
        db_ = nullptr;
    }
}

void SqliteConnection::execute(const std::string& sql) {
    char* errMsg = nullptr;
    const int rc = sqlite3_exec(db_, sql.c_str(), nullptr, nullptr, &errMsg);
    if (rc != SQLITE_OK) {
        std::string err = errMsg ? errMsg : "unknown error";
        sqlite3_free(errMsg);
        throw std::runtime_error(
            "SqliteConnection::execute failed: " + err + "\nSQL: " + sql);
    }
}

void SqliteConnection::ensureMigrationsTable() {
    execute(R"sql(
        CREATE TABLE IF NOT EXISTS schema_migrations (
            filename   TEXT PRIMARY KEY NOT NULL,
            applied_at TEXT NOT NULL DEFAULT (datetime('now'))
        );
    )sql");
}

bool SqliteConnection::isMigrationApplied(const std::string& filename) {
    const char* sql =
        "SELECT COUNT(*) FROM schema_migrations WHERE filename = ?;";
    sqlite3_stmt* stmt = nullptr;

    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK)
        throw std::runtime_error(
            "SqliteConnection::isMigrationApplied: prepare failed");

    sqlite3_bind_text(stmt, 1, filename.c_str(), -1, SQLITE_STATIC);

    int count = 0;
    if (sqlite3_step(stmt) == SQLITE_ROW)
        count = sqlite3_column_int(stmt, 0);

    sqlite3_finalize(stmt);
    return count > 0;
}

void SqliteConnection::recordMigration(const std::string& filename) {
    const char* sql =
        "INSERT INTO schema_migrations (filename) VALUES (?);";
    sqlite3_stmt* stmt = nullptr;

    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK)
        throw std::runtime_error(
            "SqliteConnection::recordMigration: prepare failed");

    sqlite3_bind_text(stmt, 1, filename.c_str(), -1, SQLITE_STATIC);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);

    if (rc != SQLITE_DONE)
        throw std::runtime_error(
            "SqliteConnection::recordMigration: step failed");
}

void SqliteConnection::runMigrations(
    const std::filesystem::path& migrationsDir)
{
    ensureMigrationsTable();

    std::vector<std::filesystem::path> files;
    for (const auto& entry :
         std::filesystem::directory_iterator(migrationsDir))
    {
        if (entry.is_regular_file() &&
            entry.path().extension() == ".sql")
        {
            files.push_back(entry.path());
        }
    }
    std::sort(files.begin(), files.end());

    for (const auto& filePath : files) {
        const std::string filename = filePath.filename().string();
        if (isMigrationApplied(filename))
            continue;

        std::ifstream file(filePath);
        if (!file.is_open())
            throw std::runtime_error(
                "runMigrations: failed to open " + filePath.string());

        std::ostringstream ss;
        ss << file.rdbuf();

        transaction([&]() {
            execute(ss.str());
            recordMigration(filename);
        });
    }
}

void SqliteConnection::transaction(const std::function<void()>& fn) {
    execute("BEGIN IMMEDIATE;");
    try {
        fn();
        execute("COMMIT;");
    } catch (...) {
        execute("ROLLBACK;");
        throw;
    }
}

} // namespace smartcart::infrastructure::db
