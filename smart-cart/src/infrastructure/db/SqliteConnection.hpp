// ===== src/infrastructure/db/SqliteConnection.hpp =====
#pragma once

#include <sqlite3.h>

#include <filesystem>
#include <functional>
#include <stdexcept>
#include <string>

namespace smartcart::infrastructure::db {

class SqliteConnection {
public:
    explicit SqliteConnection(const std::string& path);
    ~SqliteConnection();

    SqliteConnection(const SqliteConnection&)            = delete;
    SqliteConnection& operator=(const SqliteConnection&) = delete;

    SqliteConnection(SqliteConnection&& other) noexcept
        : db_(other.db_) { other.db_ = nullptr; }

    SqliteConnection& operator=(SqliteConnection&& other) noexcept {
        if (this != &other) {
            if (db_) sqlite3_close(db_);
            db_       = other.db_;
            other.db_ = nullptr;
        }
        return *this;
    }

    void execute(const std::string& sql);
    void runMigrations(const std::filesystem::path& migrationsDir);
    void transaction(const std::function<void()>& fn);

    sqlite3* handle() const noexcept { return db_; }

private:
    sqlite3* db_ = nullptr;

    void ensureMigrationsTable();
    bool isMigrationApplied(const std::string& filename);
    void recordMigration(const std::string& filename);
};

} // namespace smartcart::infrastructure::db
