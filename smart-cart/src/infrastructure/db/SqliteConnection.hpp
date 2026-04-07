#pragma once

#include <sqlite3.h>

#include <filesystem>
#include <string>

namespace smartcart::infrastructure::db {

class SqliteConnection {
public:
    explicit SqliteConnection(const std::string& sqlitePath);
    ~SqliteConnection();

    SqliteConnection(const SqliteConnection&) = delete;
    SqliteConnection& operator=(const SqliteConnection&) = delete;

    SqliteConnection(SqliteConnection&&) = delete;
    SqliteConnection& operator=(SqliteConnection&&) = delete;

    sqlite3* handle() const noexcept;

    // Выполнить SQL без результата.
    void execute(const std::string& sql);

    // Прогон миграций из директории (все *.sql по алфавиту, idempotent)
    void runMigrations(const std::filesystem::path& migrationsDir);
    void runMigrations(const std::string& migrationsDir);

private:
    sqlite3* db_ {nullptr};
};

} // namespace smartcart::infrastructure::db
