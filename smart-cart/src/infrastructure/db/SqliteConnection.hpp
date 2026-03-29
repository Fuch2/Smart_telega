#pragma once
#include <functional>
#include <string>

struct sqlite3;

namespace smartcart::infrastructure::db {

class SqliteConnection {
public:
    explicit SqliteConnection(const std::string& dbPath);
    ~SqliteConnection();

    SqliteConnection(const SqliteConnection&) = delete;
    SqliteConnection& operator=(const SqliteConnection&) = delete;

    sqlite3* handle() const { return db_; }

    void execute(const std::string& sql);
    void runMigrations(const std::string& migrationDir);

    void transaction(const std::function<void()>& fn);

private:
    sqlite3* db_{nullptr};
};

} // namespace smartcart::infrastructure::db
