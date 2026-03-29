#include "SqliteConnection.hpp"
#include <sqlite3.h>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace smartcart::infrastructure::db {

SqliteConnection::SqliteConnection(const std::string& dbPath) {
    if (sqlite3_open(dbPath.c_str(), &db_) != SQLITE_OK) {
        throw std::runtime_error("SqliteConnection: cannot open db");
    }
    execute("PRAGMA foreign_keys = ON;");
}

SqliteConnection::~SqliteConnection() {
    if (db_) sqlite3_close(db_);
}

void SqliteConnection::execute(const std::string& sql) {
    char* err = nullptr;
    const int rc = sqlite3_exec(db_, sql.c_str(), nullptr, nullptr, &err);
    if (rc != SQLITE_OK) {
        std::string msg = err ? err : "unknown sqlite error";
        sqlite3_free(err);
        throw std::runtime_error("SqliteConnection::execute: " + msg);
    }
}

void SqliteConnection::runMigrations(const std::string& migrationDir) {
    std::ifstream in(migrationDir + "/001_init.sql");
    if (!in.is_open()) throw std::runtime_error("Cannot open migration 001_init.sql");
    std::stringstream ss;
    ss << in.rdbuf();
    execute(ss.str());
}

void SqliteConnection::transaction(const std::function<void()>& fn) {
    execute("BEGIN IMMEDIATE TRANSACTION;");
    try {
        fn();
        execute("COMMIT;");
    } catch (...) {
        try { execute("ROLLBACK;"); } catch (...) {}
        throw;
    }
}

} // namespace smartcart::infrastructure::db
