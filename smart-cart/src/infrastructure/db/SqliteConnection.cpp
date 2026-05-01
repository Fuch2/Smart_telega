#include "infrastructure/db/SqliteConnection.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace smartcart::infrastructure::db {
namespace {

std::string readFileToString(const std::filesystem::path& filePath) {
    std::ifstream in(filePath, std::ios::in | std::ios::binary);
    if (!in.is_open()) {
        throw std::runtime_error("Не удалось открыть файл миграции: " + filePath.string());
    }

    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

void execSql(sqlite3* db, const std::string& sql) {
    char* errMsg = nullptr;
    const int rc = sqlite3_exec(db, sql.c_str(), nullptr, nullptr, &errMsg);
    if (rc != SQLITE_OK) {
        std::string err = errMsg ? errMsg : "unknown sqlite error";
        sqlite3_free(errMsg);
        throw std::runtime_error("Ошибка выполнения SQL: " + err);
    }
}

std::string migrationNameColumn(sqlite3* db) {
    const char* query = "PRAGMA table_info(schema_migrations);";
    sqlite3_stmt* stmt = nullptr;

    const int rcPrepare = sqlite3_prepare_v2(db, query, -1, &stmt, nullptr);
    if (rcPrepare != SQLITE_OK) {
        throw std::runtime_error("Не удалось прочитать схему schema_migrations");
    }

    bool hasFilename = false;
    bool hasName = false;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        const unsigned char* text = sqlite3_column_text(stmt, 1);
        if (text == nullptr) {
            continue;
        }

        const std::string column = reinterpret_cast<const char*>(text);
        if (column == "filename") {
            hasFilename = true;
        } else if (column == "name") {
            hasName = true;
        }
    }
    sqlite3_finalize(stmt);

    if (hasFilename) {
        return "filename";
    }
    if (hasName) {
        return "name";
    }

    throw std::runtime_error("schema_migrations не содержит filename/name");
}

std::set<std::string> loadAppliedMigrations(sqlite3* db,
                                            const std::string& nameColumn) {
    std::set<std::string> applied;

    const std::string query =
        "SELECT " + nameColumn + " FROM schema_migrations;";
    sqlite3_stmt* stmt = nullptr;

    const int rcPrepare = sqlite3_prepare_v2(db, query.c_str(), -1, &stmt, nullptr);
    if (rcPrepare != SQLITE_OK) {
        throw std::runtime_error("Не удалось подготовить SELECT schema_migrations");
    }

    while (true) {
        const int rcStep = sqlite3_step(stmt);
        if (rcStep == SQLITE_ROW) {
            const unsigned char* text = sqlite3_column_text(stmt, 0);
            if (text != nullptr) {
                applied.emplace(reinterpret_cast<const char*>(text));
            }
            continue;
        }
        if (rcStep == SQLITE_DONE) {
            break;
        }

        sqlite3_finalize(stmt);
        throw std::runtime_error("Ошибка чтения schema_migrations");
    }

    sqlite3_finalize(stmt);
    return applied;
}

void insertAppliedMigration(sqlite3* db,
                            const std::string& nameColumn,
                            const std::string& migrationName) {
    const std::string insertSql =
        "INSERT INTO schema_migrations(" + nameColumn +
        ", applied_at) VALUES(?, datetime('now'));";

    sqlite3_stmt* stmt = nullptr;
    const int rcPrepare = sqlite3_prepare_v2(db, insertSql.c_str(), -1, &stmt, nullptr);
    if (rcPrepare != SQLITE_OK) {
        throw std::runtime_error("Не удалось подготовить INSERT schema_migrations");
    }

    sqlite3_bind_text(stmt, 1, migrationName.c_str(), -1, SQLITE_TRANSIENT);

    const int rcStep = sqlite3_step(stmt);
    sqlite3_finalize(stmt);

    if (rcStep != SQLITE_DONE) {
        throw std::runtime_error("Не удалось записать применённую миграцию: " + migrationName);
    }
}

} // namespace

SqliteConnection::SqliteConnection(const std::string& sqlitePath) {
    const int rc = sqlite3_open(sqlitePath.c_str(), &db_);
    if (rc != SQLITE_OK || db_ == nullptr) {
        const std::string err = db_ ? sqlite3_errmsg(db_) : "sqlite3_open вернул null db";
        if (db_ != nullptr) {
            sqlite3_close(db_);
            db_ = nullptr;
        }
        throw std::runtime_error("Не удалось открыть SQLite БД: " + err);
    }

    // Включить WAL mode для concurrent reads
    char* errMsg = nullptr;
    sqlite3_exec(db_, "PRAGMA journal_mode=WAL;", nullptr, nullptr, &errMsg);
    if (errMsg != nullptr) {
        sqlite3_free(errMsg);
    }

    // Установить busy timeout (5 секунд) для retry при блокировках
    sqlite3_busy_timeout(db_, 5000);

    // Нормальный fsync (баланс между скоростью и надёжностью)
    sqlite3_exec(db_, "PRAGMA synchronous=NORMAL;", nullptr, nullptr, &errMsg);
    if (errMsg != nullptr) {
        sqlite3_free(errMsg);
    }
}

SqliteConnection::~SqliteConnection() {
    if (db_ != nullptr) {
        sqlite3_close(db_);
        db_ = nullptr;
    }
}

sqlite3* SqliteConnection::handle() const noexcept {
    return db_;
}

void SqliteConnection::execute(const std::string& sql) {
    if (db_ == nullptr) {
        throw std::runtime_error("execute: SQLite соединение не инициализировано");
    }
    execSql(db_, sql);
}

void SqliteConnection::beginTransaction() {
    if (db_ == nullptr) {
        throw std::runtime_error("beginTransaction: SQLite соединение не инициализировано");
    }
    execSql(db_, "BEGIN IMMEDIATE;");
}

void SqliteConnection::commit() {
    if (db_ == nullptr) {
        throw std::runtime_error("commit: SQLite соединение не инициализировано");
    }
    execSql(db_, "COMMIT;");
}

void SqliteConnection::rollback() {
    if (db_ == nullptr) {
        throw std::runtime_error("rollback: SQLite соединение не инициализировано");
    }
    execSql(db_, "ROLLBACK;");
}

void SqliteConnection::runMigrations(const std::filesystem::path& migrationsDir) {
    if (db_ == nullptr) {
        throw std::runtime_error("runMigrations: SQLite соединение не инициализировано");
    }

    // Таблица учёта применённых миграций
    execSql(
        db_,
        "CREATE TABLE IF NOT EXISTS schema_migrations ("
        "filename TEXT PRIMARY KEY NOT NULL,"
        "applied_at TEXT NOT NULL DEFAULT (datetime('now'))"
        ");"
    );

    const std::string nameColumn = migrationNameColumn(db_);
    std::filesystem::path dirPath(migrationsDir);
    if (!std::filesystem::exists(dirPath) || !std::filesystem::is_directory(dirPath)) {
        throw std::runtime_error("Каталог миграций не найден: " + migrationsDir.string());
    }

    std::vector<std::filesystem::path> migrationFiles;
    for (const auto& entry : std::filesystem::directory_iterator(dirPath)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        const auto& p = entry.path();
        if (p.extension() == ".sql") {
            migrationFiles.push_back(p);
        }
    }

    std::sort(migrationFiles.begin(), migrationFiles.end(),
              [](const auto& a, const auto& b) {
                  return a.filename().string() < b.filename().string();
              });

    const std::set<std::string> applied = loadAppliedMigrations(db_, nameColumn);

    for (const auto& filePath : migrationFiles) {
        const std::string fileName = filePath.filename().string();
        if (applied.find(fileName) != applied.end()) {
            continue; // Уже применено
        }

        const std::string sql = readFileToString(filePath);

        execSql(db_, "BEGIN IMMEDIATE TRANSACTION;");
        try {
            execSql(db_, sql);
            insertAppliedMigration(db_, nameColumn, fileName);
            execSql(db_, "COMMIT;");
        } catch (...) {
            try {
                execSql(db_, "ROLLBACK;");
            } catch (...) {
                // Игнорируем вторичную ошибку rollback
            }
            throw;
        }
    }
}

void SqliteConnection::runMigrations(const std::string& migrationsDir) {
    runMigrations(std::filesystem::path(migrationsDir));
}

} // namespace smartcart::infrastructure::db
