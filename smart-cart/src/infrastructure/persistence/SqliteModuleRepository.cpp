#include "SqliteModuleRepository.hpp"
#include <stdexcept>

SqliteModuleRepository::SqliteModuleRepository(const std::string& dbPath) {
    if (sqlite3_open(dbPath.c_str(), &db_) != SQLITE_OK) {
        throw std::runtime_error("Failed to open DB");
    }
    initSchema();
}

SqliteModuleRepository::~SqliteModuleRepository() {
    if (db_) sqlite3_close(db_);
}

void SqliteModuleRepository::execOrThrow(const std::string& sql) {
    char* err = nullptr;
    if (sqlite3_exec(db_, sql.c_str(), nullptr, nullptr, &err) != SQLITE_OK) {
        std::string msg = err ? err : "sqlite error";
        sqlite3_free(err);
        throw std::runtime_error(msg);
    }
}

void SqliteModuleRepository::initSchema() {
    execOrThrow(
        "CREATE TABLE IF NOT EXISTS modules ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT,"
        "serial TEXT NOT NULL UNIQUE,"
        "slot_count INTEGER NOT NULL,"
        "firmware TEXT NOT NULL,"
        "status TEXT NOT NULL"
        ");"
    );
}

std::vector<ModuleEntity> SqliteModuleRepository::getAll() {
    std::vector<ModuleEntity> out;

    const char* sql = "SELECT id, serial, slot_count, firmware, status FROM modules ORDER BY id;";
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);

    while (sqlite3_step(stmt) == SQLITE_ROW) {
        ModuleEntity m;
        m.id = sqlite3_column_int(stmt, 0);
        m.serial = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
        m.slotCount = sqlite3_column_int(stmt, 2);
        m.firmware = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3));
        m.status = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4));
        out.push_back(std::move(m));
    }

    sqlite3_finalize(stmt);
    return out;
}

std::optional<ModuleEntity> SqliteModuleRepository::getById(int id) {
    const char* sql = "SELECT id, serial, slot_count, firmware, status FROM modules WHERE id = ?;";
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
    sqlite3_bind_int(stmt, 1, id);

    std::optional<ModuleEntity> out;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        ModuleEntity m;
        m.id = sqlite3_column_int(stmt, 0);
        m.serial = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
        m.slotCount = sqlite3_column_int(stmt, 2);
        m.firmware = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3));
        m.status = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4));
        out = m;
    }

    sqlite3_finalize(stmt);
    return out;
}

bool SqliteModuleRepository::existsBySerial(const std::string& serial, int exceptId) {
    const char* sql =
        "SELECT COUNT(1) FROM modules "
        "WHERE lower(serial) = lower(?) AND (? = 0 OR id <> ?);";
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);

    sqlite3_bind_text(stmt, 1, serial.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 2, exceptId);
    sqlite3_bind_int(stmt, 3, exceptId);

    bool exists = false;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        exists = sqlite3_column_int(stmt, 0) > 0;
    }

    sqlite3_finalize(stmt);
    return exists;
}

int SqliteModuleRepository::add(const ModuleEntity& module) {
    const char* sql =
        "INSERT INTO modules(serial, slot_count, firmware, status) VALUES(?, ?, ?, ?);";
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);

    sqlite3_bind_text(stmt, 1, module.serial.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 2, module.slotCount);
    sqlite3_bind_text(stmt, 3, module.firmware.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 4, module.status.c_str(), -1, SQLITE_TRANSIENT);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);

    if (rc != SQLITE_DONE) return 0;
    return static_cast<int>(sqlite3_last_insert_rowid(db_));
}

bool SqliteModuleRepository::update(const ModuleEntity& module) {
    const char* sql =
        "UPDATE modules SET serial=?, slot_count=?, firmware=?, status=? WHERE id=?;";
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);

    sqlite3_bind_text(stmt, 1, module.serial.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 2, module.slotCount);
    sqlite3_bind_text(stmt, 3, module.firmware.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 4, module.status.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 5, module.id);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);

    return rc == SQLITE_DONE && sqlite3_changes(db_) > 0;
}

bool SqliteModuleRepository::remove(int id) {
    const char* sql = "DELETE FROM modules WHERE id=?;";
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
    sqlite3_bind_int(stmt, 1, id);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);

    return rc == SQLITE_DONE && sqlite3_changes(db_) > 0;
}
