#include "ModuleRepositorySqlite.hpp"
#include "../../../domain/entities/ModuleInfo.hpp"
#include <sqlite3.h>
#include <stdexcept>

namespace smartcart::infrastructure::db {

using namespace smartcart::domain;

ModuleRepositorySqlite::ModuleRepositorySqlite(SqliteConnection& conn)
    : conn_(conn) {
    ensureSchema();
}

void ModuleRepositorySqlite::ensureSchema() {
    conn_.execute(
        "CREATE TABLE IF NOT EXISTS modules ("
        "  id         INTEGER PRIMARY KEY AUTOINCREMENT,"
        "  serial     TEXT    NOT NULL UNIQUE COLLATE NOCASE,"
        "  slot_count INTEGER NOT NULL DEFAULT 24,"
        "  firmware   TEXT    NOT NULL DEFAULT '',"
        "  status     TEXT    NOT NULL DEFAULT 'OFFLINE'"
        ");"
    );
}

// ── helpers ──────────────────────────────────────────────────────────────────

static const char* kSelectCols =
    "SELECT id, serial, slot_count, firmware, status FROM modules";

ModuleInfo ModuleRepositorySqlite::rowToInfo(sqlite3_stmt* stmt) {
    ModuleInfo m;
    m.id        = sqlite3_column_int(stmt, 0);
    m.serial    = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
    m.slotCount = sqlite3_column_int(stmt, 2);
    m.firmware  = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3));
    m.status    = moduleStatusFromString(
        reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4)));
    return m;
}

// ── IModuleRepository ────────────────────────────────────────────────────────

std::vector<ModuleInfo> ModuleRepositorySqlite::getAll() {
    std::vector<ModuleInfo> out;
    const std::string sql =
        std::string(kSelectCols) + " ORDER BY id;";

    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(conn_.handle(), sql.c_str(), -1, &stmt, nullptr);
    while (sqlite3_step(stmt) == SQLITE_ROW)
        out.push_back(rowToInfo(stmt));
    sqlite3_finalize(stmt);
    return out;
}

std::optional<ModuleInfo> ModuleRepositorySqlite::getById(int id) {
    const std::string sql =
        std::string(kSelectCols) + " WHERE id = ?;";

    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(conn_.handle(), sql.c_str(), -1, &stmt, nullptr);
    sqlite3_bind_int(stmt, 1, id);

    std::optional<ModuleInfo> out;
    if (sqlite3_step(stmt) == SQLITE_ROW)
        out = rowToInfo(stmt);
    sqlite3_finalize(stmt);
    return out;
}

bool ModuleRepositorySqlite::existsBySerial(const std::string& serial,
                                             int exceptId) {
    const char* sql =
        "SELECT COUNT(1) FROM modules "
        "WHERE serial = ? COLLATE NOCASE "
        "AND (? = 0 OR id <> ?);";

    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr);
    sqlite3_bind_text(stmt, 1, serial.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 2, exceptId);
    sqlite3_bind_int(stmt, 3, exceptId);

    bool exists = false;
    if (sqlite3_step(stmt) == SQLITE_ROW)
        exists = sqlite3_column_int(stmt, 0) > 0;
    sqlite3_finalize(stmt);
    return exists;
}

int ModuleRepositorySqlite::add(const ModuleInfo& m) {
    const char* sql =
        "INSERT INTO modules(serial, slot_count, firmware, status) "
        "VALUES(?, ?, ?, ?);";

    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr);
    sqlite3_bind_text(stmt, 1, m.serial.c_str(),   -1, SQLITE_TRANSIENT);
    sqlite3_bind_int (stmt, 2, m.slotCount);
    sqlite3_bind_text(stmt, 3, m.firmware.c_str(), -1, SQLITE_TRANSIENT);

    const std::string statusStr{toString(m.status)};
    sqlite3_bind_text(stmt, 4, statusStr.c_str(),  -1, SQLITE_TRANSIENT);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    if (rc != SQLITE_DONE) return 0;
    return static_cast<int>(sqlite3_last_insert_rowid(conn_.handle()));
}

bool ModuleRepositorySqlite::update(const ModuleInfo& m) {
    const char* sql =
        "UPDATE modules "
        "SET serial=?, slot_count=?, firmware=?, status=? "
        "WHERE id=?;";

    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr);
    sqlite3_bind_text(stmt, 1, m.serial.c_str(),   -1, SQLITE_TRANSIENT);
    sqlite3_bind_int (stmt, 2, m.slotCount);
    sqlite3_bind_text(stmt, 3, m.firmware.c_str(), -1, SQLITE_TRANSIENT);

    const std::string statusStr{toString(m.status)};
    sqlite3_bind_text(stmt, 4, statusStr.c_str(),  -1, SQLITE_TRANSIENT);
    sqlite3_bind_int (stmt, 5, m.id);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return rc == SQLITE_DONE && sqlite3_changes(conn_.handle()) > 0;
}

bool ModuleRepositorySqlite::remove(int id) {
    const char* sql = "DELETE FROM modules WHERE id=?;";
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr);
    sqlite3_bind_int(stmt, 1, id);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return rc == SQLITE_DONE && sqlite3_changes(conn_.handle()) > 0;
}

} // namespace smartcart::infrastructure::db
