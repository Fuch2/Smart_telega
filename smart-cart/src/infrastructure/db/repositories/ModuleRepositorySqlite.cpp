// ===== src/infrastructure/db/repositories/ModuleRepositorySqlite.cpp =====
// Исправлено: ensureSchema() создаёт slot_states, а не только modules
#include "ModuleRepositorySqlite.hpp"
#include "../../../domain/entities/ModuleInfo.hpp"

#include <sqlite3.h>
#include <stdexcept>
#include <string>

namespace smartcart::infrastructure::db {

using namespace smartcart::domain;

static const char* kSelectCols =
    "SELECT id, serial, slot_count, firmware, status, kind FROM modules";

namespace {

std::string columnText(sqlite3_stmt* stmt, int column) {
    const auto* text = sqlite3_column_text(stmt, column);
    return text ? reinterpret_cast<const char*>(text) : std::string{};
}

bool hasColumn(sqlite3* db, const char* table, const char* column) {
    const std::string sql = std::string("PRAGMA table_info(") + table + ");";
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
        return false;
    }

    bool found = false;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        if (columnText(stmt, 1) == column) {
            found = true;
            break;
        }
    }
    sqlite3_finalize(stmt);
    return found;
}

} // namespace

ModuleRepositorySqlite::ModuleRepositorySqlite(SqliteConnection& conn)
    : conn_(conn)
{
    ensureSchema();
}

void ModuleRepositorySqlite::ensureSchema() {
    // Таблица modules
    conn_.execute(
        "CREATE TABLE IF NOT EXISTS modules ("
        "  id         INTEGER PRIMARY KEY AUTOINCREMENT,"
        "  serial     TEXT    NOT NULL UNIQUE COLLATE NOCASE,"
        "  slot_count INTEGER NOT NULL DEFAULT 24,"
        "  firmware   TEXT    NOT NULL DEFAULT '',"
        "  status     TEXT    NOT NULL DEFAULT 'OFFLINE',"
        "  kind       TEXT    NOT NULL DEFAULT 'REEL'"
        ");"
    );
    if (!hasColumn(conn_.handle(), "modules", "kind")) {
        conn_.execute(
            "ALTER TABLE modules "
            "ADD COLUMN kind TEXT NOT NULL DEFAULT 'REEL';"
        );
    }

    // Таблица slot_states — используется updateSlotState()
    conn_.execute(
        "CREATE TABLE IF NOT EXISTS slot_states ("
        "  module_id  INTEGER NOT NULL,"
        "  slot_index INTEGER NOT NULL,"
        "  state      TEXT    NOT NULL DEFAULT 'FREE',"
        "  PRIMARY KEY (module_id, slot_index)"
        ");"
    );
}

ModuleInfo ModuleRepositorySqlite::rowToInfo(sqlite3_stmt* stmt) {
    ModuleInfo m;
    m.id        = sqlite3_column_int(stmt, 0);
    m.serial    = columnText(stmt, 1);
    m.slotCount = sqlite3_column_int(stmt, 2);
    m.firmware  = columnText(stmt, 3);
    m.status    = moduleStatusFromString(columnText(stmt, 4));
    m.kind      = moduleKindFromString(columnText(stmt, 5));
    return m;
}

std::vector<ModuleInfo> ModuleRepositorySqlite::getAll() {
    const std::string sql =
        std::string(kSelectCols) + " ORDER BY id;";
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(conn_.handle(), sql.c_str(), -1, &stmt, nullptr);

    std::vector<ModuleInfo> out;
    while (sqlite3_step(stmt) == SQLITE_ROW)
        out.push_back(rowToInfo(stmt));
    sqlite3_finalize(stmt);
    return out;
}

std::optional<ModuleInfo> ModuleRepositorySqlite::getById(int id) {
    const std::string sql =
        std::string(kSelectCols) + " WHERE id=?;";
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
        "WHERE serial=? COLLATE NOCASE "
        "AND (? = 0 OR id <> ?);";
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr);
    sqlite3_bind_text(stmt, 1, serial.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int (stmt, 2, exceptId);
    sqlite3_bind_int (stmt, 3, exceptId);

    bool exists = false;
    if (sqlite3_step(stmt) == SQLITE_ROW)
        exists = sqlite3_column_int(stmt, 0) > 0;
    sqlite3_finalize(stmt);
    return exists;
}

int ModuleRepositorySqlite::add(const ModuleInfo& m) {
    const char* sql =
        "INSERT INTO modules(serial, slot_count, firmware, status, kind) "
        "VALUES(?,?,?,?,?);";
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr);
    sqlite3_bind_text(stmt, 1, m.serial.c_str(),   -1, SQLITE_TRANSIENT);
    sqlite3_bind_int (stmt, 2, m.slotCount);
    sqlite3_bind_text(stmt, 3, m.firmware.c_str(), -1, SQLITE_TRANSIENT);

    const std::string statusStr{toString(m.status)};
    sqlite3_bind_text(stmt, 4, statusStr.c_str(), -1, SQLITE_TRANSIENT);
    const std::string kindStr{toString(m.kind)};
    sqlite3_bind_text(stmt, 5, kindStr.c_str(), -1, SQLITE_TRANSIENT);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    if (rc != SQLITE_DONE) return 0;
    return static_cast<int>(sqlite3_last_insert_rowid(conn_.handle()));
}

bool ModuleRepositorySqlite::update(const ModuleInfo& m) {
    const char* sql =
        "UPDATE modules "
        "SET serial=?, slot_count=?, firmware=?, status=?, kind=? "
        "WHERE id=?;";
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr);
    sqlite3_bind_text(stmt, 1, m.serial.c_str(),   -1, SQLITE_TRANSIENT);
    sqlite3_bind_int (stmt, 2, m.slotCount);
    sqlite3_bind_text(stmt, 3, m.firmware.c_str(), -1, SQLITE_TRANSIENT);

    const std::string statusStr{toString(m.status)};
    sqlite3_bind_text(stmt, 4, statusStr.c_str(), -1, SQLITE_TRANSIENT);
    const std::string kindStr{toString(m.kind)};
    sqlite3_bind_text(stmt, 5, kindStr.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int (stmt, 6, m.id);

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

bool ModuleRepositorySqlite::updateSlotState(int moduleId,
                                              int slotIndex,
                                              SlotState state) {
    const char* sql =
        "INSERT INTO slot_states(module_id, slot_index, state) VALUES(?,?,?)"
        " ON CONFLICT(module_id, slot_index) DO UPDATE SET state=excluded.state;";
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr);
    sqlite3_bind_int(stmt, 1, moduleId);
    sqlite3_bind_int(stmt, 2, slotIndex);

    const std::string stateStr{toString(state)};
    sqlite3_bind_text(stmt, 3, stateStr.c_str(), -1, SQLITE_TRANSIENT);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return rc == SQLITE_DONE;
}

} // namespace smartcart::infrastructure::db
