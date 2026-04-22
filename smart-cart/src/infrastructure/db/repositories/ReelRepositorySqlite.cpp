// ===== src/infrastructure/db/repositories/ReelRepositorySqlite.cpp =====
// Исправлено: ensureSchema() разбит на два отдельных execute() вместо одного
// (SQLite не поддерживает несколько statements в одном sqlite3_exec без callback)
#include "ReelRepositorySqlite.hpp"

#include <sqlite3.h>
#include <stdexcept>

namespace smartcart::infrastructure::db {

using namespace smartcart::domain;

static const char* kSelectCols =
    "SELECT id, barcode, module_id, slot_index, placed_at, removed_at "
    "FROM reels";

ReelRecord ReelRepositorySqlite::rowToRecord(sqlite3_stmt* stmt) {
    ReelRecord r;
    r.id        = sqlite3_column_int(stmt, 0);
    r.barcode   = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
    r.moduleId  = sqlite3_column_int(stmt, 2);
    r.slotIndex = sqlite3_column_int(stmt, 3);
    r.placedAt  = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4));

    if (sqlite3_column_type(stmt, 5) != SQLITE_NULL)
        r.removedAt =
            reinterpret_cast<const char*>(sqlite3_column_text(stmt, 5));
    else
        r.removedAt = std::nullopt;

    return r;
}

ReelRepositorySqlite::ReelRepositorySqlite(SqliteConnection& conn)
    : conn_(conn)
{
    ensureSchema();
}

void ReelRepositorySqlite::ensureSchema() {
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

    // ← разбито на два отдельных вызова: sqlite3_exec не поддерживает
    //   несколько statements в одной строке без специального callback
    conn_.execute(
        "CREATE TABLE IF NOT EXISTS reels ("
        "  id         INTEGER PRIMARY KEY AUTOINCREMENT,"
        "  barcode    TEXT    NOT NULL,"
        "  module_id  INTEGER NOT NULL,"
        "  slot_index INTEGER NOT NULL,"
        "  placed_at  DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,"
        "  removed_at DATETIME"
        ");"
    );

    conn_.execute(
        "CREATE TABLE IF NOT EXISTS slot_states ("
        "  module_id  INTEGER NOT NULL,"
        "  slot_index INTEGER NOT NULL,"
        "  state      TEXT    NOT NULL DEFAULT 'FREE',"
        "  PRIMARY KEY (module_id, slot_index)"
        ");"
    );
}

std::vector<ReelRecord> ReelRepositorySqlite::getAll() {
    const std::string sql =
        std::string(kSelectCols) + " ORDER BY id;";
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(conn_.handle(), sql.c_str(), -1, &stmt, nullptr);

    std::vector<ReelRecord> out;
    while (sqlite3_step(stmt) == SQLITE_ROW)
        out.push_back(rowToRecord(stmt));
    sqlite3_finalize(stmt);
    return out;
}

std::vector<ReelRecord> ReelRepositorySqlite::getByModule(int moduleId) {
    const std::string sql =
        std::string(kSelectCols) +
        " WHERE module_id=? ORDER BY slot_index;";
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(conn_.handle(), sql.c_str(), -1, &stmt, nullptr);
    sqlite3_bind_int(stmt, 1, moduleId);

    std::vector<ReelRecord> out;
    while (sqlite3_step(stmt) == SQLITE_ROW)
        out.push_back(rowToRecord(stmt));
    sqlite3_finalize(stmt);
    return out;
}

std::optional<ReelRecord> ReelRepositorySqlite::getBySlot(int moduleId,
                                                           int slotIndex) {
    const std::string sql =
        std::string(kSelectCols) +
        " WHERE module_id=? AND slot_index=? AND removed_at IS NULL"
        " ORDER BY id DESC LIMIT 1;";
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(conn_.handle(), sql.c_str(), -1, &stmt, nullptr);
    sqlite3_bind_int(stmt, 1, moduleId);
    sqlite3_bind_int(stmt, 2, slotIndex);

    std::optional<ReelRecord> out;
    if (sqlite3_step(stmt) == SQLITE_ROW)
        out = rowToRecord(stmt);
    sqlite3_finalize(stmt);
    return out;
}

std::vector<ReelRecord> ReelRepositorySqlite::getActive() {
    const std::string sql =
        std::string(kSelectCols) +
        " WHERE removed_at IS NULL ORDER BY module_id, slot_index;";
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(conn_.handle(), sql.c_str(), -1, &stmt, nullptr);

    std::vector<ReelRecord> out;
    while (sqlite3_step(stmt) == SQLITE_ROW)
        out.push_back(rowToRecord(stmt));
    sqlite3_finalize(stmt);
    return out;
}

std::vector<ReelRecord> ReelRepositorySqlite::getActiveByModule(int moduleId) {
    const std::string sql =
        std::string(kSelectCols) +
        " WHERE module_id=? AND removed_at IS NULL ORDER BY slot_index;";
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(conn_.handle(), sql.c_str(), -1, &stmt, nullptr);
    sqlite3_bind_int(stmt, 1, moduleId);

    std::vector<ReelRecord> out;
    while (sqlite3_step(stmt) == SQLITE_ROW)
        out.push_back(rowToRecord(stmt));
    sqlite3_finalize(stmt);
    return out;
}

std::optional<ReelRecord> ReelRepositorySqlite::findActiveByBarcode(
    const std::string& barcode)
{
    const std::string sql =
        std::string(kSelectCols) +
        " WHERE barcode=? AND removed_at IS NULL"
        " ORDER BY id DESC LIMIT 1;";
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(conn_.handle(), sql.c_str(), -1, &stmt, nullptr);
    sqlite3_bind_text(stmt, 1, barcode.c_str(), -1, SQLITE_TRANSIENT);

    std::optional<ReelRecord> out;
    if (sqlite3_step(stmt) == SQLITE_ROW)
        out = rowToRecord(stmt);
    sqlite3_finalize(stmt);
    return out;
}

bool ReelRepositorySqlite::hasActiveRecord(int moduleId, int slotIndex) {
    const char* sql =
        "SELECT COUNT(1) FROM reels "
        "WHERE module_id=? AND slot_index=? AND removed_at IS NULL;";
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr);
    sqlite3_bind_int(stmt, 1, moduleId);
    sqlite3_bind_int(stmt, 2, slotIndex);

    bool found = false;
    if (sqlite3_step(stmt) == SQLITE_ROW)
        found = sqlite3_column_int(stmt, 0) > 0;
    sqlite3_finalize(stmt);
    return found;
}

std::vector<Slot> ReelRepositorySqlite::getSlotStates(int moduleId) {
    const char* sql =
        "SELECT slot_index, state FROM slot_states "
        "WHERE module_id=? ORDER BY slot_index;";
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr);
    sqlite3_bind_int(stmt, 1, moduleId);

    std::vector<Slot> out;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        Slot s;
        s.moduleId  = moduleId;
        s.slotIndex = sqlite3_column_int(stmt, 0);
        const std::string stateStr =
            reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
        if      (stateStr == "OCCUPIED") s.state = SlotState::Occupied;
        else if (stateStr == "RESERVED") s.state = SlotState::Reserved;
        else if (stateStr == "ERROR")    s.state = SlotState::Error;
        else                             s.state = SlotState::Free;
        out.push_back(s);
    }
    sqlite3_finalize(stmt);
    return out;
}

int ReelRepositorySqlite::add(const ReelRecord& r) {
    const char* sql =
        "INSERT INTO reels(barcode, module_id, slot_index) VALUES(?,?,?);";
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr);
    sqlite3_bind_text(stmt, 1, r.barcode.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int (stmt, 2, r.moduleId);
    sqlite3_bind_int (stmt, 3, r.slotIndex);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    if (rc != SQLITE_DONE)
        throw std::runtime_error("ReelRepositorySqlite::add failed");
    return static_cast<int>(sqlite3_last_insert_rowid(conn_.handle()));
}

bool ReelRepositorySqlite::markRemoved(int id, const std::string& removedAt) {
    const char* sql =
        "UPDATE reels SET removed_at=? WHERE id=? AND removed_at IS NULL;";
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr);
    sqlite3_bind_text(stmt, 1, removedAt.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int (stmt, 2, id);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return rc == SQLITE_DONE && sqlite3_changes(conn_.handle()) > 0;
}

bool ReelRepositorySqlite::remove(int id) {
    const char* sql = "DELETE FROM reels WHERE id=?;";
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr);
    sqlite3_bind_int(stmt, 1, id);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return rc == SQLITE_DONE && sqlite3_changes(conn_.handle()) > 0;
}

bool ReelRepositorySqlite::markRemovedBySlot(int moduleId, int slotIndex) {
    const char* sql =
        "UPDATE reels "
        "SET removed_at = datetime('now') "
        "WHERE module_id=? AND slot_index=? AND removed_at IS NULL;";
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr);
    sqlite3_bind_int(stmt, 1, moduleId);
    sqlite3_bind_int(stmt, 2, slotIndex);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return rc == SQLITE_DONE && sqlite3_changes(conn_.handle()) > 0;
}

bool ReelRepositorySqlite::setSlotState(int moduleId,
                                         int slotIndex,
                                         SlotState state) {
    const char* sql =
        "INSERT INTO slot_states(module_id, slot_index, state) VALUES(?,?,?)"
        " ON CONFLICT(module_id, slot_index) DO UPDATE SET state=excluded.state;";
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr);
    sqlite3_bind_int (stmt, 1, moduleId);
    sqlite3_bind_int (stmt, 2, slotIndex);

    const std::string stateStr{toString(state)};
    sqlite3_bind_text(stmt, 3, stateStr.c_str(), -1, SQLITE_TRANSIENT);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return rc == SQLITE_DONE;
}

int ReelRepositorySqlite::addRecord(int moduleId,
                                     int slotIndex,
                                     const std::string& barcode) {
    const char* sql =
        "INSERT INTO reels(barcode, module_id, slot_index) VALUES(?,?,?);";
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr);
    sqlite3_bind_text(stmt, 1, barcode.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int (stmt, 2, moduleId);
    sqlite3_bind_int (stmt, 3, slotIndex);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    if (rc != SQLITE_DONE)
        throw std::runtime_error("ReelRepositorySqlite::addRecord failed");
    return static_cast<int>(sqlite3_last_insert_rowid(conn_.handle()));
}

} // namespace smartcart::infrastructure::db
