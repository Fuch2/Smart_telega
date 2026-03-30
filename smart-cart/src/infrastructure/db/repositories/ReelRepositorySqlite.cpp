#include "ReelRepositorySqlite.hpp"
#include <sqlite3.h>
#include <stdexcept>

namespace smartcart::infrastructure::db {

using namespace smartcart::domain;

// col: id, barcode, module_id, slot_index, placed_at, removed_at
ReelRecord ReelRepositorySqlite::rowToRecord(sqlite3_stmt* stmt) {
    ReelRecord r;
    r.id         = sqlite3_column_int(stmt, 0);
    r.barcode    = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
    r.moduleId   = sqlite3_column_int(stmt, 2);
    r.slotIndex  = sqlite3_column_int(stmt, 3);
    r.placedAt   = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4));
    if (sqlite3_column_type(stmt, 5) != SQLITE_NULL)
        r.removedAt = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 5));
    return r;
}

// ── Ctor ─────────────────────────────────────────────────────────────────────

ReelRepositorySqlite::ReelRepositorySqlite(SqliteConnection& conn)
    : conn_(conn) {
    ensureSchema();
}

void ReelRepositorySqlite::ensureSchema() {
    conn_.execute(
        "CREATE TABLE IF NOT EXISTS reels ("
        "  id          INTEGER PRIMARY KEY AUTOINCREMENT,"
        "  barcode     TEXT    NOT NULL,"
        "  module_id   INTEGER NOT NULL,"
        "  slot_index  INTEGER NOT NULL,"
        "  placed_at   DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,"
        "  removed_at  DATETIME"
        ");"
    );
}

// ── Shared SQL ────────────────────────────────────────────────────────────────

static const char* kSelectCols =
    "SELECT id, barcode, module_id, slot_index, placed_at, removed_at "
    "FROM reels";

// ── IReelRepository ──────────────────────────────────────────────────────────

std::vector<ReelRecord> ReelRepositorySqlite::getAll() {
    const std::string sql = std::string(kSelectCols) + " ORDER BY id;";
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
        std::string(kSelectCols) + " WHERE module_id=? ORDER BY slot_index;";
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

} // namespace smartcart::infrastructure::db
