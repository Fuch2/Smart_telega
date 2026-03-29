#include "ReelRepositorySqlite.hpp"
#include <sqlite3.h>
#include <stdexcept>

namespace smartcart::infrastructure::db::repositories {

namespace {
ReelRow readRow(sqlite3_stmt* st) {
    ReelRow r;
    r.id = sqlite3_column_int(st, 0);
    r.barcode = reinterpret_cast<const char*>(sqlite3_column_text(st, 1));
    r.slotId = reinterpret_cast<const char*>(sqlite3_column_text(st, 2));
    r.status = reinterpret_cast<const char*>(sqlite3_column_text(st, 3));
    return r;
}
}

ReelRepositorySqlite::ReelRepositorySqlite(sqlite3* db) : db_(db) {
    if (!db_) throw std::runtime_error("ReelRepositorySqlite: null db");
}

int ReelRepositorySqlite::create(const std::string& barcode, const std::string& slotId, const std::string& status) {
    const char* sql = "INSERT INTO reels(barcode, slot_id, status) VALUES(?,?,?);";
    sqlite3_stmt* st = nullptr;
    sqlite3_prepare_v2(db_, sql, -1, &st, nullptr);
    sqlite3_bind_text(st, 1, barcode.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(st, 2, slotId.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(st, 3, status.c_str(), -1, SQLITE_TRANSIENT);
    if (sqlite3_step(st) != SQLITE_DONE) {
        sqlite3_finalize(st);
        throw std::runtime_error("ReelRepositorySqlite::create failed");
    }
    sqlite3_finalize(st);
    return static_cast<int>(sqlite3_last_insert_rowid(db_));
}

std::optional<ReelRow> ReelRepositorySqlite::findById(int id) {
    const char* sql = "SELECT id, barcode, slot_id, status FROM reels WHERE id=?;";
    sqlite3_stmt* st = nullptr;
    sqlite3_prepare_v2(db_, sql, -1, &st, nullptr);
    sqlite3_bind_int(st, 1, id);
    if (sqlite3_step(st) == SQLITE_ROW) {
        auto row = readRow(st);
        sqlite3_finalize(st);
        return row;
    }
    sqlite3_finalize(st);
    return std::nullopt;
}

std::optional<ReelRow> ReelRepositorySqlite::findActiveByBarcode(const std::string& barcode) {
    const char* sql =
        "SELECT id, barcode, slot_id, status FROM reels "
        "WHERE barcode=? AND status IN ('IN_CART','RESERVED') "
        "ORDER BY id DESC LIMIT 1;";
    sqlite3_stmt* st = nullptr;
    sqlite3_prepare_v2(db_, sql, -1, &st, nullptr);
    sqlite3_bind_text(st, 1, barcode.c_str(), -1, SQLITE_TRANSIENT);
    if (sqlite3_step(st) == SQLITE_ROW) {
        auto row = readRow(st);
        sqlite3_finalize(st);
        return row;
    }
    sqlite3_finalize(st);
    return std::nullopt;
}

std::optional<ReelRow> ReelRepositorySqlite::findBySlot(const std::string& slotId) {
    const char* sql =
        "SELECT id, barcode, slot_id, status FROM reels "
        "WHERE slot_id=? AND status='IN_CART' ORDER BY id DESC LIMIT 1;";
    sqlite3_stmt* st = nullptr;
    sqlite3_prepare_v2(db_, sql, -1, &st, nullptr);
    sqlite3_bind_text(st, 1, slotId.c_str(), -1, SQLITE_TRANSIENT);
    if (sqlite3_step(st) == SQLITE_ROW) {
        auto row = readRow(st);
        sqlite3_finalize(st);
        return row;
    }
    sqlite3_finalize(st);
    return std::nullopt;
}

std::vector<ReelRow> ReelRepositorySqlite::listByBarcode(const std::string& barcode) {
    const char* sql = "SELECT id, barcode, slot_id, status FROM reels WHERE barcode=? ORDER BY id DESC;";
    sqlite3_stmt* st = nullptr;
    sqlite3_prepare_v2(db_, sql, -1, &st, nullptr);
    sqlite3_bind_text(st, 1, barcode.c_str(), -1, SQLITE_TRANSIENT);
    std::vector<ReelRow> out;
    while (sqlite3_step(st) == SQLITE_ROW) out.push_back(readRow(st));
    sqlite3_finalize(st);
    return out;
}

void ReelRepositorySqlite::updateStatus(int id, const std::string& status) {
    const char* sql = "UPDATE reels SET status=?, updated_at=CURRENT_TIMESTAMP WHERE id=?;";
    sqlite3_stmt* st = nullptr;
    sqlite3_prepare_v2(db_, sql, -1, &st, nullptr);
    sqlite3_bind_text(st, 1, status.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(st, 2, id);
    if (sqlite3_step(st) != SQLITE_DONE) {
        sqlite3_finalize(st);
        throw std::runtime_error("ReelRepositorySqlite::updateStatus failed");
    }
    sqlite3_finalize(st);
}

void ReelRepositorySqlite::moveToSlot(int id, const std::string& slotId) {
    const char* sql = "UPDATE reels SET slot_id=?, updated_at=CURRENT_TIMESTAMP WHERE id=?;";
    sqlite3_stmt* st = nullptr;
    sqlite3_prepare_v2(db_, sql, -1, &st, nullptr);
    sqlite3_bind_text(st, 1, slotId.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(st, 2, id);
    if (sqlite3_step(st) != SQLITE_DONE) {
        sqlite3_finalize(st);
        throw std::runtime_error("ReelRepositorySqlite::moveToSlot failed");
    }
    sqlite3_finalize(st);
}

} // namespace smartcart::infrastructure::db::repositories
