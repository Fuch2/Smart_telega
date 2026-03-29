#include "OperationRepositorySqlite.hpp"
#include <sqlite3.h>
#include <stdexcept>

namespace smartcart::infrastructure::db::repositories {
namespace {
OperationRow readRow(sqlite3_stmt* st) {
    OperationRow r;
    r.id = sqlite3_column_int(st, 0);
    r.opType = reinterpret_cast<const char*>(sqlite3_column_text(st, 1));
    r.state = reinterpret_cast<const char*>(sqlite3_column_text(st, 2));
    r.targetBarcode = sqlite3_column_text(st, 3) ? reinterpret_cast<const char*>(sqlite3_column_text(st, 3)) : "";
    r.payloadJson = sqlite3_column_text(st, 4) ? reinterpret_cast<const char*>(sqlite3_column_text(st, 4)) : "";
    r.finished = sqlite3_column_type(st, 5) != SQLITE_NULL;
    return r;
}
}

OperationRepositorySqlite::OperationRepositorySqlite(sqlite3* db) : db_(db) {
    if (!db_) throw std::runtime_error("OperationRepositorySqlite: null db");
}

int OperationRepositorySqlite::create(const std::string& opType, const std::string& state,
                                      const std::string& targetBarcode, const std::string& payloadJson) {
    const char* sql =
        "INSERT INTO operations(op_type, state, target_barcode, payload_json) VALUES(?,?,?,?);";
    sqlite3_stmt* st = nullptr;
    sqlite3_prepare_v2(db_, sql, -1, &st, nullptr);
    sqlite3_bind_text(st, 1, opType.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(st, 2, state.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(st, 3, targetBarcode.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(st, 4, payloadJson.c_str(), -1, SQLITE_TRANSIENT);

    if (sqlite3_step(st) != SQLITE_DONE) {
        sqlite3_finalize(st);
        throw std::runtime_error("OperationRepositorySqlite::create failed");
    }
    sqlite3_finalize(st);
    return static_cast<int>(sqlite3_last_insert_rowid(db_));
}

void OperationRepositorySqlite::updateState(int id, const std::string& state, const std::string& payloadJson) {
    const char* sql =
        "UPDATE operations SET state=?, payload_json=?, updated_at=CURRENT_TIMESTAMP WHERE id=?;";
    sqlite3_stmt* st = nullptr;
    sqlite3_prepare_v2(db_, sql, -1, &st, nullptr);
    sqlite3_bind_text(st, 1, state.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(st, 2, payloadJson.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(st, 3, id);
    if (sqlite3_step(st) != SQLITE_DONE) {
        sqlite3_finalize(st);
        throw std::runtime_error("OperationRepositorySqlite::updateState failed");
    }
    sqlite3_finalize(st);
}

void OperationRepositorySqlite::finish(int id, const std::string& finalState, const std::string& payloadJson) {
    const char* sql =
        "UPDATE operations SET state=?, payload_json=?, updated_at=CURRENT_TIMESTAMP, "
        "finished_at=CURRENT_TIMESTAMP WHERE id=?;";
    sqlite3_stmt* st = nullptr;
    sqlite3_prepare_v2(db_, sql, -1, &st, nullptr);
    sqlite3_bind_text(st, 1, finalState.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(st, 2, payloadJson.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(st, 3, id);
    if (sqlite3_step(st) != SQLITE_DONE) {
        sqlite3_finalize(st);
        throw std::runtime_error("OperationRepositorySqlite::finish failed");
    }
    sqlite3_finalize(st);
}

std::optional<OperationRow> OperationRepositorySqlite::findById(int id) {
    const char* sql =
        "SELECT id, op_type, state, target_barcode, payload_json, finished_at FROM operations WHERE id=?;";
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

std::vector<OperationRow> OperationRepositorySqlite::listUnfinished() {
    const char* sql =
        "SELECT id, op_type, state, target_barcode, payload_json, finished_at "
        "FROM operations WHERE finished_at IS NULL ORDER BY id ASC;";
    sqlite3_stmt* st = nullptr;
    sqlite3_prepare_v2(db_, sql, -1, &st, nullptr);
    std::vector<OperationRow> out;
    while (sqlite3_step(st) == SQLITE_ROW) out.push_back(readRow(st));
    sqlite3_finalize(st);
    return out;
}

} // namespace smartcart::infrastructure::db::repositories
