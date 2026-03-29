#include "ModuleRepositorySqlite.hpp"
#include <sqlite3.h>
#include <stdexcept>

namespace smartcart::infrastructure::db::repositories {

ModuleRepositorySqlite::ModuleRepositorySqlite(sqlite3* db) : db_(db) {
    if (!db_) throw std::runtime_error("ModuleRepositorySqlite: null db");
}

int ModuleRepositorySqlite::upsert(const std::string& moduleSerial, const std::string& moduleType, const std::string& profileName) {
    const char* sql =
        "INSERT INTO modules(module_serial, module_type, profile_name) VALUES(?,?,?) "
        "ON CONFLICT(module_serial) DO UPDATE SET module_type=excluded.module_type, profile_name=excluded.profile_name;";
    sqlite3_stmt* st = nullptr;
    sqlite3_prepare_v2(db_, sql, -1, &st, nullptr);
    sqlite3_bind_text(st, 1, moduleSerial.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(st, 2, moduleType.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(st, 3, profileName.c_str(), -1, SQLITE_TRANSIENT);
    if (sqlite3_step(st) != SQLITE_DONE) {
        sqlite3_finalize(st);
        throw std::runtime_error("ModuleRepositorySqlite::upsert failed");
    }
    sqlite3_finalize(st);

    const char* q = "SELECT id FROM modules WHERE module_serial=? LIMIT 1;";
    sqlite3_prepare_v2(db_, q, -1, &st, nullptr);
    sqlite3_bind_text(st, 1, moduleSerial.c_str(), -1, SQLITE_TRANSIENT);
    if (sqlite3_step(st) != SQLITE_ROW) {
        sqlite3_finalize(st);
        throw std::runtime_error("ModuleRepositorySqlite::upsert fetch id failed");
    }
    int id = sqlite3_column_int(st, 0);
    sqlite3_finalize(st);
    return id;
}

void ModuleRepositorySqlite::upsertSlots(int moduleId, const std::vector<std::string>& slots) {
    const char* sql =
        "INSERT INTO slots(module_id, slot_id, is_occupied) VALUES(?,?,0) "
        "ON CONFLICT(module_id, slot_id) DO NOTHING;";
    sqlite3_stmt* st = nullptr;
    for (const auto& s : slots) {
        sqlite3_prepare_v2(db_, sql, -1, &st, nullptr);
        sqlite3_bind_int(st, 1, moduleId);
        sqlite3_bind_text(st, 2, s.c_str(), -1, SQLITE_TRANSIENT);
        if (sqlite3_step(st) != SQLITE_DONE) {
            sqlite3_finalize(st);
            throw std::runtime_error("ModuleRepositorySqlite::upsertSlots failed");
        }
        sqlite3_finalize(st);
    }
}

std::vector<std::string> ModuleRepositorySqlite::listFreeSlots(int moduleId) const {
    const char* sql =
        "SELECT s.slot_id FROM slots s "
        "LEFT JOIN reels r ON r.slot_id = s.slot_id AND r.status='IN_CART' "
        "WHERE s.module_id=? AND r.id IS NULL ORDER BY s.slot_id;";
    sqlite3_stmt* st = nullptr;
    sqlite3_prepare_v2(db_, sql, -1, &st, nullptr);
    sqlite3_bind_int(st, 1, moduleId);
    std::vector<std::string> out;
    while (sqlite3_step(st) == SQLITE_ROW) {
        out.emplace_back(reinterpret_cast<const char*>(sqlite3_column_text(st, 0)));
    }
    sqlite3_finalize(st);
    return out;
}

std::vector<std::string> ModuleRepositorySqlite::listAllSlots(int moduleId) const {
    const char* sql = "SELECT slot_id FROM slots WHERE module_id=? ORDER BY slot_id;";
    sqlite3_stmt* st = nullptr;
    sqlite3_prepare_v2(db_, sql, -1, &st, nullptr);
    sqlite3_bind_int(st, 1, moduleId);
    std::vector<std::string> out;
    while (sqlite3_step(st) == SQLITE_ROW) {
        out.emplace_back(reinterpret_cast<const char*>(sqlite3_column_text(st, 0)));
    }
    sqlite3_finalize(st);
    return out;
}

} // namespace smartcart::infrastructure::db::repositories
