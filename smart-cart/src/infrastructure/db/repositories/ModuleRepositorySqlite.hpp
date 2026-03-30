#pragma once
#include "../../../application/ports/IModuleRepository.hpp"
#include "../SqliteConnection.hpp"
struct sqlite3_stmt;

namespace smartcart::infrastructure::db {

class ModuleRepositorySqlite final
    : public application::ports::IModuleRepository {
public:
    explicit ModuleRepositorySqlite(SqliteConnection& conn);

    std::vector<domain::ModuleInfo> getAll() override;
    std::optional<domain::ModuleInfo> getById(int id) override;
    bool existsBySerial(const std::string& serial, int exceptId = 0) override;

    int  add(const domain::ModuleInfo& m) override;
    bool update(const domain::ModuleInfo& m) override;
    bool remove(int id) override;

private:
    SqliteConnection& conn_;

    static domain::ModuleInfo rowToInfo(sqlite3_stmt* stmt);
    void ensureSchema();
};

} // namespace smartcart::infrastructure::db
