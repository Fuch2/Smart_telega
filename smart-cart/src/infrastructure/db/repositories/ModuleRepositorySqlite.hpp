#pragma once
#include <string>
#include <vector>

struct sqlite3;
namespace smartcart::infrastructure::db::repositories {

struct ModuleRow {
    int id{};
    std::string moduleSerial;
    std::string moduleType;
    std::string profileName;
};

class ModuleRepositorySqlite {
public:
    explicit ModuleRepositorySqlite(sqlite3* db);

    int upsert(const std::string& moduleSerial, const std::string& moduleType, const std::string& profileName);
    void upsertSlots(int moduleId, const std::vector<std::string>& slots);
    std::vector<std::string> listFreeSlots(int moduleId) const;
    std::vector<std::string> listAllSlots(int moduleId) const;

private:
    sqlite3* db_{nullptr};
};

} // namespace smartcart::infrastructure::db::repositories
