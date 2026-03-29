#pragma once
#include "../../repositories/IModuleRepository.hpp"
#include <sqlite3.h>
#include <string>

class SqliteModuleRepository final : public IModuleRepository {
public:
    explicit SqliteModuleRepository(const std::string& dbPath);
    ~SqliteModuleRepository() override;

    std::vector<ModuleEntity> getAll() override;
    std::optional<ModuleEntity> getById(int id) override;
    bool existsBySerial(const std::string& serial, int exceptId = 0) override;

    int add(const ModuleEntity& module) override;
    bool update(const ModuleEntity& module) override;
    bool remove(int id) override;

private:
    sqlite3* db_{nullptr};

    void initSchema();
    void execOrThrow(const std::string& sql);
};
