#pragma once
#include "../domain/entities/ModuleEntity.hpp"
#include <vector>
#include <optional>
#include <string>

class IModuleRepository {
public:
    virtual ~IModuleRepository() = default;

    virtual std::vector<ModuleEntity> getAll() = 0;
    virtual std::optional<ModuleEntity> getById(int id) = 0;
    virtual bool existsBySerial(const std::string& serial, int exceptId = 0) = 0;

    virtual int add(const ModuleEntity& module) = 0;       // returns new id
    virtual bool update(const ModuleEntity& module) = 0;   // by module.id
    virtual bool remove(int id) = 0;
};
