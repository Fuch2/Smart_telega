#pragma once
#include <string>

struct ModuleEntity {
    int id{0};
    std::string serial;
    int slotCount{24};
    std::string firmware;
    std::string status;   // ONLINE / OFFLINE / MAINT
};