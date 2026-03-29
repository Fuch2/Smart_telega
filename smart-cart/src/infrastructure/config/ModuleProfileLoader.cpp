#include "ModuleProfileLoader.hpp"

#include <fstream>
#include <stdexcept>
#include <unordered_set>

#include <nlohmann/json.hpp>

namespace smartcart::infrastructure::config {
namespace {
using json = nlohmann::json;
}

ModuleProfile ModuleProfileLoader::loadFromFile(const std::string& path) {
    std::ifstream in(path);
    if (!in.is_open()) {
        throw std::runtime_error("ModuleProfileLoader: cannot open file: " + path);
    }

    json j;
    in >> j;

    ModuleProfile p;
    if (j.contains("module_type")) p.moduleType = j.at("module_type").get<std::string>();
    if (j.contains("profile_name")) p.profileName = j.at("profile_name").get<std::string>();
    if (j.contains("slots")) p.slots = j.at("slots").get<std::vector<std::string>>();

    validate(p);
    return p;
}

void ModuleProfileLoader::validate(const ModuleProfile& profile) {
    if (profile.moduleType.empty()) {
        throw std::runtime_error("ModuleProfileLoader: module_type is empty");
    }
    if (profile.profileName.empty()) {
        throw std::runtime_error("ModuleProfileLoader: profile_name is empty");
    }
    if (profile.slots.size() != 24) {
        throw std::runtime_error("ModuleProfileLoader: slots must contain 24 entries for tray24 MVP");
    }

    std::unordered_set<std::string> uniq;
    for (const auto& s : profile.slots) {
        if (s.empty() || s[0] != 'S') {
            throw std::runtime_error("ModuleProfileLoader: invalid slot id: " + s);
        }
        if (!uniq.insert(s).second) {
            throw std::runtime_error("ModuleProfileLoader: duplicated slot id: " + s);
        }
    }
}

} // namespace smartcart::infrastructure::config
