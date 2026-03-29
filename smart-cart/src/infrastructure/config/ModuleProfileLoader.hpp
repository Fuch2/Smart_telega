#pragma once

#include <string>
#include <vector>

namespace smartcart::infrastructure::config {

struct ModuleProfile {
    std::string moduleType;
    std::string profileName;
    std::vector<std::string> slots; // expected S01..S24
};

class ModuleProfileLoader {
public:
    static ModuleProfile loadFromFile(const std::string& path);

private:
    static void validate(const ModuleProfile& profile);
};

} // namespace smartcart::infrastructure::config
