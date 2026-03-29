#pragma once

#include <string>

#include "AppConfig.hpp"

namespace smartcart::infrastructure::config {

class ConfigLoader {
public:
    static AppConfig loadFromFile(const std::string& path);

private:
    static void validate(AppConfig& cfg);
};

} // namespace smartcart::infrastructure::config
