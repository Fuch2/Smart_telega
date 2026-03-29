#pragma once
#include <memory>
#include <spdlog/logger.h>
#include "../config/AppConfig.hpp"

namespace smartcart::infrastructure::logging {

class LoggerFactory {
public:
    static std::shared_ptr<spdlog::logger>
    createFileLogger(const smartcart::infrastructure::config::AppConfig& cfg);
};

} // namespace smartcart::infrastructure::logging
