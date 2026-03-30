// ===== src/infrastructure/logging/SpdlogFileLogger.cpp =====
#include "SpdlogFileLogger.hpp"
#include <spdlog/spdlog.h>

namespace smartcart::infrastructure::logging {

SpdlogFileLogger::SpdlogFileLogger(
    std::shared_ptr<spdlog::logger> logger)
    : logger_(std::move(logger))
{}

void SpdlogFileLogger::log(std::string_view level,
                           std::string_view code,
                           std::string_view message)
{
    const std::string msg =
        "[" + std::string(code) + "] " + std::string(message);

    if (level == "ERROR" || level == "error")
        logger_->error(msg);
    else if (level == "WARN" || level == "warn")
        logger_->warn(msg);
    else
        logger_->info(msg);
}

} // namespace smartcart::infrastructure::logging
