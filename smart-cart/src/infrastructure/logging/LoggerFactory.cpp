#include "LoggerFactory.hpp"
#include <spdlog/sinks/rotating_file_sink.h>

namespace smartcart::infrastructure::logging {

std::shared_ptr<spdlog::logger>
LoggerFactory::createFileLogger(const smartcart::infrastructure::config::AppConfig& cfg) {
    const auto& path = cfg.logFilePath.empty() ? cfg.logFile : cfg.logFilePath;
    auto sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(path, 5 * 1024 * 1024, 3);
    auto logger = std::make_shared<spdlog::logger>("smartcart", sink);
    logger->set_level(spdlog::level::info);
    logger->flush_on(spdlog::level::info);
    return logger;
}

} // namespace smartcart::infrastructure::logging
