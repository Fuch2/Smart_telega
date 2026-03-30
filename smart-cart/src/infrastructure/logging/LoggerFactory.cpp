// ===== src/infrastructure/logging/LoggerFactory.cpp =====
// Исправлено: cfg.logFilePath → cfg.logFile
#include "LoggerFactory.hpp"
#include <spdlog/sinks/rotating_file_sink.h>

namespace smartcart::infrastructure::logging {

std::shared_ptr<spdlog::logger>
LoggerFactory::createFileLogger(
    const smartcart::infrastructure::config::AppConfig& cfg)
{
    auto sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        cfg.logFile,        // ← исправлено
        5 * 1024 * 1024,    // 5 MB per file
        3                   // 3 rotated files
    );
    auto logger = std::make_shared<spdlog::logger>("smartcart", sink);
    logger->set_level(spdlog::level::info);
    logger->flush_on(spdlog::level::info);
    return logger;
}

} // namespace smartcart::infrastructure::logging
