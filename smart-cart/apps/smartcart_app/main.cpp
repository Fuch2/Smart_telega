// ===== apps/smartcart_app/main.cpp =====
// Минимальный CLI entry point для CI smoke-test и диагностики workflow без UI.
#include "infrastructure/config/ConfigLoader.hpp"
#include "infrastructure/db/SqliteConnection.hpp"
#include "infrastructure/db/repositories/OrderRepositorySqlite.hpp"
#include "infrastructure/db/repositories/ReelRepositorySqlite.hpp"
#include "infrastructure/db/repositories/WorkflowRepositorySqlite.hpp"
#include "infrastructure/logging/SqliteEventLogger.hpp"
#include "application/services/OrderImportService.hpp"
#include "domain/entities/CartWorkflow.hpp"

#include <sqlite3.h>

#include <filesystem>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>

// Макросы должны быть определены через CMake target_compile_definitions
#ifndef CONFIG_DIR
#  error "CONFIG_DIR is not defined. Pass -DCONFIG_DIR=... via CMake."
#endif
#ifndef MIGRATIONS_DIR
#  error "MIGRATIONS_DIR is not defined. Pass -DMIGRATIONS_DIR=... via CMake."
#endif

namespace {

void printUsage(const char* appName) {
    std::cout
        << "Usage:\n"
        << "  " << appName << "\n"
        << "  " << appName << " --diag\n"
        << "  " << appName << " --import-order /path/order.json\n";
}

void printLatestEvents(sqlite3* db) {
    const char* sql =
        "SELECT id, ts, level, code, COALESCE(message, '') "
        "FROM event_log ORDER BY id DESC LIMIT 20;";
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        std::cout << "event_log: prepare failed\n";
        return;
    }

    std::cout << "\nLatest events:\n";
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        std::cout
            << "  #" << sqlite3_column_int(stmt, 0)
            << " " << reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1))
            << " [" << reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2))
            << "] " << reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3))
            << " " << reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4))
            << "\n";
    }
    sqlite3_finalize(stmt);
}

void printDiagnostics(
    smartcart::infrastructure::db::SqliteConnection& conn,
    smartcart::infrastructure::db::OrderRepositorySqlite& orderRepo,
    smartcart::infrastructure::db::ReelRepositorySqlite& reelRepo,
    smartcart::infrastructure::db::WorkflowRepositorySqlite& workflowRepo)
{
    namespace domain = smartcart::domain;

    const auto workflow = workflowRepo.get();
    std::cout << "Workflow: " << domain::toString(workflow.state) << "\n";

    if (workflow.currentOrderId.has_value()) {
        const auto order = orderRepo.getOrderById(*workflow.currentOrderId);
        if (order.has_value()) {
            std::cout << "Order: " << order->externalOrderId
                      << " / " << order->title
                      << " / " << domain::toString(order->status)
                      << "\n";

            std::cout << "\nOrder items:\n";
            for (const auto& item : orderRepo.getItems(order->id)) {
                std::cout << "  " << item.barcode
                          << " target_slot=" << item.targetSlot
                          << " status=" << domain::toString(item.status);
                if (item.currentSlot.has_value()) {
                    std::cout << " current_slot=" << *item.currentSlot;
                }
                std::cout << "\n";
            }
        }
    } else {
        std::cout << "Order: none\n";
    }

    std::cout << "\nActive reels:\n";
    for (const auto& reel : reelRepo.getActiveByModule(1)) {
        std::cout << "  " << reel.barcode
                  << " slot=" << reel.slotIndex
                  << " placed_at=" << reel.placedAt
                  << "\n";
    }

    printLatestEvents(conn.handle());
}

} // namespace

int main(int argc, char* argv[]) {
    try {
        namespace cfg = smartcart::infrastructure::config;
        namespace db  = smartcart::infrastructure::db;
        namespace logging = smartcart::infrastructure::logging;
        namespace services = smartcart::application::services;

        const std::string configPath = std::string(CONFIG_DIR) + "/config.json";

        auto config = cfg::ConfigLoader::loadFromFile(configPath);
        db::SqliteConnection conn(config.sqlitePath);
        conn.runMigrations(std::filesystem::path(MIGRATIONS_DIR));

        db::OrderRepositorySqlite orderRepo(conn);
        db::ReelRepositorySqlite reelRepo(conn);
        db::WorkflowRepositorySqlite workflowRepo(conn);
        logging::SqliteEventLogger logger(conn.handle());

        if (argc == 1) {
            std::cout << "DB OK: " << config.sqlitePath << std::endl;
            return 0;
        }

        const std::string command = argv[1];
        if (command == "--help" || command == "-h") {
            printUsage(argv[0]);
            return 0;
        }

        if (command == "--diag") {
            printDiagnostics(conn, orderRepo, reelRepo, workflowRepo);
            return 0;
        }

        if (command == "--import-order") {
            if (argc < 3) {
                throw std::runtime_error("--import-order requires JSON path");
            }

            services::OrderImportService importSvc(
                orderRepo,
                workflowRepo,
                reelRepo,
                logger,
                {}
            );
            const auto result = importSvc.importFromFile(argv[2]);
            if (!result) {
                std::cerr << "Order import failed: " << result.message << "\n";
                printDiagnostics(conn, orderRepo, reelRepo, workflowRepo);
                return 2;
            }

            std::cout << "Order imported: id=" << result.orderId << "\n";
            printDiagnostics(conn, orderRepo, reelRepo, workflowRepo);
            return 0;
        }

        printUsage(argv[0]);
        return 2;
    } catch (const std::exception& e) {
        std::cerr << "[FATAL] " << e.what() << std::endl;
        return 1;
    }
}
