// ===== apps/smartcart_app/main.cpp =====
// Минимальный CLI entry point для CI smoke-test и диагностики workflow без UI.
#include "infrastructure/config/ConfigLoader.hpp"
#include "infrastructure/db/SqliteConnection.hpp"
#include "infrastructure/db/repositories/OrderRepositorySqlite.hpp"
#include "infrastructure/db/repositories/ReelRepositorySqlite.hpp"
#include "infrastructure/db/repositories/WorkflowRepositorySqlite.hpp"
#include "infrastructure/hw/rfid/MultiRc522RfidProvider.hpp"
#include "infrastructure/hw/rfid/Rc522RfidProvider.hpp"
#include "infrastructure/logging/SqliteEventLogger.hpp"
#include "application/services/OrderImportService.hpp"
#include "application/services/WorkflowService.hpp"
#include "domain/entities/CartWorkflow.hpp"

#include <sqlite3.h>

#include <cstddef>
#include <filesystem>
#include <iostream>
#include <iomanip>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <ctime>

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
        << "  " << appName << " --import-order /path/order.json\n"
        << "  " << appName << " --workflow-next\n"
        << "  " << appName << " --issue BARCODE\n"
        << "  " << appName << " --return-leftover BARCODE_OR_SLOT\n"
        << "  " << appName << " --rfid-watch\n";
}

std::string nowText() {
    const auto now = std::time(nullptr);
    std::tm localTm{};
#ifdef _WIN32
    localtime_s(&localTm, &now);
#else
    localtime_r(&now, &localTm);
#endif

    std::ostringstream out;
    out << std::put_time(&localTm, "%H:%M:%S");
    return out.str();
}

int runRfidWatch(const smartcart::infrastructure::config::AppConfig& config) {
    if (!config.rfidEnabled) {
        std::cerr << "RFID is disabled in config.json\n";
        return 2;
    }

    std::unique_ptr<smartcart::application::ports::IRfidProvider> provider;
    if (config.rfidSpiDevices.size() > 1) {
        provider =
            std::make_unique<smartcart::infrastructure::hw::rfid::MultiRc522RfidProvider>(
                config.rfidSpiDevices);
    } else {
        const auto device = config.rfidSpiDevices.empty()
            ? config.rfidSpiDevice
            : config.rfidSpiDevices.front();
        provider =
            std::make_unique<smartcart::infrastructure::hw::rfid::Rc522RfidProvider>(
                device);
    }

    std::cout << "RFID watch started on";
    for (const auto& device : config.rfidSpiDevices) {
        std::cout << " " << device;
    }
    std::cout << "\n";
    std::cout << "Press Ctrl+C to stop.\n";

    std::optional<std::string> lastUid;
    bool lastWasNoTag = false;

    while (true) {
        const auto uids =
            provider->readAllOnce(static_cast<int>(config.rfidReadTimeoutMs));

        if (!uids.empty()) {
            const std::string text = [&uids]() {
                std::string out;
                for (const auto& uid : uids) {
                    if (!out.empty()) {
                        out += ", ";
                    }
                    out += uid;
                }
                return out;
            }();
            if (!lastUid.has_value() || *lastUid != text || lastWasNoTag) {
                std::cout << "[" << nowText() << "] UID: " << text << "\n";
                std::cout.flush();
            }
            lastUid = text;
            lastWasNoTag = false;
        } else {
            if (!lastWasNoTag) {
                std::cout << "[" << nowText() << "] NO_TAG\n";
                std::cout.flush();
            }
            lastUid.reset();
            lastWasNoTag = true;
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(config.rfidPollMs));
    }
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
                          << " part_number=" << item.partNumber
                          << " qty=" << item.requiredQuantity
                          << " priority=" << item.usagePriority
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

std::optional<int> parsePositiveSlot(const std::string& value) {
    try {
        std::size_t parsed = 0;
        const int slot = std::stoi(value, &parsed);
        if (parsed == value.size() && slot > 0) {
            return slot;
        }
    } catch (...) {
    }
    return std::nullopt;
}

smartcart::application::services::WorkflowActionResult runNextWorkflowStep(
    smartcart::application::services::WorkflowService& workflowSvc,
    smartcart::infrastructure::db::WorkflowRepositorySqlite& workflowRepo)
{
    namespace domain = smartcart::domain;
    namespace services = smartcart::application::services;

    const auto workflow = workflowRepo.get();
    switch (workflow.state) {
        case domain::CartWorkflowState::ReadyForFeederPrep:
            return workflowSvc.startFeederPrep();
        case domain::CartWorkflowState::FeederPrep:
            return workflowSvc.markFeederPrepCompleted();
        case domain::CartWorkflowState::ReadyForLine:
            return workflowSvc.startIssuingToLine();
        case domain::CartWorkflowState::IssuingToLine:
            return workflowSvc.completeIssuing();
        case domain::CartWorkflowState::OrderCompleted:
            return workflowSvc.inspectLeftoversAfterOrderCompleted();
        case domain::CartWorkflowState::LeftoversDetected:
            return workflowSvc.startReturningLeftovers();
        default:
            return services::WorkflowActionResult{
                false,
                domain::ErrorCode::Unknown,
                "Для состояния " + std::string(domain::toString(workflow.state)) +
                    " нет безопасного автоматического перехода"
            };
    }
}

int printActionResult(
    const smartcart::application::services::WorkflowActionResult& result,
    smartcart::infrastructure::db::SqliteConnection& conn,
    smartcart::infrastructure::db::OrderRepositorySqlite& orderRepo,
    smartcart::infrastructure::db::ReelRepositorySqlite& reelRepo,
    smartcart::infrastructure::db::WorkflowRepositorySqlite& workflowRepo)
{
    if (!result) {
        std::cerr << "Workflow action failed: " << result.message << "\n";
        printDiagnostics(conn, orderRepo, reelRepo, workflowRepo);
        return 2;
    }

    std::cout << "Workflow action OK: " << result.message << "\n";
    printDiagnostics(conn, orderRepo, reelRepo, workflowRepo);
    return 0;
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
        services::WorkflowService workflowSvc(
            orderRepo,
            workflowRepo,
            reelRepo,
            logger,
            1
        );

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

        if (command == "--rfid-watch") {
            return runRfidWatch(config);
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

        if (command == "--workflow-next") {
            const auto result = runNextWorkflowStep(workflowSvc, workflowRepo);
            return printActionResult(result, conn, orderRepo, reelRepo, workflowRepo);
        }

        if (command == "--issue") {
            if (argc < 3) {
                throw std::runtime_error("--issue requires barcode");
            }

            const auto result = workflowSvc.markItemIssued(argv[2]);
            return printActionResult(result, conn, orderRepo, reelRepo, workflowRepo);
        }

        if (command == "--return-leftover") {
            if (argc < 3) {
                throw std::runtime_error("--return-leftover requires barcode or slot");
            }

            const std::string value = argv[2];
            const auto slot = parsePositiveSlot(value);
            const auto result = slot.has_value()
                ? workflowSvc.markLeftoverReturnedBySlot(*slot)
                : workflowSvc.markLeftoverReturnedByBarcode(value);
            return printActionResult(result, conn, orderRepo, reelRepo, workflowRepo);
        }

        printUsage(argv[0]);
        return 2;
    } catch (const std::exception& e) {
        std::cerr << "[FATAL] " << e.what() << std::endl;
        return 1;
    }
}
