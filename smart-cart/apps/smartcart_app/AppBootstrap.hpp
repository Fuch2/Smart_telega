// ===== apps/smartcart_app/AppBootstrap.hpp =====
// Исправлено:
//   - unique_ptr<AdminViewModel> и unique_ptr<WorkerViewModel> с custom deleter
//     через std::default_delete forward-compatible паттерн:
//     объявляем деструктор в .hpp, определяем в .cpp где типы полные.
#pragma once

#include "infrastructure/config/AppConfig.hpp"
#include "infrastructure/db/SqliteConnection.hpp"
#include "infrastructure/db/repositories/ModuleRepositorySqlite.hpp"
#include "infrastructure/db/repositories/ReelRepositorySqlite.hpp"
#include "infrastructure/db/repositories/OperationRepositorySqlite.hpp"
#include "infrastructure/db/repositories/OrderRepositorySqlite.hpp"
#include "infrastructure/db/repositories/WorkflowRepositorySqlite.hpp"
#include "infrastructure/db/repositories/DiagnosticsRepositorySqlite.hpp"
#include "infrastructure/hw/stm32/UartStm32Link.hpp"
#include "infrastructure/hw/stm32/MockStm32Link.hpp"
#include "infrastructure/hw/rfid/Rc522RfidProvider.hpp"
#include "infrastructure/hw/rfid/MultiRc522RfidProvider.hpp"
#include "application/services/StartupService.hpp"
#include "application/services/AddReelService.hpp"
#include "application/services/ReplaceReelService.hpp"
#include "application/services/RecoveryService.hpp"
#include "application/services/RfidModuleMonitorService.hpp"
#include "application/services/Stm32PollingService.hpp"
#include "application/services/ModuleSnapshotPollingService.hpp"
#include "application/services/OrderImportService.hpp"
#include "application/services/BomOrderImportService.hpp"
#include "application/services/WorkflowService.hpp"
#include "application/services/AdminDiagnosticsService.hpp"
#include "application/state/AppStateMachine.hpp"
#include "infrastructure/logging/SqliteEventLogger.hpp"

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

class MainWindow;
class AdminViewModel;
class WorkerViewModel;

class AppBootstrap {
public:
    AppBootstrap(const std::filesystem::path& configPath,
                 const std::filesystem::path& migrationsDir);

    // ИСПРАВЛЕНО: деструктор объявлен здесь, определён в AppBootstrap.cpp
    // где AdminViewModel и WorkerViewModel — полные типы.
    // Без этого default_delete падает на sizeof(incomplete type).
    ~AppBootstrap();

    void launch();

private:
    struct ModuleChannelRuntime;

    smartcart::infrastructure::config::AppConfig cfg_;

    std::unique_ptr<smartcart::infrastructure::db::SqliteConnection>          conn_;
    std::unique_ptr<smartcart::infrastructure::db::ModuleRepositorySqlite>    moduleRepo_;
    std::unique_ptr<smartcart::infrastructure::db::ReelRepositorySqlite>      reelRepo_;
    std::unique_ptr<smartcart::infrastructure::db::OperationRepositorySqlite> opRepo_;
    std::unique_ptr<smartcart::infrastructure::db::OrderRepositorySqlite>     orderRepo_;
    std::unique_ptr<smartcart::infrastructure::db::WorkflowRepositorySqlite>  workflowRepo_;
    std::unique_ptr<smartcart::infrastructure::db::DiagnosticsRepositorySqlite> diagnosticsRepo_;
    std::unique_ptr<smartcart::infrastructure::logging::SqliteEventLogger>    eventLogger_;

    std::unique_ptr<smartcart::application::ports::IStm32Link>                stm32Link_;
    std::unique_ptr<smartcart::application::ports::IRfidProvider>             rfidProvider_;

    std::unique_ptr<smartcart::application::services::StartupService>         startupSvc_;
    std::unique_ptr<smartcart::application::services::AddReelService>         addReelSvc_;
    std::unique_ptr<smartcart::application::services::ReplaceReelService>     replaceReelSvc_;
    std::unique_ptr<smartcart::application::services::RecoveryService>        recoverySvc_;
    std::unique_ptr<smartcart::application::services::RfidModuleMonitorService> rfidMonitorSvc_;
    std::unique_ptr<smartcart::application::services::Stm32PollingService>    pollingSvc_;
    std::unique_ptr<smartcart::application::services::OrderImportService>     orderImportSvc_;
    std::unique_ptr<smartcart::application::services::BomOrderImportService>  bomOrderImportSvc_;
    std::unique_ptr<smartcart::application::services::WorkflowService>        workflowSvc_;
    std::unique_ptr<smartcart::application::services::AdminDiagnosticsService> adminDiagnosticsSvc_;

    std::unique_ptr<smartcart::application::AppStateMachine>                  stateMachine_;

    std::unique_ptr<AdminViewModel>  adminVm_;
    std::unique_ptr<WorkerViewModel> workerVm_;
    MainWindow*                      mainWindow_ = nullptr;
    smartcart::infrastructure::hw::stm32::MockStm32Link* mockLink_ = nullptr;
    int activeModuleId_ = 1;
    std::string activeModuleUid_;
    std::string activeStm32Device_;
    std::vector<std::unique_ptr<ModuleChannelRuntime>> moduleChannelRuntimes_;

    std::vector<int> slotToLedMap_;
    void buildSlotToLedMap();
    int resolveActiveModuleId();
    void syncModuleStatuses();
    int ensureModuleForUid(const std::string& uid);
    void buildActiveStm32Link();
    void buildModuleChannelRuntimes();
    void startModuleChannelRuntimes();
    void destroyModuleChannelRuntimes();
    void buildModuleScopedSession();
    void destroyModuleScopedSession(bool keepWindow = false);
    void showMainWindow();
    void switchToModuleUid(const std::string& uid);
};
