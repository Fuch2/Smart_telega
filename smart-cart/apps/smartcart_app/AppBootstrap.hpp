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
#include "infrastructure/hw/stm32/UartStm32Link.hpp"
#include "infrastructure/hw/stm32/MockStm32Link.hpp"
#include "application/services/StartupService.hpp"
#include "application/services/AddReelService.hpp"
#include "application/services/ReplaceReelService.hpp"
#include "application/services/RecoveryService.hpp"
#include "application/state/AppStateMachine.hpp"

#include <filesystem>
#include <memory>
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
    smartcart::infrastructure::config::AppConfig cfg_;

    std::unique_ptr<smartcart::infrastructure::db::SqliteConnection>          conn_;
    std::unique_ptr<smartcart::infrastructure::db::ModuleRepositorySqlite>    moduleRepo_;
    std::unique_ptr<smartcart::infrastructure::db::ReelRepositorySqlite>      reelRepo_;
    std::unique_ptr<smartcart::infrastructure::db::OperationRepositorySqlite> opRepo_;

    std::unique_ptr<smartcart::application::ports::IStm32Link>                stm32Link_;

    std::unique_ptr<smartcart::application::services::StartupService>         startupSvc_;
    std::unique_ptr<smartcart::application::services::AddReelService>         addReelSvc_;
    std::unique_ptr<smartcart::application::services::ReplaceReelService>     replaceReelSvc_;
    std::unique_ptr<smartcart::application::services::RecoveryService>        recoverySvc_;

    std::unique_ptr<smartcart::application::AppStateMachine>                  stateMachine_;

    std::unique_ptr<AdminViewModel>  adminVm_;
    std::unique_ptr<WorkerViewModel> workerVm_;
    MainWindow*                      mainWindow_ = nullptr;

    std::vector<int> slotToLedMap_;
    void buildSlotToLedMap();
};
