// ===== apps/smartcart_app/AppBootstrap.hpp =====
// Исправлено:
//   - убраны неиспользуемые includes (LoggerFactory, SqliteEventLogger)
//   - добавлен include <QMetaObject> для invokeMethod
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

// Forward declarations Qt
class MainWindow;
class AdminViewModel;
class WorkerViewModel;

class AppBootstrap {
public:
    AppBootstrap(const std::filesystem::path& configPath,
                 const std::filesystem::path& migrationsDir);

    /// Создаёт MainWindow и показывает его. Вызывать после QApplication.
    void launch();

private:
    // ── Config ────────────────────────────────────────────────────────────────
    smartcart::infrastructure::config::AppConfig cfg_;

    // ── DB ────────────────────────────────────────────────────────────────────
    std::unique_ptr<smartcart::infrastructure::db::SqliteConnection>          conn_;
    std::unique_ptr<smartcart::infrastructure::db::ModuleRepositorySqlite>    moduleRepo_;
    std::unique_ptr<smartcart::infrastructure::db::ReelRepositorySqlite>      reelRepo_;
    std::unique_ptr<smartcart::infrastructure::db::OperationRepositorySqlite> opRepo_;

    // ── HW ────────────────────────────────────────────────────────────────────
    std::unique_ptr<smartcart::application::ports::IStm32Link>                stm32Link_;

    // ── Services ──────────────────────────────────────────────────────────────
    std::unique_ptr<smartcart::application::services::StartupService>         startupSvc_;
    std::unique_ptr<smartcart::application::services::AddReelService>         addReelSvc_;
    std::unique_ptr<smartcart::application::services::ReplaceReelService>     replaceReelSvc_;
    std::unique_ptr<smartcart::application::services::RecoveryService>        recoverySvc_;

    // ── State machine ─────────────────────────────────────────────────────────
    std::unique_ptr<smartcart::application::AppStateMachine>                  stateMachine_;

    // ── Presentation ──────────────────────────────────────────────────────────
    // ViewModels хранятся здесь — MainWindow не владеет ими.
    // MainWindow* — raw ptr, owned by Qt parent (nullptr → Qt не удаляет,
    // но AppBootstrap живёт всё время работы приложения).
    std::unique_ptr<AdminViewModel>  adminVm_;
    std::unique_ptr<WorkerViewModel> workerVm_;
    MainWindow*                      mainWindow_ = nullptr;

    // ── Helpers ───────────────────────────────────────────────────────────────
    std::vector<int> slotToLedMap_;
    void buildSlotToLedMap();
};
