// ===== apps/smartcart_app/AppBootstrap.cpp =====
// Исправлено:
//   - убраны кириллица-мусор в строках ошибок
//   - добавлен #include <QMetaObject>
//   - mainWindow_ создаётся без parent → Qt не удалит его раньше AppBootstrap
#include "AppBootstrap.hpp"

#include "infrastructure/config/ConfigLoader.hpp"
#include "presentation/qt/MainWindow.hpp"
#include "presentation/qt/viewmodels/AdminViewModel.hpp"
#include "presentation/qt/viewmodels/WorkerViewModel.hpp"

#include <QMetaObject>
#include <stdexcept>

using namespace smartcart;
using namespace smartcart::infrastructure;
using namespace smartcart::application;
using namespace smartcart::application::services;

AppBootstrap::AppBootstrap(const std::filesystem::path& configPath,
                           const std::filesystem::path& migrationsDir)
{
    // ── 1. Config ─────────────────────────────────────────────────────────────
    cfg_ = config::ConfigLoader::loadFromFile(configPath.string());

    // ── 2. DB ─────────────────────────────────────────────────────────────────
    conn_ = std::make_unique<db::SqliteConnection>(cfg_.sqlitePath);
    conn_->runMigrations(migrationsDir);

    moduleRepo_ = std::make_unique<db::ModuleRepositorySqlite>(*conn_);
    reelRepo_   = std::make_unique<db::ReelRepositorySqlite>(*conn_);
    opRepo_     = std::make_unique<db::OperationRepositorySqlite>(*conn_);

    // ── 3. LED map ────────────────────────────────────────────────────────────
    buildSlotToLedMap();

    // ── 4. STM32 link ─────────────────────────────────────────────────────────
    if (cfg_.demoMode) {
        // Mock: всегда отвечает Ack; snapshot — все слоты пусты
        stm32Link_ = std::make_unique<hw::stm32::MockStm32Link>(
            [](const hw::stm32::Frame& cmd) -> std::optional<hw::stm32::Frame> {
                hw::stm32::Frame resp;
                resp.seq   = cmd.seq;
                resp.cmdId = cmd.cmdId;

                if (cmd.cmdId == hw::stm32::CommandId::GetSwitchSnapshot) {
                    resp.type    = hw::stm32::FrameType::Resp;
                    resp.payload = {0x00, 0x00, 0x00}; // 24 бита = все пусты
                } else if (cmd.cmdId == hw::stm32::CommandId::GetReadyState) {
                    resp.type    = hw::stm32::FrameType::Resp;
                    resp.payload = {0x01};              // ready
                } else {
                    resp.type = hw::stm32::FrameType::Ack;
                }
                return resp;
            }
        );
        stm32Link_->open();
    } else {
        auto uart = std::make_unique<hw::stm32::UartStm32Link>(
            "/dev/ttyUSB0", 115200, 500
        );
        if (!uart->open())
            throw std::runtime_error(
                "AppBootstrap: failed to open UART /dev/ttyUSB0");
        stm32Link_ = std::move(uart);
    }

    // ── 5. Services ───────────────────────────────────────────────────────────
    StartupConfig startupCfg;
    startupCfg.moduleId       = 1;
    startupCfg.slotCount      = 24;
    startupCfg.readyTimeoutMs = 5000;
    startupCfg.slotToLedMap   = slotToLedMap_;

    startupSvc_ = std::make_unique<StartupService>(
        *stm32Link_, *reelRepo_, *moduleRepo_, startupCfg
    );

    AddReelConfig addCfg;
    addCfg.moduleId        = 1;
    addCfg.slotCount       = 24;
    addCfg.stableConfirmMs = cfg_.stableConfirmMs;
    addCfg.slotToLedMap    = slotToLedMap_;

    addReelSvc_ = std::make_unique<AddReelService>(
        *stm32Link_, *reelRepo_, *opRepo_, addCfg
    );

    ReplaceReelConfig replaceCfg;
    replaceCfg.moduleId        = 1;
    replaceCfg.stableConfirmMs = cfg_.stableConfirmMs;
    replaceCfg.slotToLedMap    = slotToLedMap_;

    replaceReelSvc_ = std::make_unique<ReplaceReelService>(
        *stm32Link_, *reelRepo_, *opRepo_, replaceCfg
    );

    RecoveryConfig recoveryCfg;
    recoveryCfg.moduleId     = 1;
    recoveryCfg.slotCount    = 24;
    recoveryCfg.slotToLedMap = slotToLedMap_;

    recoverySvc_ = std::make_unique<RecoveryService>(
        *opRepo_, *reelRepo_, *stm32Link_, recoveryCfg
    );

    // ── 6. State machine ──────────────────────────────────────────────────────
    stateMachine_ = std::make_unique<AppStateMachine>(
        *startupSvc_, *addReelSvc_, *replaceReelSvc_, *recoverySvc_
    );

    // ── 7. ViewModels ─────────────────────────────────────────────────────────
    adminVm_  = std::make_unique<AdminViewModel>(*moduleRepo_);
    workerVm_ = std::make_unique<WorkerViewModel>(
        *reelRepo_, *opRepo_, *stateMachine_
    );
}

void AppBootstrap::buildSlotToLedMap() {
    if (!cfg_.slotToLedMap.empty()) {
        slotToLedMap_ = cfg_.slotToLedMap;
        return;
    }
    // Дефолт: слот N → LED (N-1)*2
    slotToLedMap_.resize(24);
    for (int i = 0; i < 24; ++i)
        slotToLedMap_[i] = i * 2;
}

void AppBootstrap::launch() {
    // mainWindow_ без parent — Qt не удалит его раньше AppBootstrap
    mainWindow_ = new MainWindow(adminVm_.get(), workerVm_.get());
    mainWindow_->show();

    // Запустить инициализацию после показа окна (через event loop)
    QMetaObject::invokeMethod(
        stateMachine_.get(),
        "startup",
        Qt::QueuedConnection
    );
}
