// ===== apps/smartcart_app/AppBootstrap.cpp =====
#include "AppBootstrap.hpp"

#include "infrastructure/config/ConfigLoader.hpp"
#include "infrastructure/hw/stm32/MockStm32Link.hpp"
#include "presentation/qt/MainWindow.hpp"
#include "presentation/qt/viewmodels/AdminViewModel.hpp"
#include "presentation/qt/viewmodels/WorkerViewModel.hpp"

#include <QMetaObject>
#include <QObject>
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
    orderRepo_  = std::make_unique<db::OrderRepositorySqlite>(*conn_);
    workflowRepo_ = std::make_unique<db::WorkflowRepositorySqlite>(*conn_);
    eventLogger_ = std::make_unique<logging::SqliteEventLogger>(conn_->handle());

    // ── 3. LED map ────────────────────────────────────────────────────────────
    buildSlotToLedMap();

    // ── 4. STM32 link ─────────────────────────────────────────────────────────
    if (cfg_.demoMode) {
        auto mock = std::make_unique<hw::stm32::MockStm32Link>(
            [](const hw::stm32::Frame& cmd) -> std::optional<hw::stm32::Frame> {
                hw::stm32::Frame resp;
                resp.seq   = cmd.seq;
                resp.cmdId = cmd.cmdId;

                if (cmd.cmdId == hw::stm32::CommandId::GetSwitchSnapshot) {
                    resp.type    = hw::stm32::FrameType::Resp;
                    resp.payload = {0x00, 0x00, 0x00};
                } else if (cmd.cmdId == hw::stm32::CommandId::GetReadyState) {
                    resp.type    = hw::stm32::FrameType::Resp;
                    resp.payload = {0x01};
                } else {
                    resp.type = hw::stm32::FrameType::Ack;
                }
                return resp;
            }
        );
        mockLink_  = mock.get();   // сохраняем сырой указатель ДО move
        stm32Link_ = std::move(mock);
        stm32Link_->open();
    } else {
        auto uart = std::make_unique<hw::stm32::UartStm32Link>(
            cfg_.stm32Device, 115200, 500
        );
        if (!uart->open())
            throw std::runtime_error(
                "AppBootstrap: failed to open UART " + cfg_.stm32Device);
        stm32Link_ = std::move(uart);
        // mockLink_ остаётся nullptr — MainWindow не покажет demo-панель
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

    workflowSvc_ = std::make_unique<WorkflowService>(
        *orderRepo_,
        *workflowRepo_,
        *reelRepo_,
        *eventLogger_,
        1
    );

    Stm32PollingConfig pollingCfg;
    pollingCfg.moduleId  = 1;
    pollingCfg.slotCount = 24;
    pollingCfg.pollMs    = static_cast<int>(cfg_.stm32PollMs);
    pollingCfg.debounceMs = static_cast<int>(cfg_.debounceMs);
    pollingCfg.trackedChannels = {1, 3};
    pollingCfg.ignoredChannels = {11};

    pollingSvc_ = std::make_unique<Stm32PollingService>(
        *stm32Link_,
        *reelRepo_,
        *opRepo_,
        *orderRepo_,
        *workflowRepo_,
        *workflowSvc_,
        *eventLogger_,
        pollingCfg
    );

    OrderImportConfig orderImportCfg;
    orderImportCfg.moduleId = 1;
    orderImportSvc_ = std::make_unique<OrderImportService>(
        *orderRepo_,
        *workflowRepo_,
        *reelRepo_,
        *eventLogger_,
        orderImportCfg
    );

    // ── 6. State machine ──────────────────────────────────────────────────────
    stateMachine_ = std::make_unique<AppStateMachine>(
        *startupSvc_,
        *addReelSvc_,
        *replaceReelSvc_,
        *recoverySvc_,
        *reelRepo_,
        *pollingSvc_
    );

    // ── 7. ViewModels ─────────────────────────────────────────────────────────
    adminVm_  = std::make_unique<AdminViewModel>(*moduleRepo_);
    workerVm_ = std::make_unique<WorkerViewModel>(
        *reelRepo_,
        *opRepo_,
        *orderRepo_,
        *workflowRepo_,
        *orderImportSvc_,
        *workflowSvc_,
        *stateMachine_
    );
}

AppBootstrap::~AppBootstrap() = default;

void AppBootstrap::buildSlotToLedMap() {
    if (!cfg_.slotToLedMap.empty()) {
        slotToLedMap_ = cfg_.slotToLedMap;
        return;
    }
    slotToLedMap_.resize(24);
    for (int i = 0; i < 24; ++i)
        slotToLedMap_[i] = i * 2;
}

void AppBootstrap::launch() {
    // mockLink_ == nullptr в prod-режиме → MainWindow не покажет demo-панель
    mainWindow_ = new MainWindow(adminVm_.get(), workerVm_.get(), mockLink_);
    mainWindow_->show();

    QObject::connect(
        stateMachine_.get(),
        &AppStateMachine::stateChanged,
        mainWindow_,
        [this](AppState state) {
            if (state == AppState::Ready && pollingSvc_) {
                pollingSvc_->start();
            }
        }
    );

    QMetaObject::invokeMethod(
        stateMachine_.get(),
        "startup",
        Qt::QueuedConnection
    );
}
