// ===== apps/smartcart_app/AppBootstrap.cpp =====
#include "AppBootstrap.hpp"

#include "infrastructure/config/ConfigLoader.hpp"
#include "infrastructure/hw/rfid/Rc522RfidProvider.hpp"
#include "infrastructure/hw/stm32/MockStm32Link.hpp"
#include "presentation/qt/MainWindow.hpp"
#include "presentation/qt/viewmodels/AdminViewModel.hpp"
#include "presentation/qt/viewmodels/WorkerViewModel.hpp"

#include <QApplication>
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
    diagnosticsRepo_ = std::make_unique<db::DiagnosticsRepositorySqlite>(*conn_);
    eventLogger_ = std::make_unique<logging::SqliteEventLogger>(conn_->handle());

    if (cfg_.rfidEnabled) {
        rfidProvider_ = std::make_unique<hw::rfid::Rc522RfidProvider>(
            cfg_.rfidSpiDevice
        );
    }

    activeModuleId_ = resolveActiveModuleId();
    syncModuleStatuses();

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

    buildModuleScopedSession();
}

AppBootstrap::~AppBootstrap() {
    destroyModuleScopedSession();
}

void AppBootstrap::buildSlotToLedMap() {
    if (!cfg_.slotToLedMap.empty()) {
        slotToLedMap_ = cfg_.slotToLedMap;
        return;
    }
    slotToLedMap_.resize(24);
    for (int i = 0; i < 24; ++i)
        slotToLedMap_[i] = i * 2;
}

int AppBootstrap::resolveActiveModuleId() {
    if (!cfg_.rfidEnabled) {
        return 1;
    }

    try {
        if (!rfidProvider_) {
            return 1;
        }
        const auto uid = rfidProvider_->readOnce(static_cast<int>(cfg_.rfidReadTimeoutMs));
        if (!uid.has_value() || uid->empty()) {
            eventLogger_->log("WARN",
                              "RfidModuleDetectionFallback",
                              "RFID-метка модуля не найдена, используем module_id=1");
            return 1;
        }

        activeModuleUid_ = *uid;
        const std::string serial = "RFID-" + *uid;

        for (const auto& module : moduleRepo_->getAll()) {
            if (module.serial == serial) {
                eventLogger_->log("INFO",
                                  "RfidModuleDetected",
                                  "uid=" + *uid +
                                  " module_id=" + std::to_string(module.id));
                return module.id;
            }
        }

        domain::ModuleInfo module;
        module.serial = serial;
        module.slotCount = 24;
        module.firmware = "rfid-auto";
        module.status = domain::ModuleStatus::Online;

        const int moduleId = moduleRepo_->add(module);
        if (moduleId > 0) {
            eventLogger_->log("INFO",
                              "RfidModuleRegistered",
                              "uid=" + *uid +
                              " module_id=" + std::to_string(moduleId));
            return moduleId;
        }
    } catch (const std::exception& ex) {
        eventLogger_->log("ERROR", "RfidModuleDetectionFailed", ex.what());
    } catch (...) {
        eventLogger_->log("ERROR",
                          "RfidModuleDetectionFailed",
                          "Неизвестная ошибка RFID-определения модуля");
    }

    eventLogger_->log("WARN",
                      "RfidModuleDetectionFallback",
                      "Используем module_id=1 после ошибки RFID");
    return 1;
}

void AppBootstrap::syncModuleStatuses() {
    const bool hasActivePhysicalModule =
        !cfg_.rfidEnabled || !activeModuleUid_.empty();

    for (auto module : moduleRepo_->getAll()) {
        const auto targetStatus =
            (hasActivePhysicalModule && module.id == activeModuleId_)
                ? smartcart::domain::ModuleStatus::Online
                : smartcart::domain::ModuleStatus::Offline;
        if (module.status == targetStatus) {
            continue;
        }
        module.status = targetStatus;
        moduleRepo_->update(module);
    }
}

int AppBootstrap::ensureModuleForUid(const std::string& uid) {
    const std::string serial = "RFID-" + uid;

    for (const auto& module : moduleRepo_->getAll()) {
        if (module.serial == serial) {
            return module.id;
        }
    }

    smartcart::domain::ModuleInfo module;
    module.serial = serial;
    module.slotCount = 24;
    module.firmware = "rfid-auto";
    module.status = smartcart::domain::ModuleStatus::Offline;

    return moduleRepo_->add(module);
}

void AppBootstrap::buildModuleScopedSession() {
    orderRepo_ = std::make_unique<db::OrderRepositorySqlite>(*conn_, activeModuleId_);
    workflowRepo_ = std::make_unique<db::WorkflowRepositorySqlite>(*conn_, activeModuleId_);

    StartupConfig startupCfg;
    startupCfg.moduleId = activeModuleId_;
    startupCfg.slotCount = 24;
    startupCfg.readyTimeoutMs = 5000;
    startupCfg.slotToLedMap = slotToLedMap_;
    startupCfg.ignoredChannels = cfg_.switchIgnoredChannels;
    startupCfg.channelToSlotMap = cfg_.switchChannelToSlotMap;
    startupSvc_ = std::make_unique<StartupService>(
        *stm32Link_, *reelRepo_, *moduleRepo_, startupCfg);

    AddReelConfig addCfg;
    addCfg.moduleId = activeModuleId_;
    addCfg.slotCount = 24;
    addCfg.stableConfirmMs = cfg_.stableConfirmMs;
    addCfg.slotToLedMap = slotToLedMap_;
    addReelSvc_ = std::make_unique<AddReelService>(
        *stm32Link_, *reelRepo_, *opRepo_, addCfg);

    ReplaceReelConfig replaceCfg;
    replaceCfg.moduleId = activeModuleId_;
    replaceCfg.stableConfirmMs = cfg_.stableConfirmMs;
    replaceCfg.slotToLedMap = slotToLedMap_;
    replaceReelSvc_ = std::make_unique<ReplaceReelService>(
        *stm32Link_, *reelRepo_, *opRepo_, replaceCfg);

    RecoveryConfig recoveryCfg;
    recoveryCfg.moduleId = activeModuleId_;
    recoveryCfg.slotCount = 24;
    recoveryCfg.slotToLedMap = slotToLedMap_;
    recoverySvc_ = std::make_unique<RecoveryService>(
        *opRepo_, *reelRepo_, *stm32Link_, recoveryCfg);

    workflowSvc_ = std::make_unique<WorkflowService>(
        *orderRepo_, *workflowRepo_, *reelRepo_, *eventLogger_, activeModuleId_);

    Stm32PollingConfig pollingCfg;
    pollingCfg.moduleId = activeModuleId_;
    pollingCfg.slotCount = 24;
    pollingCfg.pollMs = static_cast<int>(cfg_.stm32PollMs);
    pollingCfg.debounceMs = static_cast<int>(cfg_.debounceMs);
    pollingCfg.snapshotFallbackMs =
        static_cast<int>(cfg_.stableConfirmMs + cfg_.stm32PollMs);
    pollingCfg.trackedChannels = cfg_.switchTrackedChannels;
    pollingCfg.ignoredChannels = cfg_.switchIgnoredChannels;
    pollingCfg.channelToSlotMap = cfg_.switchChannelToSlotMap;
    pollingSvc_ = std::make_unique<Stm32PollingService>(
        *stm32Link_,
        *reelRepo_,
        *opRepo_,
        *orderRepo_,
        *workflowRepo_,
        *workflowSvc_,
        *eventLogger_,
        pollingCfg);

    OrderImportConfig orderImportCfg;
    orderImportCfg.moduleId = activeModuleId_;
    orderImportSvc_ = std::make_unique<OrderImportService>(
        *orderRepo_, *workflowRepo_, *reelRepo_, *eventLogger_, orderImportCfg);

    AdminDiagnosticsConfig adminDiagnosticsCfg;
    adminDiagnosticsCfg.moduleId = activeModuleId_;
    adminDiagnosticsCfg.trackedChannels = cfg_.switchTrackedChannels;
    adminDiagnosticsCfg.ignoredChannels = cfg_.switchIgnoredChannels;
    adminDiagnosticsCfg.channelToSlotMap = cfg_.switchChannelToSlotMap;
    adminDiagnosticsSvc_ = std::make_unique<AdminDiagnosticsService>(
        *pollingSvc_, *diagnosticsRepo_, adminDiagnosticsCfg);

    if (cfg_.rfidEnabled &&
        rfidProvider_)
    {
        RfidModuleMonitorConfig monitorCfg;
        monitorCfg.moduleId = activeModuleId_;
        monitorCfg.pollMs = static_cast<int>(cfg_.rfidPollMs);
        monitorCfg.readTimeoutMs = static_cast<int>(cfg_.rfidReadTimeoutMs);
        monitorCfg.offlineTimeoutMs =
            static_cast<int>(cfg_.rfidOfflineTimeoutMs);
        monitorCfg.expectedUid = activeModuleUid_;

        rfidMonitorSvc_ = std::make_unique<RfidModuleMonitorService>(
            *rfidProvider_, *moduleRepo_, *eventLogger_, monitorCfg);
        rfidMonitorSvc_->setModuleSwitchCallback([this](std::string uid) {
            if (uid.empty() || uid == activeModuleUid_) {
                return;
            }
            QMetaObject::invokeMethod(
                qApp,
                [this, uid = std::move(uid)]() {
                    switchToModuleUid(uid);
                },
                Qt::QueuedConnection
            );
        });
    }

    stateMachine_ = std::make_unique<AppStateMachine>(
        *startupSvc_,
        *addReelSvc_,
        *replaceReelSvc_,
        *recoverySvc_,
        *reelRepo_,
        *pollingSvc_);

    adminVm_ = std::make_unique<AdminViewModel>(*moduleRepo_, *adminDiagnosticsSvc_);
    workerVm_ = std::make_unique<WorkerViewModel>(
        *moduleRepo_,
        *reelRepo_,
        *opRepo_,
        *orderRepo_,
        *workflowRepo_,
        *orderImportSvc_,
        *workflowSvc_,
        *stateMachine_);
}

void AppBootstrap::destroyModuleScopedSession(bool keepWindow) {
    if (keepWindow && mainWindow_) {
        mainWindow_->beginModuleSwitch();
    }

    if (stm32Link_) {
        stm32Link_->setEventCallback({});
    }
    if (rfidMonitorSvc_) {
        rfidMonitorSvc_->stop();
    }
    if (pollingSvc_) {
        pollingSvc_->stop();
    }
    if (!keepWindow && mainWindow_) {
        delete mainWindow_;
        mainWindow_ = nullptr;
    }

    workerVm_.reset();
    adminVm_.reset();
    stateMachine_.reset();
    adminDiagnosticsSvc_.reset();
    orderImportSvc_.reset();
    workflowSvc_.reset();
    pollingSvc_.reset();
    rfidMonitorSvc_.reset();
    recoverySvc_.reset();
    replaceReelSvc_.reset();
    addReelSvc_.reset();
    startupSvc_.reset();
    workflowRepo_.reset();
    orderRepo_.reset();
}

void AppBootstrap::showMainWindow() {
    if (!mainWindow_) {
        mainWindow_ = new MainWindow(adminVm_.get(), workerVm_.get(), mockLink_);
        mainWindow_->show();
    } else {
        mainWindow_->finishModuleSwitch(adminVm_.get(), workerVm_.get());
    }

    QObject::connect(
        stateMachine_.get(),
        &AppStateMachine::stateChanged,
        mainWindow_,
        [this](AppState state) {
            if (state == AppState::Ready && pollingSvc_) {
                stm32Link_->setEventCallback([svc = pollingSvc_.get()](
                                                 const hw::stm32::Frame& frame) {
                    if (svc) {
                        svc->handleEventFrame(frame);
                    }
                });
                pollingSvc_->start();
                if (rfidMonitorSvc_) {
                    rfidMonitorSvc_->start();
                }
            }
        }
    );

    QMetaObject::invokeMethod(
        stateMachine_.get(),
        "startup",
        Qt::QueuedConnection
    );
}

void AppBootstrap::switchToModuleUid(const std::string& uid) {
    if (uid.empty() || uid == activeModuleUid_) {
        return;
    }

    const int newModuleId = ensureModuleForUid(uid);
    if (newModuleId <= 0) {
        return;
    }

    eventLogger_->log("INFO",
                      "RfidModuleSwitch",
                      "uid=" + uid +
                      " from_module_id=" + std::to_string(activeModuleId_) +
                      " to_module_id=" + std::to_string(newModuleId));

    destroyModuleScopedSession(true);
    activeModuleUid_ = uid;
    activeModuleId_ = newModuleId;
    syncModuleStatuses();
    buildModuleScopedSession();
    showMainWindow();
}

void AppBootstrap::launch() {
    showMainWindow();
}
