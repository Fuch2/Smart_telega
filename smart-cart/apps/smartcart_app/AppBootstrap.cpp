// ===== apps/smartcart_app/AppBootstrap.cpp =====
#include "AppBootstrap.hpp"

#include "infrastructure/config/ConfigLoader.hpp"
#include "infrastructure/hw/rfid/MultiRc522RfidProvider.hpp"
#include "infrastructure/hw/rfid/Rc522RfidProvider.hpp"
#include "infrastructure/hw/stm32/MockStm32Link.hpp"
#include "presentation/qt/MainWindow.hpp"
#include "presentation/qt/viewmodels/AdminViewModel.hpp"
#include "presentation/qt/viewmodels/WorkerViewModel.hpp"

#include <QApplication>
#include <QInputDialog>
#include <QMessageBox>
#include <QMetaObject>
#include <QObject>
#include <QStringList>
#include <algorithm>
#include <fstream>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

using namespace smartcart;
using namespace smartcart::infrastructure;
using namespace smartcart::application;
using namespace smartcart::application::services;

namespace {

const config::ModuleChannelConfig* moduleChannelForUid(
    const config::AppConfig& cfg,
    const std::string& uid) {
    if (uid.empty()) {
        return nullptr;
    }

    const auto it = std::find_if(
        cfg.moduleChannels.begin(),
        cfg.moduleChannels.end(),
        [&uid](const config::ModuleChannelConfig& channel) {
            return channel.enabled && channel.rfidUid == uid;
        });

    return it == cfg.moduleChannels.end() ? nullptr : &(*it);
}

domain::ModuleKind moduleKindForUid(const config::AppConfig& cfg,
                                    const std::string& uid) {
    if (const auto* channel = moduleChannelForUid(cfg, uid);
        channel != nullptr)
    {
        return domain::moduleKindFromString(channel->kind);
    }

    if (const auto it = cfg.rfidModuleKinds.find(uid);
        it != cfg.rfidModuleKinds.end())
    {
        return domain::moduleKindFromString(it->second);
    }

    const std::string serialKey = "RFID-" + uid;
    if (const auto it = cfg.rfidModuleKinds.find(serialKey);
        it != cfg.rfidModuleKinds.end())
    {
        return domain::moduleKindFromString(it->second);
    }

    return domain::ModuleKind::Unknown;
}

} // namespace

struct AppBootstrap::ModuleChannelRuntime {
    int moduleId{0};
    std::string uid;
    std::string device;
    std::unique_ptr<application::ports::IStm32Link> link;
    std::unique_ptr<ModuleSnapshotPollingService> polling;
};

AppBootstrap::AppBootstrap(const std::filesystem::path& configPath,
                           const std::filesystem::path& migrationsDir)
{
    // ── 1. Config ─────────────────────────────────────────────────────────────
    configPath_ = configPath;
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
        if (cfg_.rfidSpiDevices.size() > 1) {
            rfidProvider_ = std::make_unique<hw::rfid::MultiRc522RfidProvider>(
                cfg_.rfidSpiDevices
            );
        } else {
            const auto device = cfg_.rfidSpiDevices.empty()
                ? cfg_.rfidSpiDevice
                : cfg_.rfidSpiDevices.front();
            rfidProvider_ = std::make_unique<hw::rfid::Rc522RfidProvider>(device);
        }
    }

    activeModuleId_ = resolveActiveModuleId();
    syncModuleStatuses();

    // ── 3. LED map ────────────────────────────────────────────────────────────
    buildSlotToLedMap();

    // ── 4. STM32 link ─────────────────────────────────────────────────────────
    buildActiveStm32Link();
    buildModuleScopedSession();
    buildModuleChannelRuntimes();
}

AppBootstrap::~AppBootstrap() {
    destroyModuleChannelRuntimes();
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
        if (!isRfidUidConfigured(*uid) &&
            !ensureRfidModuleRole(*uid).has_value())
        {
            eventLogger_->log("WARN",
                              "RfidModuleRegistrationSkipped",
                              "uid=" + *uid);
            activeModuleUid_.clear();
            return 1;
        }

        const std::string serial = "RFID-" + *uid;

        for (const auto& module : moduleRepo_->getAll()) {
            if (module.serial == serial) {
                const auto configuredKind = moduleKindForUid(cfg_, *uid);
                if (module.kind != configuredKind) {
                    auto updated = module;
                    updated.kind = configuredKind;
                    moduleRepo_->update(updated);
                }
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
        module.kind = moduleKindForUid(cfg_, *uid);

        const int moduleId = moduleRepo_->add(module);
        if (moduleId > 0) {
            eventLogger_->log("INFO",
                              "RfidModuleRegistered",
                              "uid=" + *uid +
                              " module_id=" + std::to_string(moduleId) +
                              " kind=" + std::string{domain::toString(module.kind)});
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

    if (!cfg_.moduleChannels.empty()) {
        if (auto module = moduleRepo_->getById(activeModuleId_);
            module.has_value())
        {
            const auto targetStatus = hasActivePhysicalModule
                ? smartcart::domain::ModuleStatus::Online
                : smartcart::domain::ModuleStatus::Offline;
            if (module->status != targetStatus) {
                module->status = targetStatus;
                moduleRepo_->update(*module);
            }
        }
        return;
    }

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
            const auto configuredKind = moduleKindForUid(cfg_, uid);
            if (module.kind != configuredKind) {
                auto updated = module;
                updated.kind = configuredKind;
                moduleRepo_->update(updated);
            }
            return module.id;
        }
    }

    smartcart::domain::ModuleInfo module;
    module.serial = serial;
    module.slotCount = 24;
    module.firmware = "rfid-auto";
    module.status = smartcart::domain::ModuleStatus::Offline;
    module.kind = moduleKindForUid(cfg_, uid);

    return moduleRepo_->add(module);
}

bool AppBootstrap::isRfidUidConfigured(const std::string& uid) const {
    if (uid.empty()) {
        return false;
    }

    if (moduleChannelForUid(cfg_, uid) != nullptr) {
        return true;
    }

    if (cfg_.rfidModuleKinds.find(uid) != cfg_.rfidModuleKinds.end()) {
        return true;
    }

    const std::string serialKey = "RFID-" + uid;
    return cfg_.rfidModuleKinds.find(serialKey) != cfg_.rfidModuleKinds.end();
}

std::optional<smartcart::domain::ModuleKind>
AppBootstrap::ensureRfidModuleRole(const std::string& uid) {
    if (uid.empty()) {
        return std::nullopt;
    }

    if (isRfidUidConfigured(uid)) {
        return moduleKindForUid(cfg_, uid);
    }

    const QStringList choices{
        QString::fromUtf8("Модуль катушек (REEL)"),
        QString::fromUtf8("Модуль питателей (FEEDER)"),
        QString::fromUtf8("Неизвестный модуль (UNKNOWN)")
    };

    bool ok = false;
    const QString choice = QInputDialog::getItem(
        mainWindow_,
        QString::fromUtf8("Новая RFID-метка"),
        QString::fromUtf8(
            "Обнаружена новая RFID-метка:\n%1\n\n"
            "Выберите тип модуля. Выбор будет сохранён в config.json.")
            .arg(QString::fromStdString(uid)),
        choices,
        0,
        false,
        &ok
    );

    if (!ok || choice.isEmpty()) {
        eventLogger_->log("WARN",
                          "RfidModuleRegistrationCancelled",
                          "uid=" + uid);
        return std::nullopt;
    }

    smartcart::domain::ModuleKind kind = smartcart::domain::ModuleKind::Unknown;
    if (choice.contains(QStringLiteral("REEL"))) {
        kind = smartcart::domain::ModuleKind::Reel;
    } else if (choice.contains(QStringLiteral("FEEDER"))) {
        kind = smartcart::domain::ModuleKind::Feeder;
    }

    if (!persistRfidModuleRole(uid, kind)) {
        QMessageBox::warning(
            mainWindow_,
            QString::fromUtf8("RFID-модуль не сохранён"),
            QString::fromUtf8(
                "Не удалось записать новую RFID-метку в config.json.\n"
                "Проверьте права на файл конфигурации.")
        );
        return std::nullopt;
    }

    return kind;
}

bool AppBootstrap::persistRfidModuleRole(
    const std::string& uid,
    smartcart::domain::ModuleKind kind)
{
    try {
        nlohmann::json root;
        {
            std::ifstream in(configPath_);
            if (!in.is_open()) {
                throw std::runtime_error(
                    "cannot open config: " + configPath_.string());
            }
            in >> root;
        }

        if (!root.contains("rfid_module_roles") ||
            !root["rfid_module_roles"].is_object())
        {
            root["rfid_module_roles"] = nlohmann::json::object();
        }

        const std::string kindText{smartcart::domain::toString(kind)};
        root["rfid_module_roles"][uid] = kindText;

        {
            std::ofstream out(configPath_, std::ios::trunc);
            if (!out.is_open()) {
                throw std::runtime_error(
                    "cannot write config: " + configPath_.string());
            }
            out << root.dump(2) << '\n';
        }

        cfg_.rfidModuleKinds[uid] = kindText;
        eventLogger_->log("INFO",
                          "RfidModuleRoleSaved",
                          "uid=" + uid + " kind=" + kindText +
                              " config=" + configPath_.string());
        return true;
    } catch (const std::exception& ex) {
        eventLogger_->log("ERROR",
                          "RfidModuleRoleSaveFailed",
                          "uid=" + uid + " error=" + ex.what());
    } catch (...) {
        eventLogger_->log("ERROR",
                          "RfidModuleRoleSaveFailed",
                          "uid=" + uid + " error=unknown");
    }
    return false;
}

void AppBootstrap::buildActiveStm32Link() {
    mockLink_ = nullptr;
    activeStm32Device_.clear();

    if (cfg_.demoMode) {
        auto mock = std::make_unique<hw::stm32::MockStm32Link>(
            [](const hw::stm32::Frame& cmd) -> std::optional<hw::stm32::Frame> {
                hw::stm32::Frame resp;
                resp.seq = cmd.seq;
                resp.cmdId = cmd.cmdId;

                if (cmd.cmdId == hw::stm32::CommandId::GetSwitchSnapshot) {
                    resp.type = hw::stm32::FrameType::Resp;
                    resp.payload = {0x00, 0x00, 0x00};
                } else if (cmd.cmdId == hw::stm32::CommandId::GetReadyState) {
                    resp.type = hw::stm32::FrameType::Resp;
                    resp.payload = {0x01};
                } else {
                    resp.type = hw::stm32::FrameType::Ack;
                }
                return resp;
            }
        );
        mockLink_ = mock.get();
        stm32Link_ = std::move(mock);
        activeStm32Device_ = "mock";
        stm32Link_->open();
        return;
    }

    std::string device = cfg_.stm32Device;
    if (const auto* channel = moduleChannelForUid(cfg_, activeModuleUid_);
        channel != nullptr &&
        !channel->stm32Device.empty())
    {
        device = channel->stm32Device;
    }

    auto uart = std::make_unique<hw::stm32::UartStm32Link>(
        device, 115200, 500
    );
    if (!uart->open()) {
        throw std::runtime_error(
            "AppBootstrap: failed to open UART " + device);
    }

    activeStm32Device_ = device;
    stm32Link_ = std::move(uart);
}

void AppBootstrap::buildModuleChannelRuntimes() {
    destroyModuleChannelRuntimes();
    if (cfg_.demoMode || cfg_.moduleChannels.empty()) {
        return;
    }

    for (const auto& channel : cfg_.moduleChannels) {
        if (!channel.enabled ||
            channel.stm32Device.empty() ||
            channel.stm32Device == activeStm32Device_ ||
            channel.rfidUid == activeModuleUid_)
        {
            continue;
        }

        const int moduleId = ensureModuleForUid(channel.rfidUid);
        if (moduleId <= 0) {
            eventLogger_->log("ERROR",
                              "ModuleChannelInitFailed",
                              "Не удалось создать модуль для uid=" +
                                  channel.rfidUid);
            continue;
        }

        auto link = std::make_unique<hw::stm32::UartStm32Link>(
            channel.stm32Device, 115200, 500
        );
        if (!link->open()) {
            if (auto module = moduleRepo_->getById(moduleId); module.has_value()) {
                module->status = smartcart::domain::ModuleStatus::Offline;
                moduleRepo_->update(*module);
            }
            eventLogger_->log("ERROR",
                              "ModuleStm32OpenFailed",
                              "module_id=" + std::to_string(moduleId) +
                                  " uid=" + channel.rfidUid +
                                  " device=" + channel.stm32Device);
            continue;
        }

        ModuleSnapshotPollingConfig pollingCfg;
        pollingCfg.moduleId = moduleId;
        pollingCfg.slotCount = channel.slotCount;
        pollingCfg.pollMs = static_cast<int>(cfg_.stm32PollMs);
        pollingCfg.debounceMs = static_cast<int>(cfg_.debounceMs);
        pollingCfg.channelName = channel.name;
        pollingCfg.stm32Device = channel.stm32Device;
        pollingCfg.trackedChannels = channel.trackedChannels;
        pollingCfg.ignoredChannels = channel.ignoredChannels;
        pollingCfg.channelToSlotMap = channel.channelToSlotMap;

        auto polling = std::make_unique<ModuleSnapshotPollingService>(
            *link,
            *moduleRepo_,
            *reelRepo_,
            *eventLogger_,
            std::move(pollingCfg));

        link->setEventCallback(
            [svc = polling.get()](const hw::stm32::Frame& frame) {
                if (svc) {
                    svc->handleEventFrame(frame);
                }
            });

        auto runtime = std::make_unique<ModuleChannelRuntime>();
        runtime->moduleId = moduleId;
        runtime->uid = channel.rfidUid;
        runtime->device = channel.stm32Device;
        runtime->link = std::move(link);
        runtime->polling = std::move(polling);
        moduleChannelRuntimes_.push_back(std::move(runtime));
    }
}

void AppBootstrap::startModuleChannelRuntimes() {
    for (const auto& runtime : moduleChannelRuntimes_) {
        if (runtime && runtime->polling) {
            runtime->polling->start();
        }
    }
}

void AppBootstrap::destroyModuleChannelRuntimes() {
    for (const auto& runtime : moduleChannelRuntimes_) {
        if (!runtime) {
            continue;
        }
        if (runtime->polling) {
            runtime->polling->stop();
        }
        if (runtime->link) {
            runtime->link->setEventCallback({});
            runtime->link->close();
        }
    }
    moduleChannelRuntimes_.clear();
}

void AppBootstrap::buildModuleScopedSession() {
    if (cfg_.rfidEnabled && activeModuleUid_.empty()) {
        if (const auto module = moduleRepo_->getById(activeModuleId_);
            module.has_value() &&
            module->status != smartcart::domain::ModuleStatus::Offline)
        {
            auto updated = *module;
            updated.status = smartcart::domain::ModuleStatus::Offline;
            moduleRepo_->update(updated);
        }
    }

    orderRepo_ = std::make_unique<db::OrderRepositorySqlite>(*conn_, activeModuleId_);
    workflowRepo_ = std::make_unique<db::WorkflowRepositorySqlite>(*conn_, activeModuleId_);

    const auto* activeChannel = moduleChannelForUid(cfg_, activeModuleUid_);
    const int activeSlotCount = activeChannel != nullptr
        ? activeChannel->slotCount
        : 24;
    const auto& activeTrackedChannels = activeChannel != nullptr
        ? activeChannel->trackedChannels
        : cfg_.switchTrackedChannels;
    const auto& activeIgnoredChannels = activeChannel != nullptr
        ? activeChannel->ignoredChannels
        : cfg_.switchIgnoredChannels;
    const auto& activeChannelToSlotMap = activeChannel != nullptr
        ? activeChannel->channelToSlotMap
        : cfg_.switchChannelToSlotMap;

    StartupConfig startupCfg;
    startupCfg.moduleId = activeModuleId_;
    startupCfg.slotCount = activeSlotCount;
    startupCfg.readyTimeoutMs = 5000;
    startupCfg.slotToLedMap = slotToLedMap_;
    startupCfg.ignoredChannels = activeIgnoredChannels;
    startupCfg.channelToSlotMap = activeChannelToSlotMap;
    startupSvc_ = std::make_unique<StartupService>(
        *stm32Link_, *reelRepo_, *moduleRepo_, startupCfg);

    AddReelConfig addCfg;
    addCfg.moduleId = activeModuleId_;
    addCfg.slotCount = activeSlotCount;
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
    recoveryCfg.slotCount = activeSlotCount;
    recoveryCfg.slotToLedMap = slotToLedMap_;
    recoverySvc_ = std::make_unique<RecoveryService>(
        *opRepo_, *reelRepo_, *stm32Link_, recoveryCfg);

    workflowSvc_ = std::make_unique<WorkflowService>(
        *orderRepo_, *workflowRepo_, *reelRepo_, *eventLogger_, activeModuleId_);

    Stm32PollingConfig pollingCfg;
    pollingCfg.moduleId = activeModuleId_;
    pollingCfg.slotCount = activeSlotCount;
    pollingCfg.pollMs = static_cast<int>(cfg_.stm32PollMs);
    pollingCfg.debounceMs = static_cast<int>(cfg_.debounceMs);
    pollingCfg.snapshotFallbackMs =
        static_cast<int>(cfg_.stableConfirmMs + cfg_.stm32PollMs);
    pollingCfg.trackedChannels = activeTrackedChannels;
    pollingCfg.ignoredChannels = activeIgnoredChannels;
    pollingCfg.channelToSlotMap = activeChannelToSlotMap;
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
    bomOrderImportSvc_ =
        std::make_unique<BomOrderImportService>(*orderImportSvc_);

    AdminDiagnosticsConfig adminDiagnosticsCfg;
    adminDiagnosticsCfg.moduleId = activeModuleId_;
    adminDiagnosticsCfg.trackedChannels = activeTrackedChannels;
    adminDiagnosticsCfg.ignoredChannels = activeIgnoredChannels;
    adminDiagnosticsCfg.channelToSlotMap = activeChannelToSlotMap;
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
        *bomOrderImportSvc_,
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
    bomOrderImportSvc_.reset();
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
                startModuleChannelRuntimes();
            }
        }
    );

    // RFID должен работать даже на экране "Вставьте модуль".
    // Иначе приложение, запущенное без метки рядом со считывателем,
    // не сможет увидеть модуль и выйти из блокирующего экрана.
    if (rfidMonitorSvc_) {
        rfidMonitorSvc_->start();
    }

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

    if (!isRfidUidConfigured(uid) &&
        !ensureRfidModuleRole(uid).has_value())
    {
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
    destroyModuleChannelRuntimes();
    stm32Link_.reset();
    activeModuleUid_ = uid;
    activeModuleId_ = newModuleId;
    syncModuleStatuses();
    buildActiveStm32Link();
    buildModuleScopedSession();
    buildModuleChannelRuntimes();
    showMainWindow();
}

void AppBootstrap::launch() {
    showMainWindow();
}
