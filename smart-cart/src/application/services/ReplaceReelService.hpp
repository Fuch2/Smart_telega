// ===== src/application/services/StartupService.hpp =====
#pragma once

#include "application/ports/IStm32Link.hpp"
#include "application/ports/IModuleRepository.hpp"
#include "application/ports/IReelRepository.hpp"
#include "domain/entities/Slot.hpp"
#include "domain/errors/ErrorCode.hpp"

#include <vector>
#include <variant>

namespace smartcart::application::services {

struct StartupConfig {
    int moduleId         = 1;
    int slotCount        = 24;
    int readyTimeoutMs   = 5000;
    std::vector<int> slotToLedMap;
};

using StartupResult = std::variant<std::vector<domain::Slot>, domain::ErrorCode>;

class StartupService {
public:
    StartupService(
        ports::IStm32Link&         link,
        ports::IReelRepository&    reelRepo,
        ports::IModuleRepository&  moduleRepo,
        StartupConfig              config
    );

    StartupResult run();

private:
    ports::IStm32Link&        link_;
    ports::IReelRepository&   reelRepo_;
    ports::IModuleRepository& moduleRepo_;
    StartupConfig             config_;

    /// Создать модуль в БД если не существует.
    void ensureModuleExists();

    /// Инициализировать все слоты как FREE если их неет в slot_states.
    void ensureSlotsInitialized();

    bool              ping();
    bool              waitReady();
    std::vector<bool> getSnapshot();
    std::vector<domain::Slot> reconcile(const std::vector<bool>& physical);
};

} // namespace smartcart::application::services
