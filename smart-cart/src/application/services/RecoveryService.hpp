#pragma once

#include "application/ports/IOperationRepository.hpp"
#include "application/ports/IReelRepository.hpp"
#include "application/ports/IStm32Link.hpp"

#include <vector>

namespace smartcart::application::services {

struct RecoveryConfig {
    int moduleId  = 1;
    int slotCount = 24;
    std::vector<int> slotToLedMap;
};

/// Сервис восстановления незавершённых операций после перезапуска
class RecoveryService {
public:
    RecoveryService(
        ports::IOperationRepository& opRepo,
        ports::IReelRepository&      reelRepo,
        ports::IStm32Link&           link,
        RecoveryConfig               config
    );

    /// Выполнить восстановление. Возвращает true при успехе.
    bool run();

private:
    ports::IOperationRepository& opRepo_;
    ports::IReelRepository&      reelRepo_;
    ports::IStm32Link&           link_;
    RecoveryConfig               config_;

    std::vector<bool> getPhysicalSnapshot();
    void recoverOperation(const domain::Operation&  op,
                          const std::vector<bool>&  physical);
};

} // namespace smartcart::application::services
