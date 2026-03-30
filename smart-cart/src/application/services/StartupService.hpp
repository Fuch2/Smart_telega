#pragma once

#include "application/ports/IStm32Link.hpp"
#include "application/ports/IModuleRepository.hpp"
#include "application/ports/IReelRepository.hpp"
#include "domain/entities/Slot.hpp"
#include "domain/errors/ErrorCode.hpp"

#include <vector>
#include <variant>

namespace smartcart::application::services {

/// Конфигурация, необходимая сервису запуска.
struct StartupConfig {
    int moduleId         = 1;       // ID модуля в БД
    int slotCount        = 24;      // количество слотов
    int readyTimeoutMs   = 5000;    // таймаут ожидания EvtReady
    std::vector<int> slotToLedMap; // slot_index (0-based) → led_index
};

/// Результат запуска: либо вектор слотов, либо код ошибки.
using StartupResult = std::variant<std::vector<domain::Slot>, domain::ErrorCode>;

/// Сервис инициализации: пингует STM32, синхронизирует снимок слотов с БД.
class StartupService {
public:
    StartupService(
        ports::IStm32Link&         link,
        ports::IReelRepository&    reelRepo,
        ports::IModuleRepository&  moduleRepo,
        StartupConfig              config
    );

    /// Выполнить полную последовательность запуска.
    /// Блокирует вызывающий поток до завершения или ошибки.
    StartupResult run();

private:
    ports::IStm32Link&        link_;
    ports::IReelRepository&   reelRepo_;
    ports::IModuleRepository& moduleRepo_;
    StartupConfig             config_;

    /// Шаг 1: Ping → проверить связь.
    bool ping();

    /// Шаг 2: GetReadyState → при необходимости ждать EvtReady.
    bool waitReady();

    /// Шаг 3: GetSwitchSnapshot → битовая маска занятых слотов.
    /// Возвращает вектор bool[slotCount], true = занят.
    std::vector<bool> getSnapshot();

    /// Шаг 4: сверить snapshot с БД, обновить состояния слотов.
    std::vector<domain::Slot> reconcile(const std::vector<bool>& physical);
};

} // namespace smartcart::application::services
