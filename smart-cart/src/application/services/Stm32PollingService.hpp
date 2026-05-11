#pragma once

#include "application/ports/IEventLogger.hpp"
#include "application/ports/IOrderRepository.hpp"
#include "application/ports/IOperationRepository.hpp"
#include "application/ports/IReelRepository.hpp"
#include "application/ports/IStm32Link.hpp"
#include "application/ports/IWorkflowRepository.hpp"
#include "application/services/WorkflowService.hpp"
#include "application/services/SwitchEventHandler.hpp"
#include "application/services/CartLightingService.hpp"
#include "domain/errors/ErrorCode.hpp"

#include <atomic>
#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_set>
#include <vector>

namespace smartcart::application::services {

struct Stm32PollingConfig {
    int moduleId  = 1;
    int slotCount = 24;
    int pollMs    = 500;
    int debounceMs = 50;
    int snapshotFallbackMs = 0;

    // На текущей плате реально подключены PA1/PA2.
    std::vector<int> trackedChannels {1, 3};
    std::vector<int> ignoredChannels {11};
    std::vector<int> channelToSlotMap;
};

struct BarcodeScanResult {
    bool success{false};
    int operationId{0};
    int targetSlot{0};
    domain::ErrorCode error{domain::ErrorCode::None};
    std::string message;

    explicit operator bool() const noexcept { return success; }
};

struct Stm32ConnectionStatus {
    bool uartOpen{false};
    bool pollingRunning{false};
    std::vector<int> activeChannels;
    std::string lastEvent{"нет событий"};
    std::string lastEventAt;
    std::string lastSnapshot{"нет snapshot"};
    std::string lastSnapshotAt;

    // Hardware health
    bool online{false};
    std::chrono::system_clock::time_point lastSeen{};
    std::string lastError{};
};

class Stm32PollingService {
public:
    Stm32PollingService(
        ports::IStm32Link&      link,
        ports::IReelRepository& reelRepo,
        ports::IOperationRepository& opRepo,
        ports::IOrderRepository& orderRepo,
        ports::IWorkflowRepository& workflowRepo,
        WorkflowService& workflowSvc,
        ports::IEventLogger&    eventLogger,
        Stm32PollingConfig      config
    );
    ~Stm32PollingService();

    Stm32PollingService(const Stm32PollingService&) = delete;
    Stm32PollingService& operator=(const Stm32PollingService&) = delete;

    void start();
    void stop();
    void pollOnce();

    /// Установить сервис RGB-подсветки. Будет вызываться из pollLoop().
    /// Вызывать до start(). Nullable — если nullptr, подсветка не обновляется.
    void setCartLighting(CartLightingService* svc) noexcept { cartLighting_ = svc; }
    bool isRunning() const noexcept { return running_.load(); }
    void handleEventFrame(const smartcart::infrastructure::hw::stm32::Frame& frame);
    Stm32ConnectionStatus connectionStatus() const;

    BarcodeScanResult recordBarcodeScan(const std::string& barcode);

private:
    struct PendingScan {
        std::string barcode;
        int operationId = 0;
        int orderItemId = 0;
        int targetSlot = 0;
    };

    ports::IStm32Link&      link_;
    ports::IReelRepository& reelRepo_;
    ports::IOperationRepository& opRepo_;
    ports::IOrderRepository& orderRepo_;
    ports::IWorkflowRepository& workflowRepo_;
    WorkflowService& workflowSvc_;
    ports::IEventLogger&    eventLogger_;
    Stm32PollingConfig      config_;

    CartLightingService* cartLighting_{nullptr}; // nullable — опционально
    SwitchEventHandler switchEventHandler_;

    // Кэшированные множества каналов для O(1) проверок в hot path
    // (applySnapshot вызывается ~2 раза в секунду на каждый канал).
    // Заполняются в конструкторе из векторов конфигурации.
    std::unordered_set<int> trackedChannelsSet_;
    std::unordered_set<int> ignoredChannelsSet_;

    std::atomic<bool> running_ {false};
    std::thread       thread_;

    std::optional<std::vector<bool>> lastSnapshot_;
    std::optional<std::vector<bool>> rawSnapshot_;
    std::vector<std::chrono::steady_clock::time_point> rawChangedAt_;
    std::mutex stateMtx_;
    mutable std::mutex statusMtx_;
    Stm32ConnectionStatus status_;
    std::mutex pendingMtx_;
    std::optional<PendingScan> pendingScan_;

    void pollLoop();
    std::optional<std::vector<bool>> requestSnapshot();
    void applySnapshot(const std::vector<bool>& snapshot);
    void applyChannelStateLocked(int channel,
                                 bool occupied,
                                 std::string_view source);
    void handleOccupiedSlot(int channel, int slotIndex);
    void handleFreedSlot(int channel, int slotIndex);
    std::optional<PendingScan> consumePendingScan();
    BarcodeScanResult rejectBarcode(domain::ErrorCode code,
                                    std::string message,
                                    std::string_view logCode) const;

    bool isIgnoredChannel(int channel) const;
    bool isTrackedChannel(int channel) const;
    std::optional<int> slotIndexForChannel(int channel) const;
    void updateLastEvent(std::string message);
    void updateLastSnapshot(std::string message);
    void updateActiveChannels(const std::vector<bool>& snapshot);
    void logSafe(std::string_view level,
                 std::string_view code,
                 std::string_view message) const;
};

} // namespace smartcart::application::services
