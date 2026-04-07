#pragma once

#include "application/ports/IEventLogger.hpp"
#include "application/ports/IOrderRepository.hpp"
#include "application/ports/IOperationRepository.hpp"
#include "application/ports/IReelRepository.hpp"
#include "application/ports/IStm32Link.hpp"
#include "application/ports/IWorkflowRepository.hpp"
#include "domain/errors/ErrorCode.hpp"

#include <atomic>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace smartcart::application::services {

struct Stm32PollingConfig {
    int moduleId  = 1;
    int slotCount = 24;
    int pollMs    = 500;

    // На текущей плате реально подключены PA1/PA2.
    std::vector<int> trackedChannels {1, 3};
    std::vector<int> ignoredChannels {11};
};

struct BarcodeScanResult {
    bool success{false};
    int operationId{0};
    int targetSlot{0};
    domain::ErrorCode error{domain::ErrorCode::None};
    std::string message;

    explicit operator bool() const noexcept { return success; }
};

class Stm32PollingService {
public:
    Stm32PollingService(
        ports::IStm32Link&      link,
        ports::IReelRepository& reelRepo,
        ports::IOperationRepository& opRepo,
        ports::IOrderRepository& orderRepo,
        ports::IWorkflowRepository& workflowRepo,
        ports::IEventLogger&    eventLogger,
        Stm32PollingConfig      config
    );
    ~Stm32PollingService();

    Stm32PollingService(const Stm32PollingService&) = delete;
    Stm32PollingService& operator=(const Stm32PollingService&) = delete;

    void start();
    void stop();
    void pollOnce();
    bool isRunning() const noexcept { return running_.load(); }

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
    ports::IEventLogger&    eventLogger_;
    Stm32PollingConfig      config_;

    std::atomic<bool> running_ {false};
    std::thread       thread_;

    std::optional<std::vector<bool>> lastSnapshot_;
    std::mutex pendingMtx_;
    std::optional<PendingScan> pendingScan_;

    void pollLoop();
    std::optional<std::vector<bool>> requestSnapshot();
    void applySnapshot(const std::vector<bool>& snapshot);
    void handleOccupiedSlot(int channel, int slotIndex);
    void handleFreedSlot(int channel, int slotIndex);
    std::optional<PendingScan> consumePendingScan();
    BarcodeScanResult rejectBarcode(domain::ErrorCode code,
                                    std::string message,
                                    std::string_view logCode) const;

    bool isIgnoredChannel(int channel) const;
    bool isTrackedChannel(int channel) const;
    void logSafe(std::string_view level,
                 std::string_view code,
                 std::string_view message) const;
};

} // namespace smartcart::application::services
