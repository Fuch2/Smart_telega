// ===== src/application/services/ReplaceReelService.hpp =====
#pragma once

#include "application/ports/IStm32Link.hpp"
#include "application/ports/IReelRepository.hpp"
#include "application/ports/IOperationRepository.hpp"
#include "application/services/RgbColor.hpp"
#include "domain/entities/Operation.hpp"
#include "domain/errors/ErrorCode.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace smartcart::application::services {

struct ReplaceReelConfig {
    int              moduleId        = 1;
    int              stableConfirmMs = 5000;
    std::vector<int> slotToLedMap;
};

class ReplaceReelService {
public:
    using CompletionCallback    = std::function<void(int opId,
                                                     domain::OperationStatus)>;
    using SlotHighlightCallback = std::function<void(int slotIndex,
                                                     RgbColor color)>;
    using ErrorCallback         = std::function<void(domain::ErrorCode,
                                                     std::string)>;

    ReplaceReelService(
        ports::IStm32Link&           link,
        ports::IReelRepository&      reelRepo,
        ports::IOperationRepository& opRepo,
        ReplaceReelConfig            config
    );

    int  start(const std::string& newBarcode);
    void cancel();

    void setCompletionCallback(CompletionCallback cb)       { onComplete_ = std::move(cb); }
    void setSlotHighlightCallback(SlotHighlightCallback cb) { onHighlight_ = std::move(cb); }
    void setErrorCallback(ErrorCallback cb)                 { onError_     = std::move(cb); }

private:
    ports::IStm32Link&           link_;
    ports::IReelRepository&      reelRepo_;
    ports::IOperationRepository& opRepo_;
    ReplaceReelConfig            config_;

    std::atomic<bool> cancelled_{false};

    CompletionCallback    onComplete_;
    SlotHighlightCallback onHighlight_;
    ErrorCallback         onError_;

    static bool isValidBarcode(const std::string& barcode);
    void        setSlotLed(int slotIndex, uint8_t r, uint8_t g, uint8_t b);
    void        clearAllLeds();
    bool        waitForSlotEvent(int slotIndex, bool expectedOccupied);
};

} // namespace smartcart::application::services
