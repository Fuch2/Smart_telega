// ===== src/application/services/AddReelService.hpp =====
#pragma once

#include "application/ports/IStm32Link.hpp"
#include "application/ports/IReelRepository.hpp"
#include "application/ports/IOperationRepository.hpp"
#include "application/services/RgbColor.hpp"
#include "domain/entities/Slot.hpp"
#include "domain/entities/Operation.hpp"
#include "domain/errors/ErrorCode.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace smartcart::application::services {

struct AddReelConfig {
    int moduleId        = 1;
    int slotCount       = 24;
    int stableConfirmMs = 1000;
    std::vector<int> slotToLedMap;
};

class AddReelService {
public:
    using CompletionCallback    = std::function<void(int opId,
                                                     domain::OperationStatus)>;
    using SlotHighlightCallback = std::function<void(int slotIndex,
                                                     RgbColor color)>;
    using ErrorCallback         = std::function<void(domain::ErrorCode,
                                                     std::string)>;

    AddReelService(
        ports::IStm32Link&           link,
        ports::IReelRepository&      reelRepo,
        ports::IOperationRepository& opRepo,
        AddReelConfig                config
    );

    void setCompletionCallback(CompletionCallback cb)       { onComplete_ = std::move(cb); }
    void setSlotHighlightCallback(SlotHighlightCallback cb) { onHighlight_ = std::move(cb); }
    void setErrorCallback(ErrorCallback cb)                 { onError_    = std::move(cb); }

    int  start(const std::string& barcode);
    void cancel();

private:
    ports::IStm32Link&           link_;
    ports::IReelRepository&      reelRepo_;
    ports::IOperationRepository& opRepo_;
    AddReelConfig                config_;

    CompletionCallback    onComplete_;
    SlotHighlightCallback onHighlight_;
    ErrorCallback         onError_;

    std::atomic<bool> cancelled_   = false;
    int               currentOpId_ = -1;
    int               currentSlot_ = -1;

    int  findFreeSlot();
    void setSlotLed(int slotIndex, uint8_t r, uint8_t g, uint8_t b);
    void clearAllLeds();
    static bool isValidBarcode(const std::string& barcode);
};

} // namespace smartcart::application::services
