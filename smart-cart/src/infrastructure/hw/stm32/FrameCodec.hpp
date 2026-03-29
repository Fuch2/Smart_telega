#pragma once

#include "Protocol.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace smartcart::infrastructure::hw::stm32 {

class FrameCodec final {
public:
    static uint16_t crc16CcittFalse(const uint8_t* data, size_t size) noexcept;

    // Serialize full frame with SOF + CRC.
    static std::vector<uint8_t> encode(const Frame& frame);

    // Decode one complete frame buffer.
    // Returns std::nullopt if frame invalid (SOF/length/CRC).
    static std::optional<Frame> decode(const uint8_t* data, size_t size);

    // --- Streaming parser (suitable for UART byte stream / STM32 style feed) ---
    enum class ParseEventType : uint8_t {
        None,
        FrameReady,
        CrcError,
        LengthError
    };

    struct ParseEvent {
        ParseEventType type {ParseEventType::None};
        std::optional<Frame> frame {};
    };

    class StreamParser final {
    public:
        StreamParser();
        void reset() noexcept;

        // Feed one byte at a time.
        ParseEvent feed(uint8_t byte);

        uint32_t crcDropCount() const noexcept { return crc_drop_count_; }
        uint32_t parseErrorCount() const noexcept { return parse_error_count_; }

    private:
        enum class State : uint8_t {
            WaitSof1,
            WaitSof2,
            Header,   // version,type,seq,cmd,len_lo,len_hi
            Payload,
            CrcLo,
            CrcHi
        };

        State state_ {State::WaitSof1};
        uint8_t header_[6] {};
        size_t header_pos_ {0};

        uint16_t payload_len_ {0};
        std::vector<uint8_t> payload_;

        uint8_t crc_lo_ {0};
        std::vector<uint8_t> crc_data_; // [ver..payload]
        uint32_t crc_drop_count_ {0};
        uint32_t parse_error_count_ {0};

        void restartToSofSearch() noexcept;
    };
};

} // namespace smartcart::infrastructure::hw::stm32
