#include "FrameCodec.hpp"

#include <stdexcept>

namespace smartcart::infrastructure::hw::stm32 {

namespace {
constexpr size_t kHeaderNoSofSize = 6; // ver,type,seq,cmd,len_lo,len_hi
constexpr size_t kMinFrameSize = 2 + kHeaderNoSofSize + 2; // SOF + header + CRC

inline void appendLe16(std::vector<uint8_t>& out, uint16_t v) {
    out.push_back(static_cast<uint8_t>(v & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
}
} // namespace

uint16_t FrameCodec::crc16CcittFalse(const uint8_t* data, size_t size) noexcept {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < size; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8U;
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x8000U) crc = static_cast<uint16_t>((crc << 1U) ^ 0x1021U);
            else crc <<= 1U;
        }
    }
    return crc;
}

std::vector<uint8_t> FrameCodec::encode(const Frame& frame) {
    if (frame.payload.size() > kMaxPayloadV1) {
        throw std::runtime_error("payload exceeds kMaxPayloadV1");
    }

    std::vector<uint8_t> out;
    out.reserve(2 + kHeaderNoSofSize + frame.payload.size() + 2);

    out.push_back(kSof1);
    out.push_back(kSof2);

    out.push_back(frame.protocolVersion);
    out.push_back(static_cast<uint8_t>(frame.frameType));
    out.push_back(frame.seq);
    out.push_back(frame.commandId);

    appendLe16(out, static_cast<uint16_t>(frame.payload.size()));
    out.insert(out.end(), frame.payload.begin(), frame.payload.end());

    const uint16_t crc = crc16CcittFalse(out.data() + 2, out.size() - 2);
    appendLe16(out, crc);
    return out;
}

std::optional<Frame> FrameCodec::decode(const uint8_t* data, size_t size) {
    if (data == nullptr || size < kMinFrameSize) return std::nullopt;
    if (data[0] != kSof1 || data[1] != kSof2) return std::nullopt;

    const uint16_t payload_len = static_cast<uint16_t>(data[6]) |
                                 (static_cast<uint16_t>(data[7]) << 8U);
    if (payload_len > kMaxPayloadV1) return std::nullopt;

    const size_t expected = 2 + kHeaderNoSofSize + payload_len + 2;
    if (size != expected) return std::nullopt;

    const uint16_t actual_crc =
        static_cast<uint16_t>(data[size - 2]) |
        (static_cast<uint16_t>(data[size - 1]) << 8U);

    const uint16_t calc_crc = crc16CcittFalse(data + 2, size - 4);
    if (actual_crc != calc_crc) return std::nullopt;

    Frame f;
    f.protocolVersion = data[2];
    f.frameType = static_cast<FrameType>(data[3]);
    f.seq = data[4];
    f.commandId = data[5];
    f.payload.assign(data + 8, data + 8 + payload_len);
    return f;
}

// ---------------- StreamParser ----------------

FrameCodec::StreamParser::StreamParser() {
    reset();
}

void FrameCodec::StreamParser::reset() noexcept {
    state_ = State::WaitSof1;
    header_pos_ = 0;
    payload_len_ = 0;
    payload_.clear();
    crc_data_.clear();
    crc_lo_ = 0;
}

void FrameCodec::StreamParser::restartToSofSearch() noexcept {
    state_ = State::WaitSof1;
    header_pos_ = 0;
    payload_len_ = 0;
    payload_.clear();
    crc_data_.clear();
    crc_lo_ = 0;
}

FrameCodec::ParseEvent FrameCodec::StreamParser::feed(uint8_t byte) {
    ParseEvent ev{};

    switch (state_) {
    case State::WaitSof1:
        if (byte == kSof1) state_ = State::WaitSof2;
        return ev;

    case State::WaitSof2:
        if (byte == kSof2) {
            state_ = State::Header;
            header_pos_ = 0;
            crc_data_.clear();
        } else if (byte != kSof1) {
            state_ = State::WaitSof1;
        }
        return ev;

    case State::Header:
        header_[header_pos_++] = byte;
        crc_data_.push_back(byte);
        if (header_pos_ == kHeaderNoSofSize) {
            payload_len_ = static_cast<uint16_t>(header_[4]) |
                           (static_cast<uint16_t>(header_[5]) << 8U);
            if (payload_len_ > kMaxPayloadV1) {
                ++parse_error_count_;
                restartToSofSearch();
                ev.type = ParseEventType::LengthError;
                return ev;
            }
            payload_.clear();
            payload_.reserve(payload_len_);
            state_ = (payload_len_ == 0) ? State::CrcLo : State::Payload;
        }
        return ev;

    case State::Payload:
        payload_.push_back(byte);
        crc_data_.push_back(byte);
        if (payload_.size() == payload_len_) state_ = State::CrcLo;
        return ev;

    case State::CrcLo:
        crc_lo_ = byte;
        state_ = State::CrcHi;
        return ev;

    case State::CrcHi: {
        const uint16_t rx_crc = static_cast<uint16_t>(crc_lo_) |
                                (static_cast<uint16_t>(byte) << 8U);
        const uint16_t calc_crc = FrameCodec::crc16CcittFalse(crc_data_.data(), crc_data_.size());
        if (rx_crc != calc_crc) {
            ++crc_drop_count_;
            restartToSofSearch();
            ev.type = ParseEventType::CrcError;
            return ev;
        }

        Frame f;
        f.protocolVersion = header_[0];
        f.frameType = static_cast<FrameType>(header_[1]);
        f.seq = header_[2];
        f.commandId = header_[3];
        f.payload = payload_;

        restartToSofSearch();
        ev.type = ParseEventType::FrameReady;
        ev.frame = std::move(f);
        return ev;
    }
    }

    return ev;
}

} // namespace smartcart::infrastructure::hw::stm32
