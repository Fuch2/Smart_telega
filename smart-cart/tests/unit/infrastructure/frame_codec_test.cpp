// ===== tests/unit/infrastructure/frame_codec_test.cpp =====
// Исправлено: frameType→type, commandId→cmdId, uint8_t→CommandId
#include "infrastructure/hw/stm32/FrameCodec.hpp"
#include "infrastructure/hw/stm32/Protocol.hpp"

#include <gtest/gtest.h>

using namespace smartcart::infrastructure::hw::stm32;

TEST(FrameCodecTest, EncodeDecode_Roundtrip_GetFwVersion) {
    Frame f;
    f.protocolVersion = kProtocolVersionV1;
    f.type   = FrameType::Cmd;
    f.seq    = 0x10;
    f.cmdId  = CommandId::GetFwVersion;
    f.payload = {};

    const auto bytes   = FrameCodec::encode(f);
    const auto decoded = FrameCodec::decode(bytes.data(), bytes.size());

    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->protocolVersion, f.protocolVersion);
    EXPECT_EQ(decoded->type,   f.type);
    EXPECT_EQ(decoded->seq,    f.seq);
    EXPECT_EQ(decoded->cmdId,  f.cmdId);
    EXPECT_EQ(decoded->payload, f.payload);
}

TEST(FrameCodecTest, Decode_Fails_OnCrcError) {
    Frame f;
    f.protocolVersion = kProtocolVersionV1;
    f.type   = FrameType::Cmd;
    f.seq    = 0x22;
    f.cmdId  = CommandId::LedSetSlot;
    f.payload = {0x05, 0xFF, 0x00, 0x00, 0x01};

    auto bytes = FrameCodec::encode(f);
    bytes[8] ^= 0x01; // повредить payload

    EXPECT_FALSE(FrameCodec::decode(bytes.data(), bytes.size()).has_value());
}

TEST(FrameCodecTest, Decode_Fails_OnBadSof) {
    Frame f;
    f.protocolVersion = kProtocolVersionV1;
    f.type   = FrameType::Cmd;
    f.seq    = 1;
    f.cmdId  = CommandId::Ping;
    f.payload = {1, 2, 3, 4};

    auto bytes = FrameCodec::encode(f);
    bytes[0] = 0x00;

    EXPECT_FALSE(FrameCodec::decode(bytes.data(), bytes.size()).has_value());
}

TEST(FrameCodecTest, EncodeDecode_WithPayload) {
    Frame f;
    f.protocolVersion = kProtocolVersionV1;
    f.type   = FrameType::Cmd;
    f.seq    = 0x42;
    f.cmdId  = CommandId::LedSetSlot;
    f.payload = {0x03, 0xFF, 0x80, 0x00};

    const auto bytes   = FrameCodec::encode(f);
    const auto decoded = FrameCodec::decode(bytes.data(), bytes.size());

    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->cmdId,   CommandId::LedSetSlot);
    EXPECT_EQ(decoded->payload, f.payload);
}
