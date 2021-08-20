#include "pandar/pandar64.h"

#include <iostream>

void Pandar64::Init() {

    blockOffsetSingle_ = {
        -(55.56f * 5.0f + 42.58f),
        -(55.56f * 4.0f + 42.58f),
        -(55.56f * 3.0f + 42.58f),
        -(55.56f * 2.0f + 42.58f),
        -(55.56f * 1.0f + 42.58f),
        -(55.56f * 0.0f + 42.58f)};

    blockOffsetDual_ = {
        -(55.56f * 2.0f + 42.58f),
        -(55.56f * 2.0f + 42.58f),
        -(55.56f * 1.0f + 42.58f),
        -(55.56f * 1.0f + 42.58f),
        -(55.56f * 0.0f + 42.58f),
        -(55.56f * 0.0f + 42.58f)};

    laserOffset_ = {
        -(1.304f * 15.0f + 1.968f * 0.0f + 3.62f),  -(1.304f * 14.0f + 1.968f * 0.0f + 3.62f),
        -(1.304f * 13.0f + 1.968f * 0.0f + 3.62f),  -(1.304f * 12.0f + 1.968f * 0.0f + 3.62f),
        -(1.304f * 11.0f + 1.968f * 0.0f + 3.62f),  -(1.304f * 10.0f + 1.968f * 0.0f + 3.62f),
        -(1.304f * 6.0f + 1.968f * 0.0f + 3.62f),   -(1.304f * 15.0f + 1.968f * 12.0f + 3.62f),
        -(1.304f * 3.0f + 1.968f * 0.0f + 3.62f),   -(1.304f * 15.0f + 1.968f * 7.0f + 3.62f),
        -(1.304f * 15.0f + 1.968f * 14.0f + 3.62f), -(1.304f * 15.0f + 1.968f * 16.0f + 3.62f),
        -(1.304f * 15.0f + 1.968f * 9.0f + 3.62f),  -(1.304f * 15.0f + 1.968f * 11.0f + 3.62f),
        -(1.304f * 15.0f + 1.968f * 4.0f + 3.62f),  -(1.304f * 15.0f + 1.968f * 6.0f + 3.62f),
        -(1.304f * 15.0f + 1.968f * 13.0f + 3.62f), -(1.304f * 15.0f + 1.968f * 15.0f + 3.62f),
        -(1.304f * 15.0f + 1.968f * 8.0f + 3.62f),  -(1.304f * 15.0f + 1.968f * 10.0f + 3.62f),
        -(1.304f * 15.0f + 1.968f * 3.0f + 3.62f),  -(1.304f * 15.0f + 1.968f * 5.0f + 3.62f),
        -(1.304f * 15.0f + 1.968f * 12.0f + 3.62f), -(1.304f * 15.0f + 1.968f * 1.0f + 3.62f),
        -(1.304f * 15.0f + 1.968f * 7.0f + 3.62f),  -(1.304f * 15.0f + 1.968f * 14.0f + 3.62f),
        -(1.304f * 15.0f + 1.968f * 2.0f + 3.62f),  -(1.304f * 15.0f + 1.968f * 9.0f + 3.62f),
        -(1.304f * 15.0f + 1.968f * 11.0f + 3.62f), -(1.304f * 15.0f + 1.968f * 4.0f + 3.62f),
        -(1.304f * 15.0f + 1.968f * 6.0f + 3.62f),  -(1.304f * 15.0f + 1.968f * 13.0f + 3.62f),
        -(1.304f * 15.0f + 1.968f * 1.0f + 3.62f),  -(1.304f * 15.0f + 1.968f * 8.0f + 3.62f),
        -(1.304f * 15.0f + 1.968f * 10.0f + 3.62f), -(1.304f * 15.0f + 1.968f * 3.0f + 3.62f),
        -(1.304f * 15.0f + 1.968f * 5.0f + 3.62f),  -(1.304f * 15.0f + 1.968f * 15.0f + 3.62f),
        -(1.304f * 2.0f + 1.968f * 0.0f + 3.62f),   -(1.304f * 15.0f + 1.968f * 16.0f + 3.62f),
        -(1.304f * 9.0f + 1.968f * 0.0f + 3.62f),   -(1.304f * 15.0f + 1.968f * 2.0f + 3.62f),
        -(1.304f * 5.0f + 1.968f * 0.0f + 3.62f),   -(1.304f * 15.0f + 1.968f * 0.0f + 3.62f),
        -(1.304f * 1.0f + 1.968f * 0.0f + 3.62f),   -(1.304f * 14.0f + 1.968f * 0.0f + 3.62f),
        -(1.304f * 8.0f + 1.968f * 0.0f + 3.62f),   -(1.304f * 11.0f + 1.968f * 0.0f + 3.62f),
        -(1.304f * 4.0f + 1.968f * 0.0f + 3.62f),   -(1.304f * 12.0f + 1.968f * 0.0f + 3.62f),
        -(1.304f * 0.0f + 1.968f * 0.0f + 3.62f),   -(1.304f * 13.0f + 1.968f * 0.0f + 3.62f),
        -(1.304f * 7.0f + 1.968f * 0.0f + 3.62f),   -(1.304f * 10.0f + 1.968f * 0.0f + 3.62f),
        -(1.304f * 3.0f + 1.968f * 0.0f + 3.62f),   -(1.304f * 6.0f + 1.968f * 0.0f + 3.62f),
        -(1.304f * 2.0f + 1.968f * 0.0f + 3.62f),   -(1.304f * 9.0f + 1.968f * 0.0f + 3.62f),
        -(1.304f * 5.0f + 1.968f * 0.0f + 3.62f),   -(1.304f * 1.0f + 1.968f * 0.0f + 3.62f),
        -(1.304f * 0.0f + 1.968f * 0.0f + 3.62f),   -(1.304f * 8.0f + 1.968f * 0.0f + 3.62f),
        -(1.304f * 4.0f + 1.968f * 0.0f + 3.62f),   -(1.304f * 7.0f + 1.968f * 0.0f + 3.62f)};
}

std::optional<HS_LIDAR_Packet> Pandar64::parseLidarPacket(const std::vector<uint8_t> &recvbuf) {

    if (recvbuf.size() != HS_LIDAR_L64_PACKET_SIZE &&
        recvbuf.size() != HS_LIDAR_L64_PACKET_WITHOUT_UDPSEQ_SIZE) {
        std::cout << "Packet Size Mismatch (Pandar64): " << recvbuf.size() << "\n";
        return std::nullopt;
    }

    int index = 0;

    // Parse 8 Bytes Header
    uint16_t sop = recvbuf[index] << 8 | recvbuf[index + 1];
    uint8_t nLasers = recvbuf[index + 2]; // 64
    uint8_t nBlocks = recvbuf[index + 3]; // 6
    uint8_t disUnit = recvbuf[index + 5]; // 4(mm)
    index += HS_LIDAR_L64_HEAD_SIZE;

    if (sop != 0xEEFF) {
        std::cout << "Error Start of Packet!\n";
        return std::nullopt;
    }
    if (nLasers != cfg_.num_lasers()) {
        std::cout << "Error nLasers!\n";
        return std::nullopt;
    }
    if (nBlocks != 0x06) {
        std::cout << "Error nBlocks!\n";
        return std::nullopt;
    }
    if (disUnit != 0x04) {
        std::cout << "Error disUnit!\n";
        return std::nullopt;
    }

    HS_LIDAR_Packet packet;
    packet.blocks.resize(nBlocks);
    for (auto &block : packet.blocks) {
        block.azimuth = recvbuf[index] | recvbuf[index + 1] << 8;
        index += HS_LIDAR_L64_BLOCK_HEADER_AZIMUTH;

        block.payload = std::vector<uint8_t>(
            recvbuf.begin() + index,
            recvbuf.begin() + index + 3 * cfg_.num_lasers());
        index += 3 * cfg_.num_lasers();
    }

    index += HS_LIDAR_L64_RESERVED_SIZE;
    index += HS_LIDAR_L64_ENGINE_VELOCITY;

    uint32_t timestamp_us = recvbuf[index] | recvbuf[index + 1] << 8 | recvbuf[index + 2] << 16 |
                            recvbuf[index + 3] << 24;
    index += HS_LIDAR_L64_TIMESTAMP_SIZE;

    uint8_t returnMode = recvbuf[index];
    if (cfg_.dual_return_mode() != (returnMode >= 0x39)) {
        std::cout << "Return Mode Mismatch: 0x" << std::hex << static_cast<int>(returnMode)
                  << "\n";
        return std::nullopt;
    }
    index += HS_LIDAR_L64_RETURN_MODE_SIZE;
    index += HS_LIDAR_L64_FACTORY_SIZE;

    packet.timestamp = parseUTC(&recvbuf[index]) + timestamp_us / 1000000.0;
    index += HS_LIDAR_L64_TIME_SIZE;

    return packet;
}
