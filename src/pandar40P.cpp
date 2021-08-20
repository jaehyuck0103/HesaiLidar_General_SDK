#include "pandar/pandar40P.h"

#include <iostream>

std::optional<HS_LIDAR_Packet> Pandar40P::parseLidarPacket(const std::vector<uint8_t> &recvbuf) {

    if (recvbuf.size() != HS_LIDAR_L40_PACKET_SIZE &&
        recvbuf.size() != HS_LIDAR_L40_PACKET_SIZE + HS_LIDAR_L40_SEQ_NUM_SIZE) {
        std::cout << "Packet Size Mismatch (Pandar40): " << recvbuf.size() << "\n";
        return std::nullopt;
    }

    int index = 0;
    HS_LIDAR_Packet packet;
    // 10 BLOCKs
    packet.blocks.resize(HS_LIDAR_L40_BLOCKS_PER_PACKET);
    for (auto &block : packet.blocks) {

        uint16_t sob = recvbuf[index] | recvbuf[index + 1] << 8;
        block.azimuth = recvbuf[index + 2] | recvbuf[index + 3] << 8;
        index += HS_LIDAR_L40_SOB_SIZE;

        if (sob != 0xEEFF) {
            std::cout << "Error Start of Block!\n";
            return std::nullopt;
        }

        block.payload = std::vector<uint8_t>(
            recvbuf.begin() + index,
            recvbuf.begin() + index + 3 * cfg_.num_lasers());
        index += 3 * cfg_.num_lasers();
    }

    index += HS_LIDAR_L40_RESERVE_SIZE;
    index += HS_LIDAR_L40_REVOLUTION_SIZE;

    uint32_t timestamp_us = recvbuf[index] | recvbuf[index + 1] << 8 | recvbuf[index + 2] << 16 |
                            recvbuf[index + 3] << 24;
    index += HS_LIDAR_L40_TIMESTAMP_SIZE;

    uint8_t returnMode = recvbuf[index];
    if (cfg_.dual_return_mode() != (returnMode >= 0x39)) {
        std::cout << "Return Mode Mismatch: 0x" << std::hex << static_cast<int>(returnMode)
                  << "\n";
        return std::nullopt;
    }

    index += HS_LIDAR_L40_FACTORY_INFO_SIZE + HS_LIDAR_L40_ECHO_SIZE;

    packet.timestamp = parseUTC(&recvbuf[index]) + timestamp_us / 1000000.0;
    index += HS_LIDAR_L40_UTC_TIME;

    return packet;
}
