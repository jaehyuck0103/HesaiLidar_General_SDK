#include "pandar/pandar40P.h"

#include <iostream>

std::optional<HS_LIDAR_Packet> Pandar40P::parseLidarPacket(const std::vector<uint8_t> &recvbuf) {

    int index = 0;

    // Check Packet Size
    if (recvbuf.size() != 1262 && recvbuf.size() != 1266) {
        std::cout << "Packet Size Mismatch (Pandar40P): " << recvbuf.size() << "\n";
        return std::nullopt;
    }

    HS_LIDAR_Packet packet;
    packet.blocks.resize(10); // 10 blocks
    for (auto &block : packet.blocks) {

        uint16_t sob = recvbuf[index] | recvbuf[index + 1] << 8;
        block.azimuth = recvbuf[index + 2] | recvbuf[index + 3] << 8;
        index += 4; // sob + azimuth

        if (sob != 0xEEFF) {
            std::cout << "Error Start of Block!\n";
            return std::nullopt;
        }

        block.payload = std::vector<uint8_t>(
            recvbuf.begin() + index,
            recvbuf.begin() + index + cfg_.elem_bytes * cfg_.num_lasers());
        index += cfg_.elem_bytes * cfg_.num_lasers(); // block payload
    }

    index += 8; // reserved
    index += 2; // motor speed

    uint32_t timestamp_us = recvbuf[index] | recvbuf[index + 1] << 8 | recvbuf[index + 2] << 16 |
                            recvbuf[index + 3] << 24;
    index += 4; // timestamp

    uint8_t returnMode = recvbuf[index];
    if (cfg_.dual_return_mode() != (returnMode >= 0x39)) {
        std::cout << "Return Mode Mismatch: 0x" << std::hex << static_cast<int>(returnMode)
                  << "\n";
        return std::nullopt;
    }

    index += 1; // return mode
    index += 1; // factory info

    packet.timestamp = parseUTC(&recvbuf[index]) + timestamp_us / 1000000.0;
    index += 6; // Date & Time

    return packet;
}
