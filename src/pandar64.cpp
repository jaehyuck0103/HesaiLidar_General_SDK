#include "pandar/pandar64.h"

#include <iostream>

std::optional<HS_LIDAR_Packet> Pandar64::parseLidarPacket(const std::vector<uint8_t> &recvbuf) {

    int index = 0;

    // Check Packet Size
    if (recvbuf.size() != 1194 && recvbuf.size() != 1198) {
        std::cout << "Packet Size Mismatch (Pandar64): " << recvbuf.size() << "\n";
        return std::nullopt;
    }

    // Parse 8 Bytes Header
    uint16_t sop = recvbuf[index] << 8 | recvbuf[index + 1];
    uint8_t nLasers = recvbuf[index + 2]; // 64
    uint8_t nBlocks = recvbuf[index + 3]; // 6
    uint8_t disUnit = recvbuf[index + 5]; // 4(mm)
    index += 8;                           // header

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
        index += 2; // azimuth

        block.payload = std::vector<uint8_t>(
            recvbuf.begin() + index,
            recvbuf.begin() + index + cfg_.elem_bytes * cfg_.num_lasers());
        index += cfg_.elem_bytes * cfg_.num_lasers(); // block payload
    }

    index += 8; // reserved
    index += 2; // motor speed

    uint32_t timestamp_us = recvbuf[index] | recvbuf[index + 1] << 8 | recvbuf[index + 2] << 16 |
                            recvbuf[index + 3] << 24;
    index += 4; // GPS Timestamp

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
