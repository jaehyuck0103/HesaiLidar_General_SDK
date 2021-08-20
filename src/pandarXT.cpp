#include "pandar/pandarXT.h"

#include <iostream>

std::optional<HS_LIDAR_Packet> PandarXT::parseLidarPacket(const std::vector<uint8_t> &recvbuf) {

    int index = 0;

    // Check Packet Size
    if (recvbuf.size() != 568 && recvbuf.size() != 1080) {
        std::cout << "Packet Size Mismatch (PandarXT): " << recvbuf.size() << "\n";
        return std::nullopt;
    }

    // Parse 12 Bytes Header
    uint16_t sop = recvbuf[index] << 8 | recvbuf[index + 1];
    uint8_t protocolVerMajor = recvbuf[index + 2];
    uint8_t nLasers = recvbuf[index + 6]; // 32 or 16
    uint8_t nBlocks = recvbuf[index + 7]; // 8
    uint8_t disUnit = recvbuf[index + 9]; // 4 (mm)
    index += 12;                          // header

    if (sop != 0xEEFF) {
        std::cout << "Error Start of Packet!\n";
        return std::nullopt;
    }
    if (protocolVerMajor != 0x06) {
        std::cout << "Error protocalVerMajor!\n";
        return std::nullopt;
    }
    if (static_cast<int>(nLasers) != cfg_.num_lasers()) {
        std::cout << "Error nLasers!\n";
        return std::nullopt;
    }
    if (nBlocks != 0x08) {
        std::cout << "Error nBlocks!\n";
        return std::nullopt;
    }
    if (disUnit != 0x04) {
        std::cout << "Error disUnit!\n";
        return std::nullopt;
    }

    // Parse body
    HS_LIDAR_Packet packet;
    packet.blocks.resize(nBlocks);
    for (auto &block : packet.blocks) {
        block.azimuth = recvbuf[index] | recvbuf[index + 1] << 8;
        index += 2; // azimuth

        block.payload.resize(cfg_.elem_bytes * cfg_.num_lasers());
        for (int i = 0; i < cfg_.num_lasers(); ++i) {
            std::copy(
                recvbuf.begin() + index,
                recvbuf.begin() + index + cfg_.elem_bytes,
                block.payload.begin() + i * cfg_.elem_bytes);
            index += cfg_.elem_bytes + 1; // 3bytes (raw_distance + intensity) + 1byte reserved
        }
    }

    index += 10; // reserved

    uint8_t returnMode = recvbuf[index];
    if (cfg_.dual_return_mode() != (returnMode >= 0x39)) {
        std::cout << "Return Mode Mismatch: 0x" << std::hex << static_cast<int>(returnMode)
                  << "\n";
        return std::nullopt;
    }
    index += 1; // return mode
    index += 2; // motor speed

    packet.timestamp = parseUTC(&recvbuf[index]);
    index += 6; // Date & Time

    uint32_t timestamp_us = recvbuf[index] | recvbuf[index + 1] << 8 | recvbuf[index + 2] << 16 |
                            recvbuf[index + 3] << 24;
    packet.timestamp += timestamp_us / 1000000.0;
    index += 4; // Timestamp

    return packet;
}
