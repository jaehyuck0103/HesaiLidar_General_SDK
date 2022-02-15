#include "pandar_packet_parsers.h"

#include <ctime>
#include <iostream>

namespace PandarPacketParsers {

double parseUTC(const uint8_t *buf) {
    // Do not use for a while
    if constexpr (false) {
        std::tm tTm;
        tTm.tm_year = buf[0] + 100; // years since 1900
        tTm.tm_mon = buf[1] - 1;    // years since 1900
        tTm.tm_mday = buf[2];
        tTm.tm_hour = buf[3];
        tTm.tm_min = buf[4];
        tTm.tm_sec = buf[5];
        tTm.tm_isdst = 0;

        return static_cast<double>(std::mktime(&tTm));
    } else {
        return 0.0;
    }
}

std::optional<HS_LIDAR_Packet>
pandar40p(const PandarConfig &cfg, std::span<const uint8_t> recvbuf) {

    // Check Packet Size
    if (recvbuf.size() != 1262 && recvbuf.size() != 1266) {
        std::cout << "Packet Size Mismatch (Pandar40P): " << recvbuf.size() << "\n";
        return std::nullopt;
    }

    HS_LIDAR_Packet packet;
    packet.blocks.resize(10); // 10 blocks
    for (auto &block : packet.blocks) {

        uint16_t sob = recvbuf[0] | recvbuf[1] << 8;
        block.azimuth = recvbuf[2] | recvbuf[3] << 8;
        recvbuf = recvbuf.subspan(4); // sob + azimuth

        if (sob != 0xEEFF) {
            std::cout << "Error Start of Block!\n";
            return std::nullopt;
        }

        block.payload = std::vector<uint8_t>(
            recvbuf.begin(),
            recvbuf.begin() + cfg.elem_bytes * cfg.num_lasers());
        recvbuf = recvbuf.subspan(cfg.elem_bytes * cfg.num_lasers()); // block payload
    }

    recvbuf = recvbuf.subspan(8); // reserved
    recvbuf = recvbuf.subspan(2); // motor speed

    uint32_t timestamp_us = recvbuf[0] | recvbuf[1] << 8 | recvbuf[2] << 16 | recvbuf[3] << 24;
    recvbuf = recvbuf.subspan(4); // timestamp

    uint8_t returnMode = recvbuf[0];
    if (cfg.dual_return_mode() != (returnMode >= 0x39)) {
        std::cout << "Return Mode Mismatch: 0x" << std::hex << static_cast<int>(returnMode)
                  << "\n";
        return std::nullopt;
    }

    recvbuf = recvbuf.subspan(1); // return mode
    recvbuf = recvbuf.subspan(1); // factory info

    packet.timestamp = parseUTC(&recvbuf[0]) + timestamp_us / 1000000.0;
    recvbuf = recvbuf.subspan(6); // Date & Time

    return packet;
};

std::optional<HS_LIDAR_Packet>
pandar64(const PandarConfig &cfg, std::span<const uint8_t> recvbuf) {

    // Check Packet Size
    if (recvbuf.size() != 1194 && recvbuf.size() != 1198) {
        std::cout << "Packet Size Mismatch (Pandar64): " << recvbuf.size() << "\n";
        return std::nullopt;
    }

    // Parse 8 Bytes Header
    uint16_t sop = recvbuf[0] << 8 | recvbuf[1];
    uint8_t nLasers = recvbuf[2]; // 64
    uint8_t nBlocks = recvbuf[3]; // 6
    uint8_t disUnit = recvbuf[5]; // 4(mm)
    recvbuf = recvbuf.subspan(8); // header

    if (sop != 0xEEFF) {
        std::cout << "Error Start of Packet!\n";
        return std::nullopt;
    }
    if (nLasers != cfg.num_lasers()) {
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
        block.azimuth = recvbuf[0] | recvbuf[1] << 8;
        recvbuf = recvbuf.subspan(2); // azimuth

        block.payload = std::vector<uint8_t>(
            recvbuf.begin(),
            recvbuf.begin() + cfg.elem_bytes * cfg.num_lasers());
        recvbuf = recvbuf.subspan(cfg.elem_bytes * cfg.num_lasers()); // block payload
    }

    recvbuf = recvbuf.subspan(8); // reserved
    recvbuf = recvbuf.subspan(2); // motor speed

    uint32_t timestamp_us = recvbuf[0] | recvbuf[1] << 8 | recvbuf[2] << 16 | recvbuf[3] << 24;
    recvbuf = recvbuf.subspan(4); // GPS Timestamp

    uint8_t returnMode = recvbuf[0];
    if (cfg.dual_return_mode() != (returnMode >= 0x39)) {
        std::cout << "Return Mode Mismatch: 0x" << std::hex << static_cast<int>(returnMode)
                  << "\n";
        return std::nullopt;
    }
    recvbuf = recvbuf.subspan(1); // return mode
    recvbuf = recvbuf.subspan(1); // factory info

    packet.timestamp = parseUTC(&recvbuf[0]) + timestamp_us / 1000000.0;
    recvbuf = recvbuf.subspan(6); // Date & Time

    return packet;
}

std::optional<HS_LIDAR_Packet>
pandarQT(const PandarConfig &cfg, std::span<const uint8_t> recvbuf) {

    // Check Packet Size
    if (recvbuf.size() != 1068 && recvbuf.size() != 1072) {
        std::cout << "Packet Size Mismatch (PandarQT): " << recvbuf.size() << "\n";
        return std::nullopt;
    }

    // Parse 12 Bytes Header
    uint16_t sop = recvbuf[0] << 8 | recvbuf[1];
    uint8_t protocolVerMajor = recvbuf[2];
    uint8_t nLasers = recvbuf[6];  // 64
    uint8_t nBlocks = recvbuf[7];  // 4
    uint8_t disUnit = recvbuf[9];  // 4 (mm)
    recvbuf = recvbuf.subspan(12); // header

    if (sop != 0xEEFF) {
        std::cout << "Error Start of Packet!\n";
        return std::nullopt;
    }
    if (protocolVerMajor != 0x03) {
        std::cout << "Error protocalVerMajor!\n";
        return std::nullopt;
    }
    if (nLasers != cfg.num_lasers()) {
        std::cout << "Error nLasers!\n";
        return std::nullopt;
    }
    if (nBlocks != 0x04) {
        std::cout << "Error nBlocks!\n";
        return std::nullopt;
    }
    if (disUnit != 0x04) {
        std::cout << "Error disUnit!\n";
        return std::nullopt;
    }

    // Parse Body
    HS_LIDAR_Packet packet;
    packet.blocks.resize(nBlocks);
    for (auto &block : packet.blocks) {
        block.azimuth = recvbuf[0] | recvbuf[1] << 8;
        recvbuf = recvbuf.subspan(2); // azimuth

        block.payload.resize(cfg.elem_bytes * cfg.num_lasers());
        for (int i = 0; i < cfg.num_lasers(); ++i) {
            std::copy(
                recvbuf.begin(),
                recvbuf.begin() + cfg.elem_bytes,
                block.payload.begin() + i * cfg.elem_bytes);
            recvbuf = recvbuf.subspan(
                cfg.elem_bytes + 1); // 3bytes (raw_distance + intensity) + 1byte reserved
        }
    }

    recvbuf = recvbuf.subspan(10); // reserved
    recvbuf = recvbuf.subspan(2);  // motor speed

    uint32_t timestamp_us = recvbuf[0] | recvbuf[1] << 8 | recvbuf[2] << 16 | recvbuf[3] << 24;
    recvbuf = recvbuf.subspan(4); // timestamp

    uint8_t returnMode = recvbuf[0];
    if (cfg.dual_return_mode() != (returnMode >= 0x39)) {
        std::cout << "Return Mode Mismatch: 0x" << std::hex << static_cast<int>(returnMode)
                  << "\n";
        return std::nullopt;
    }
    recvbuf = recvbuf.subspan(1); // return_mode
    recvbuf = recvbuf.subspan(1); // factory info

    packet.timestamp = parseUTC(&recvbuf[0]) + timestamp_us / 1000000.0;
    recvbuf = recvbuf.subspan(6); // Date & Time

    return packet;
}

std::optional<HS_LIDAR_Packet>
pandarXT(const PandarConfig &cfg, std::span<const uint8_t> recvbuf) {

    // Check Packet Size
    if (recvbuf.size() != 568 && recvbuf.size() != 1080) {
        std::cout << "Packet Size Mismatch (PandarXT): " << recvbuf.size() << "\n";
        return std::nullopt;
    }

    // Parse 12 Bytes Header
    uint16_t sop = recvbuf[0] << 8 | recvbuf[1];
    uint8_t protocolVerMajor = recvbuf[2];
    uint8_t nLasers = recvbuf[6];  // 32 or 16
    uint8_t nBlocks = recvbuf[7];  // 8
    uint8_t disUnit = recvbuf[9];  // 4 (mm)
    recvbuf = recvbuf.subspan(12); // header

    if (sop != 0xEEFF) {
        std::cout << "Error Start of Packet!\n";
        return std::nullopt;
    }
    if (protocolVerMajor != 0x06) {
        std::cout << "Error protocalVerMajor!\n";
        return std::nullopt;
    }
    if (static_cast<int>(nLasers) != cfg.num_lasers()) {
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
        block.azimuth = recvbuf[0] | recvbuf[1] << 8;
        recvbuf = recvbuf.subspan(2); // azimuth

        block.payload.resize(cfg.elem_bytes * cfg.num_lasers());
        for (int i = 0; i < cfg.num_lasers(); ++i) {
            std::copy(
                recvbuf.begin(),
                recvbuf.begin() + cfg.elem_bytes,
                block.payload.begin() + i * cfg.elem_bytes);
            recvbuf = recvbuf.subspan(
                cfg.elem_bytes + 1); // 3bytes (raw_distance + intensity) + 1byte reserved
        }
    }

    recvbuf = recvbuf.subspan(10); // reserved

    uint8_t returnMode = recvbuf[0];
    if (cfg.dual_return_mode() != (returnMode >= 0x39)) {
        std::cout << "Return Mode Mismatch: 0x" << std::hex << static_cast<int>(returnMode)
                  << "\n";
        return std::nullopt;
    }
    recvbuf = recvbuf.subspan(1); // return mode
    recvbuf = recvbuf.subspan(2); // motor speed

    packet.timestamp = parseUTC(&recvbuf[0]);
    recvbuf = recvbuf.subspan(6); // Date & Time

    uint32_t timestamp_us = recvbuf[0] | recvbuf[1] << 8 | recvbuf[2] << 16 | recvbuf[3] << 24;
    packet.timestamp += timestamp_us / 1000000.0;
    recvbuf = recvbuf.subspan(4); // Timestamp

    return packet;
}

} // namespace PandarPacketParsers
