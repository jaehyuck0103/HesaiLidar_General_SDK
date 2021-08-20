#include "pandar/pandarXT.h"

#include <iostream>

void PandarXT::Init() {

    laserOffset_.clear();
    for (int i = 0; i < HS_LIDAR_XT_UNIT_NUM; ++i) {
        laserOffset_.push_back(1.512f * i + 0.28f);
    }

    if (cfg_.lidar_model() == LidarModel::PandarXT_16) {
        for (int i = 0; i < HS_LIDAR_XT16_UNIT_NUM; i++) {
            laserOffset_[i] = laserOffset_[i * 2];
        }
        laserOffset_.resize(16);
    }

    blockOffsetSingle_ = {
        3.28f - 50.0f * 7.0f,
        3.28f - 50.0f * 6.0f,
        3.28f - 50.0f * 5.0f,
        3.28f - 50.0f * 4.0f,
        3.28f - 50.0f * 3.0f,
        3.28f - 50.0f * 2.0f,
        3.28f - 50.0f * 1.0f,
        3.28f - 50.0f * 0.0f};

    blockOffsetDual_ = {
        3.28f - 50.0f * 3.0f,
        3.28f - 50.0f * 3.0f,
        3.28f - 50.0f * 2.0f,
        3.28f - 50.0f * 2.0f,
        3.28f - 50.0f * 1.0f,
        3.28f - 50.0f * 1.0f,
        3.28f - 50.0f * 0.0f,
        3.28f - 50.0f * 0.0f};
}

std::optional<HS_LIDAR_Packet> PandarXT::parseLidarPacket(const std::vector<uint8_t> &recvbuf) {

    if (recvbuf.size() != HS_LIDAR_XT_PACKET_SIZE && recvbuf.size() != HS_LIDAR_XT16_PACKET_SIZE) {
        std::cout << "Packet Size Mismatch (PandarXT): " << recvbuf.size() << "\n";
        return std::nullopt;
    }

    int index = 0;
    // Parse 12 Bytes Header
    uint16_t sop = recvbuf[index] << 8 | recvbuf[index + 1];
    uint8_t protocolVerMajor = recvbuf[index + 2];
    uint8_t nLasers = recvbuf[index + 6]; // 32 or 16
    uint8_t nBlocks = recvbuf[index + 7]; // 8
    uint8_t disUnit = recvbuf[index + 9]; // 4 (mm)
    index += HS_LIDAR_XT_HEAD_SIZE;

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

    HS_LIDAR_Packet packet;
    packet.blocks.resize(nBlocks);
    for (auto &block : packet.blocks) {
        block.azimuth = recvbuf[index] | recvbuf[index + 1] << 8;
        index += HS_LIDAR_XT_BLOCK_HEADER_AZIMUTH;

        block.payload = std::vector<uint8_t>(
            recvbuf.begin() + index,
            recvbuf.begin() + index + 3 * cfg_.num_lasers());
        index += 3 * cfg_.num_lasers();
    }

    index += HS_LIDAR_XT_RESERVED_SIZE;

    uint8_t returnMode = recvbuf[index];
    if (cfg_.dual_return_mode() != (returnMode >= 0x39)) {
        std::cout << "Return Mode Mismatch: 0x" << std::hex << static_cast<int>(returnMode)
                  << "\n";
        return std::nullopt;
    }
    index += HS_LIDAR_XT_ECHO_SIZE;
    index += HS_LIDAR_XT_ENGINE_VELOCITY;

    packet.timestamp = parseUTC(&recvbuf[index]);
    index += HS_LIDAR_XT_UTC_SIZE;

    uint32_t timestamp_us = recvbuf[index] | recvbuf[index + 1] << 8 | recvbuf[index + 2] << 16 |
                            recvbuf[index + 3] << 24;
    packet.timestamp += timestamp_us / 1000000.0;

    return packet;
}
