#include "pandar/pandarQT.h"

#include <iostream>

void PandarQT::Init() {

    blockOffsetSingle_ = {25.71f, 25.71f + 166.67f, 25.71f + 333.33f, 25.71f + 500.00f};
    blockOffsetDual_ = {25.71f, 25.71f, 25.71f + 166.67f, 25.71f + 166.67f};

    laserOffset_ = {
        10.0f + 2.31f,   10.0f + 4.37f,   10.0f + 6.43f,   10.0f + 8.49f,   10.0f + 10.54f,
        10.0f + 12.60f,  10.0f + 14.66f,  10.0f + 16.71f,  10.0f + 19.16f,  10.0f + 21.22f,
        10.0f + 23.28f,  10.0f + 25.34f,  10.0f + 27.39f,  10.0f + 29.45f,  10.0f + 31.50f,
        10.0f + 33.56f,  10.0f + 36.61f,  10.0f + 38.67f,  10.0f + 40.73f,  10.0f + 42.78f,
        10.0f + 44.84f,  10.0f + 46.90f,  10.0f + 48.95f,  10.0f + 51.01f,  10.0f + 53.45f,
        10.0f + 55.52f,  10.0f + 57.58f,  10.0f + 59.63f,  10.0f + 61.69f,  10.0f + 63.74f,
        10.0f + 65.80f,  10.0f + 67.86f,  10.0f + 70.90f,  10.0f + 72.97f,  10.0f + 75.02f,
        10.0f + 77.08f,  10.0f + 79.14f,  10.0f + 81.19f,  10.0f + 83.25f,  10.0f + 85.30f,
        10.0f + 87.75f,  10.0f + 89.82f,  10.0f + 91.87f,  10.0f + 93.93f,  10.0f + 95.98f,
        10.0f + 98.04f,  10.0f + 100.10f, 10.0f + 102.15f, 10.0f + 105.20f, 10.0f + 107.26f,
        10.0f + 109.32f, 10.0f + 111.38f, 10.0f + 113.43f, 10.0f + 115.49f, 10.0f + 117.54f,
        10.0f + 119.60f, 10.0f + 122.05f, 10.0f + 124.11f, 10.0f + 126.17f, 10.0f + 128.22f,
        10.0f + 130.28f, 10.0f + 132.34f, 10.0f + 134.39f, 10.0f + 136.45f};

    num_lasers_ = 64;

    if (fps_ == 10) {
        azimuth_res_ = 60;
    } else {
        std::cout << "Unavailable FPS: " << fps_ << std::endl;
        std::terminate();
    }
}

std::optional<HS_LIDAR_Packet> PandarQT::parseLidarPacket(const std::vector<uint8_t> &recvbuf) {

    if (recvbuf.size() != HS_LIDAR_QT_PACKET_SIZE &&
        recvbuf.size() != HS_LIDAR_QT_PACKET_WITHOUT_UDPSEQ_SIZE) {
        std::cout << "Packet Size Mismatch (PandarQT): " << recvbuf.size() << "\n";
        return std::nullopt;
    }

    int index = 0;

    // Parse 12 Bytes Header
    uint16_t sop = recvbuf[index] << 8 | recvbuf[index + 1];
    uint8_t protocolVerMajor = recvbuf[index + 2];
    uint8_t nLasers = recvbuf[index + 6]; // 64
    uint8_t nBlocks = recvbuf[index + 7]; // 4
    uint8_t disUnit = recvbuf[index + 9]; // 4 (mm)
    index += HS_LIDAR_QT_HEAD_SIZE;

    if (sop != 0xEEFF) {
        std::cout << "Error Start of Packet!\n";
        return std::nullopt;
    }
    if (protocolVerMajor != 0x03) {
        std::cout << "Error protocalVerMajor!\n";
        return std::nullopt;
    }
    if (nLasers != num_lasers_) {
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

    HS_LIDAR_Packet packet;
    packet.blocks.resize(nBlocks);
    for (auto &block : packet.blocks) {
        block.azimuth = recvbuf[index] | recvbuf[index + 1] << 8;
        index += HS_LIDAR_QT_BLOCK_HEADER_AZIMUTH;

        block.payload = std::vector<uint8_t>(
            recvbuf.begin() + index,
            recvbuf.begin() + index + 3 * num_lasers_);
        index += 3 * num_lasers_;
    }

    index += HS_LIDAR_QT_RESERVED_SIZE;
    index += HS_LIDAR_QT_ENGINE_VELOCITY;

    uint32_t timestamp_us = recvbuf[index] | recvbuf[index + 1] << 8 | recvbuf[index + 2] << 16 |
                            recvbuf[index + 3] << 24;
    index += HS_LIDAR_QT_TIMESTAMP_SIZE;

    uint8_t returnMode = recvbuf[index];
    if (dual_return_mode_ != (returnMode >= 0x39)) {
        std::cout << "Return Mode Mismatch: 0x" << std::hex << static_cast<int>(returnMode)
                  << "\n";
        return std::nullopt;
    }
    index += HS_LIDAR_QT_ECHO_SIZE;
    index += HS_LIDAR_QT_FACTORY_SIZE;

    packet.timestamp = parseUTC(&recvbuf[index]) + timestamp_us / 1000000.0;
    index += HS_LIDAR_QT_UTC_SIZE;

    return packet;
}
