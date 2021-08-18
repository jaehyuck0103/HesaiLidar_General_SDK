#include "pandar/pandar40P.h"

#include <iostream>

void Pandar40P::Init() {

    blockOffsetSingle_ = {
        -(55.56f * 9.0f + 28.58f),
        -(55.56f * 8.0f + 28.58f),
        -(55.56f * 7.0f + 28.58f),
        -(55.56f * 6.0f + 28.58f),
        -(55.56f * 5.0f + 28.58f),
        -(55.56f * 4.0f + 28.58f),
        -(55.56f * 3.0f + 28.58f),
        -(55.56f * 2.0f + 28.58f),
        -(55.56f * 1.0f + 28.58f),
        -(55.56f * 0.0f + 28.58f)};

    blockOffsetDual_ = {
        -(55.56f * 4.0f + 28.58f),
        -(55.56f * 4.0f + 28.58f),
        -(55.56f * 3.0f + 28.58f),
        -(55.56f * 3.0f + 28.58f),
        -(55.56f * 2.0f + 28.58f),
        -(55.56f * 2.0f + 28.58f),
        -(55.56f * 1.0f + 28.58f),
        -(55.56f * 1.0f + 28.58f),
        -(55.56f * 0.0f + 28.58f),
        -(55.56f * 0.0f + 28.58f)};

    laserOffset_ = {-42.22f, -28.47f, -16.04f, -3.62f, -45.49f, -31.74f,   -47.46f, -54.67f,
                    -20.62f, -33.71f, -40.91f, -8.19f, -20.62f, -27.16f,   -50.73f, -8.19f,
                    -14.74f, -36.98f, -45.49f, -52.7f, -23.89f, -31.7447f, -38.95f, -11.47f,
                    -18.65f, -25.19f, -48.76f, -6.23f, -12.77f, -35.01f,   -21.92f, -9.5f,
                    -43.52f, -29.77f, -17.35f, -4.92f, -42.22f, -28.47f,   -16.04f, -3.62f};

    elev_angle_map_ = {15.0f,  11.0f,  8.0f,   5.0f,   3.0f,   2.0f,   1.67f,  1.33f,
                       1.0f,   0.67f,  0.33f,  0.0f,   -0.33f, -0.67f, -1.0f,  -1.33f,
                       -1.66f, -2.0f,  -2.33f, -2.67f, -3.0f,  -3.33f, -3.67f, -4.0f,
                       -4.33f, -4.67f, -5.0f,  -5.33f, -5.67f, -6.0f,  -7.0f,  -8.0f,
                       -9.0f,  -10.0f, -11.0f, -12.0f, -13.0f, -14.0f, -19.0f, -25.0f};

    azimuth_offset_map_ = {-1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, 3.125f,  -5.208f,
                           -1.042f, 3.125f,  -5.208f, -1.042f, 3.125f,  -5.208f, -1.042f, 3.125f,
                           -5.208f, -1.042f, 3.125f,  -5.208f, -1.042f, 3.125f,  -5.208f, -1.042f,
                           3.125f,  -5.208f, -1.042f, 3.125f,  -5.208f, -1.042f, -1.042f, -1.042f,
                           -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f};

    num_lasers_ = 40;
}

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

        // 40 units
        block.units.resize(HS_LIDAR_L40_LASER_COUNT);
        for (auto &unit : block.units) {

            unit.rawDistance = recvbuf[index] | recvbuf[index + 1] << 8;
            unit.intensity = recvbuf[index + 2];

            index += HS_LIDAR_L40_RAW_MEASURE_SIZE;
        }
    }

    index += HS_LIDAR_L40_RESERVE_SIZE;
    index += HS_LIDAR_L40_REVOLUTION_SIZE;

    uint32_t timestamp_us = recvbuf[index] | recvbuf[index + 1] << 8 | recvbuf[index + 2] << 16 |
                            recvbuf[index + 3] << 24;
    index += HS_LIDAR_L40_TIMESTAMP_SIZE;

    packet.returnMode = recvbuf[index];

    index += HS_LIDAR_L40_FACTORY_INFO_SIZE + HS_LIDAR_L40_ECHO_SIZE;

    packet.timestamp = parseUTC(&recvbuf[index]) + timestamp_us / 1000000.0;
    index += HS_LIDAR_L40_UTC_TIME;

    return packet;
}
