#include "pandar/pandar40P.h"

#include <iostream>

void Pandar40P::Init() {

    blockOffsetSingle_.resize(HS_LIDAR_L40_BLOCKS_PER_PACKET);
    blockOffsetSingle_[9] = -(55.56f * 0.0f + 28.58f);
    blockOffsetSingle_[8] = -(55.56f * 1.0f + 28.58f);
    blockOffsetSingle_[7] = -(55.56f * 2.0f + 28.58f);
    blockOffsetSingle_[6] = -(55.56f * 3.0f + 28.58f);
    blockOffsetSingle_[5] = -(55.56f * 4.0f + 28.58f);
    blockOffsetSingle_[4] = -(55.56f * 5.0f + 28.58f);
    blockOffsetSingle_[3] = -(55.56f * 6.0f + 28.58f);
    blockOffsetSingle_[2] = -(55.56f * 7.0f + 28.58f);
    blockOffsetSingle_[1] = -(55.56f * 8.0f + 28.58f);
    blockOffsetSingle_[0] = -(55.56f * 9.0f + 28.58f);

    blockOffsetDual_.resize(HS_LIDAR_L40_BLOCKS_PER_PACKET);
    blockOffsetDual_[9] = -(55.56f * 0.0f + 28.58f);
    blockOffsetDual_[8] = -(55.56f * 0.0f + 28.58f);
    blockOffsetDual_[7] = -(55.56f * 1.0f + 28.58f);
    blockOffsetDual_[6] = -(55.56f * 1.0f + 28.58f);
    blockOffsetDual_[5] = -(55.56f * 2.0f + 28.58f);
    blockOffsetDual_[4] = -(55.56f * 2.0f + 28.58f);
    blockOffsetDual_[3] = -(55.56f * 3.0f + 28.58f);
    blockOffsetDual_[2] = -(55.56f * 3.0f + 28.58f);
    blockOffsetDual_[1] = -(55.56f * 4.0f + 28.58f);
    blockOffsetDual_[0] = -(55.56f * 4.0f + 28.58f);

    laserOffset_.resize(HS_LIDAR_L40_LASER_COUNT);
    laserOffset_[3] = -3.62f;
    laserOffset_[39] = -3.62f;
    laserOffset_[35] = -4.92f;
    laserOffset_[27] = -6.23f;
    laserOffset_[11] = -8.19f;
    laserOffset_[15] = -8.19f;
    laserOffset_[31] = -9.5f;
    laserOffset_[23] = -11.47f;
    laserOffset_[28] = -12.77f;
    laserOffset_[16] = -14.74f;
    laserOffset_[2] = -16.04f;
    laserOffset_[38] = -16.04f;
    laserOffset_[34] = -17.35f;
    laserOffset_[24] = -18.65f;
    laserOffset_[8] = -20.62f;
    laserOffset_[12] = -20.62f;
    laserOffset_[30] = -21.92f;
    laserOffset_[20] = -23.89f;
    laserOffset_[25] = -25.19f;
    laserOffset_[13] = -27.16f;
    laserOffset_[1] = -28.47f;
    laserOffset_[37] = -28.47f;
    laserOffset_[33] = -29.77f;
    laserOffset_[5] = -31.74f;
    laserOffset_[21] = -31.7447f;
    laserOffset_[9] = -33.71f;
    laserOffset_[29] = -35.01f;
    laserOffset_[17] = -36.98f;
    laserOffset_[22] = -38.95f;
    laserOffset_[10] = -40.91f;
    laserOffset_[0] = -42.22f;
    laserOffset_[36] = -42.22f;
    laserOffset_[32] = -43.52f;
    laserOffset_[4] = -45.49f;
    laserOffset_[18] = -45.49f;
    laserOffset_[6] = -47.46f;
    laserOffset_[26] = -48.76f;
    laserOffset_[14] = -50.73f;
    laserOffset_[19] = -52.7f;
    laserOffset_[7] = -54.67f;

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

std::optional<HS_LIDAR_Packet> Pandar40P::parseLidarPacket(const uint8_t *recvbuf, const int len) {
    if (len != HS_LIDAR_L40_PACKET_SIZE &&
        len != HS_LIDAR_L40_PACKET_SIZE + HS_LIDAR_L40_SEQ_NUM_SIZE) {
        std::cout << "Packet Size Mismatch (Pandar40): " << len << "\n";
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

    packet.timestamp = recvbuf[index] | recvbuf[index + 1] << 8 | recvbuf[index + 2] << 16 |
                       recvbuf[index + 3] << 24;
    index += HS_LIDAR_L40_TIMESTAMP_SIZE;

    packet.returnMode = recvbuf[index];

    index += HS_LIDAR_L40_FACTORY_INFO_SIZE + HS_LIDAR_L40_ECHO_SIZE;

    packet.UTC[0] = recvbuf[index];
    packet.UTC[1] = recvbuf[index + 1];
    packet.UTC[2] = recvbuf[index + 2];
    packet.UTC[3] = recvbuf[index + 3];
    packet.UTC[4] = recvbuf[index + 4];
    packet.UTC[5] = recvbuf[index + 5];

    index += HS_LIDAR_L40_UTC_TIME;

    return packet;
}
