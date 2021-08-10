#include "pandar/pandarQT.h"

#include <iostream>

void PandarQT::Init() {

    blockOffsetSingle_.resize(HS_LIDAR_QT_BLOCK_NUMBER);
    blockOffsetSingle_[0] = 25.71f;
    blockOffsetSingle_[1] = 25.71f + 166.67f;
    blockOffsetSingle_[2] = 25.71f + 333.33f;
    blockOffsetSingle_[3] = 25.71f + 500.00f;

    blockOffsetDual_.resize(HS_LIDAR_QT_BLOCK_NUMBER);
    blockOffsetDual_[0] = 25.71f;
    blockOffsetDual_[1] = 25.71f;
    blockOffsetDual_[2] = 25.71f + 166.67f;
    blockOffsetDual_[3] = 25.71f + 166.67f;

    laserOffset_.resize(HS_LIDAR_QT_UNIT_NUM);
    laserOffset_[0] = 10.0f + 2.31f;
    laserOffset_[1] = 10.0f + 4.37f;
    laserOffset_[2] = 10.0f + 6.43f;
    laserOffset_[3] = 10.0f + 8.49f;
    laserOffset_[4] = 10.0f + 10.54f;
    laserOffset_[5] = 10.0f + 12.60f;
    laserOffset_[6] = 10.0f + 14.66f;
    laserOffset_[7] = 10.0f + 16.71f;
    laserOffset_[8] = 10.0f + 19.16f;
    laserOffset_[9] = 10.0f + 21.22f;
    laserOffset_[10] = 10.0f + 23.28f;
    laserOffset_[11] = 10.0f + 25.34f;
    laserOffset_[12] = 10.0f + 27.39f;
    laserOffset_[13] = 10.0f + 29.45f;
    laserOffset_[14] = 10.0f + 31.50f;
    laserOffset_[15] = 10.0f + 33.56f;

    laserOffset_[16] = 10.0f + 36.61f;
    laserOffset_[17] = 10.0f + 38.67f;
    laserOffset_[18] = 10.0f + 40.73f;
    laserOffset_[19] = 10.0f + 42.78f;
    laserOffset_[20] = 10.0f + 44.84f;
    laserOffset_[21] = 10.0f + 46.90f;
    laserOffset_[22] = 10.0f + 48.95f;
    laserOffset_[23] = 10.0f + 51.01f;
    laserOffset_[24] = 10.0f + 53.45f;
    laserOffset_[25] = 10.0f + 55.52f;
    laserOffset_[26] = 10.0f + 57.58f;
    laserOffset_[27] = 10.0f + 59.63f;
    laserOffset_[28] = 10.0f + 61.69f;
    laserOffset_[29] = 10.0f + 63.74f;
    laserOffset_[30] = 10.0f + 65.80f;
    laserOffset_[31] = 10.0f + 67.86f;

    laserOffset_[32] = 10.0f + 70.90f;
    laserOffset_[33] = 10.0f + 72.97f;
    laserOffset_[34] = 10.0f + 75.02f;
    laserOffset_[35] = 10.0f + 77.08f;
    laserOffset_[36] = 10.0f + 79.14f;
    laserOffset_[37] = 10.0f + 81.19f;
    laserOffset_[38] = 10.0f + 83.25f;
    laserOffset_[39] = 10.0f + 85.30f;
    laserOffset_[40] = 10.0f + 87.75f;
    laserOffset_[41] = 10.0f + 89.82f;
    laserOffset_[42] = 10.0f + 91.87f;
    laserOffset_[43] = 10.0f + 93.93f;
    laserOffset_[44] = 10.0f + 95.98f;
    laserOffset_[45] = 10.0f + 98.04f;
    laserOffset_[46] = 10.0f + 100.10f;
    laserOffset_[47] = 10.0f + 102.15f;

    laserOffset_[48] = 10.0f + 105.20f;
    laserOffset_[49] = 10.0f + 107.26f;
    laserOffset_[50] = 10.0f + 109.32f;
    laserOffset_[51] = 10.0f + 111.38f;
    laserOffset_[52] = 10.0f + 113.43f;
    laserOffset_[53] = 10.0f + 115.49f;
    laserOffset_[54] = 10.0f + 117.54f;
    laserOffset_[55] = 10.0f + 119.60f;
    laserOffset_[56] = 10.0f + 122.05f;
    laserOffset_[57] = 10.0f + 124.11f;
    laserOffset_[58] = 10.0f + 126.17f;
    laserOffset_[59] = 10.0f + 128.22f;
    laserOffset_[60] = 10.0f + 130.28f;
    laserOffset_[61] = 10.0f + 132.34f;
    laserOffset_[62] = 10.0f + 134.39f;
    laserOffset_[63] = 10.0f + 136.45f;

    elev_angle_map_ = {
        -52.121f, -49.785f, -47.577f, -45.477f, -43.465f, -41.528f, -39.653f, -37.831f,
        -36.055f, -34.320f, -32.619f, -30.950f, -29.308f, -27.690f, -26.094f, -24.517f,
        -22.964f, -21.420f, -19.889f, -18.372f, -16.865f, -15.368f, -13.880f, -12.399f,
        -10.925f, -9.457f,  -7.994f,  -6.535f,  -5.079f,  -3.626f,  -2.175f,  -0.725f,
        0.725f,   2.175f,   3.626f,   5.079f,   6.534f,   7.993f,   9.456f,   10.923f,
        12.397f,  13.877f,  15.365f,  16.861f,  18.368f,  19.885f,  21.415f,  22.959f,
        24.524f,  26.101f,  27.697f,  29.315f,  30.957f,  32.627f,  34.328f,  36.064f,
        37.840f,  39.662f,  41.537f,  43.475f,  45.487f,  47.587f,  49.795f,  52.133f};

    azimuth_offset_map_ = {8.736f,  8.314f,  7.964f,  7.669f,  7.417f,  7.198f,  7.007f,  6.838f,
                           6.688f,  6.554f,  6.434f,  6.326f,  6.228f,  6.140f,  6.059f,  5.987f,
                           -5.270f, -5.216f, -5.167f, -5.123f, -5.083f, -5.047f, -5.016f, -4.988f,
                           -4.963f, -4.942f, -4.924f, -4.910f, -4.898f, -4.889f, -4.884f, -4.881f,
                           5.493f,  5.496f,  5.502f,  5.512f,  5.525f,  5.541f,  5.561f,  5.584f,
                           5.611f,  5.642f,  5.676f,  5.716f,  5.759f,  5.808f,  5.862f,  5.921f,
                           -5.330f, -5.396f, -5.469f, -5.550f, -5.640f, -5.740f, -5.850f, -5.974f,
                           -6.113f, -6.269f, -6.447f, -6.651f, -6.887f, -7.163f, -7.493f, -7.892f};

    num_lasers_ = 64;
}

std::optional<HS_LIDAR_Packet> PandarQT::parseLidarPacket(const uint8_t *recvbuf, const int len) {
    if (len != HS_LIDAR_QT_PACKET_SIZE && len != HS_LIDAR_QT_PACKET_WITHOUT_UDPSEQ_SIZE) {
        std::cout << "Packet Size Mismatch (PandarQT): " << len << "\n";
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
    if (nLasers != 0x40) {
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

        block.units.resize(nLasers);
        for (auto &unit : block.units) {
            unit.rawDistance = recvbuf[index] | recvbuf[index + 1] << 8;
            unit.intensity = recvbuf[index + 2];
            index += HS_LIDAR_QT_UNIT_SIZE;
        }
    }

    index += HS_LIDAR_QT_RESERVED_SIZE;
    index += HS_LIDAR_QT_ENGINE_VELOCITY;

    packet.timestamp = recvbuf[index] | recvbuf[index + 1] << 8 | recvbuf[index + 2] << 16 |
                       recvbuf[index + 3] << 24;
    index += HS_LIDAR_QT_TIMESTAMP_SIZE;

    packet.returnMode = recvbuf[index];

    index += HS_LIDAR_QT_ECHO_SIZE;
    index += HS_LIDAR_QT_FACTORY_SIZE;

    packet.UTC[0] = recvbuf[index];
    packet.UTC[1] = recvbuf[index + 1];
    packet.UTC[2] = recvbuf[index + 2];
    packet.UTC[3] = recvbuf[index + 3];
    packet.UTC[4] = recvbuf[index + 4];
    packet.UTC[5] = recvbuf[index + 5];

    index += HS_LIDAR_QT_UTC_SIZE;

    return packet;
}
