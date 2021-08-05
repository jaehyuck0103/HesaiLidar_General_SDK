#include "pandar/pandar64.h"

void Pandar64::Init() {

    blockOffsetSingle_.resize(HS_LIDAR_L64_BLOCK_NUMBER);
    blockOffsetSingle_[5] = 55.56f * 0.0f + 42.58f;
    blockOffsetSingle_[4] = 55.56f * 1.0f + 42.58f;
    blockOffsetSingle_[3] = 55.56f * 2.0f + 42.58f;
    blockOffsetSingle_[2] = 55.56f * 3.0f + 42.58f;
    blockOffsetSingle_[1] = 55.56f * 4.0f + 42.58f;
    blockOffsetSingle_[0] = 55.56f * 5.0f + 42.58f;

    blockOffsetDual_.resize(HS_LIDAR_L64_BLOCK_NUMBER);
    blockOffsetDual_[5] = 55.56f * 0.0f + 42.58f;
    blockOffsetDual_[4] = 55.56f * 0.0f + 42.58f;
    blockOffsetDual_[3] = 55.56f * 1.0f + 42.58f;
    blockOffsetDual_[2] = 55.56f * 1.0f + 42.58f;
    blockOffsetDual_[1] = 55.56f * 2.0f + 42.58f;
    blockOffsetDual_[0] = 55.56f * 2.0f + 42.58f;

    laserOffset_.resize(HS_LIDAR_L64_UNIT_NUM);
    laserOffset_[50] = 1.304f * 0.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[60] = 1.304f * 0.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[44] = 1.304f * 1.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[59] = 1.304f * 1.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[38] = 1.304f * 2.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[56] = 1.304f * 2.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[8] = 1.304f * 3.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[54] = 1.304f * 3.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[48] = 1.304f * 4.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[62] = 1.304f * 4.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[42] = 1.304f * 5.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[58] = 1.304f * 5.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[6] = 1.304f * 6.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[55] = 1.304f * 6.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[52] = 1.304f * 7.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[63] = 1.304f * 7.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[46] = 1.304f * 8.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[61] = 1.304f * 8.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[40] = 1.304f * 9.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[57] = 1.304f * 9.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[5] = 1.304f * 10.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[53] = 1.304f * 10.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[4] = 1.304f * 11.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[47] = 1.304f * 11.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[3] = 1.304f * 12.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[49] = 1.304f * 12.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[2] = 1.304f * 13.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[51] = 1.304f * 13.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[1] = 1.304f * 14.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[45] = 1.304f * 14.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[0] = 1.304f * 15.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[43] = 1.304f * 15.0f + 1.968f * 0.0f + 3.62f;
    laserOffset_[23] = 1.304f * 15.0f + 1.968f * 1.0f + 3.62f;
    laserOffset_[32] = 1.304f * 15.0f + 1.968f * 1.0f + 3.62f;
    laserOffset_[26] = 1.304f * 15.0f + 1.968f * 2.0f + 3.62f;
    laserOffset_[41] = 1.304f * 15.0f + 1.968f * 2.0f + 3.62f;
    laserOffset_[20] = 1.304f * 15.0f + 1.968f * 3.0f + 3.62f;
    laserOffset_[35] = 1.304f * 15.0f + 1.968f * 3.0f + 3.62f;
    laserOffset_[14] = 1.304f * 15.0f + 1.968f * 4.0f + 3.62f;
    laserOffset_[29] = 1.304f * 15.0f + 1.968f * 4.0f + 3.62f;
    laserOffset_[21] = 1.304f * 15.0f + 1.968f * 5.0f + 3.62f;
    laserOffset_[36] = 1.304f * 15.0f + 1.968f * 5.0f + 3.62f;
    laserOffset_[15] = 1.304f * 15.0f + 1.968f * 6.0f + 3.62f;
    laserOffset_[30] = 1.304f * 15.0f + 1.968f * 6.0f + 3.62f;
    laserOffset_[9] = 1.304f * 15.0f + 1.968f * 7.0f + 3.62f;
    laserOffset_[24] = 1.304f * 15.0f + 1.968f * 7.0f + 3.62f;
    laserOffset_[18] = 1.304f * 15.0f + 1.968f * 8.0f + 3.62f;
    laserOffset_[33] = 1.304f * 15.0f + 1.968f * 8.0f + 3.62f;
    laserOffset_[12] = 1.304f * 15.0f + 1.968f * 9.0f + 3.62f;
    laserOffset_[27] = 1.304f * 15.0f + 1.968f * 9.0f + 3.62f;
    laserOffset_[19] = 1.304f * 15.0f + 1.968f * 10.0f + 3.62f;
    laserOffset_[34] = 1.304f * 15.0f + 1.968f * 10.0f + 3.62f;
    laserOffset_[13] = 1.304f * 15.0f + 1.968f * 11.0f + 3.62f;
    laserOffset_[28] = 1.304f * 15.0f + 1.968f * 11.0f + 3.62f;
    laserOffset_[7] = 1.304f * 15.0f + 1.968f * 12.0f + 3.62f;
    laserOffset_[22] = 1.304f * 15.0f + 1.968f * 12.0f + 3.62f;
    laserOffset_[16] = 1.304f * 15.0f + 1.968f * 13.0f + 3.62f;
    laserOffset_[31] = 1.304f * 15.0f + 1.968f * 13.0f + 3.62f;
    laserOffset_[10] = 1.304f * 15.0f + 1.968f * 14.0f + 3.62f;
    laserOffset_[25] = 1.304f * 15.0f + 1.968f * 14.0f + 3.62f;
    laserOffset_[17] = 1.304f * 15.0f + 1.968f * 15.0f + 3.62f;
    laserOffset_[37] = 1.304f * 15.0f + 1.968f * 15.0f + 3.62f;
    laserOffset_[11] = 1.304f * 15.0f + 1.968f * 16.0f + 3.62f;
    laserOffset_[39] = 1.304f * 15.0f + 1.968f * 16.0f + 3.62f;

    elev_angle_map_ = {
        14.882f, 11.032f, 8.059f,   5.057f,   3.04f,    2.028f,  1.86f,    1.688f,
        1.522f,  1.351f,  1.184f,   1.013f,   0.846f,   0.675f,  0.508f,   0.337f,
        0.169f,  0.0f,    -0.169f,  -0.337f,  -0.508f,  -0.675f, -0.845f,  -1.013f,
        -1.184f, -1.351f, -1.522f,  -1.688f,  -1.86f,   -2.028f, -2.198f,  -2.365f,
        -2.536f, -2.7f,   -2.873f,  -3.04f,   -3.21f,   -3.375f, -3.548f,  -3.712f,
        -3.884f, -4.05f,  -4.221f,  -4.385f,  -4.558f,  -4.72f,  -4.892f,  -5.057f,
        -5.229f, -5.391f, -5.565f,  -5.726f,  -5.898f,  -6.061f, -7.063f,  -8.059f,
        -9.06f,  -9.885f, -11.032f, -12.006f, -12.974f, -13.93f, -18.889f, -24.897f};

    azimuth_offset_map_ = {-1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, 1.042f,  3.125f,
                           5.208f,  -5.208f, -3.125f, -1.042f, 1.042f,  3.125f,  5.208f,  -5.208f,
                           -3.125f, -1.042f, 1.042f,  3.125f,  5.208f,  -5.208f, -3.125f, -1.042f,
                           1.042f,  3.125f,  5.208f,  -5.208f, -3.125f, -1.042f, 1.042f,  3.125f,
                           5.208f,  -5.208f, -3.125f, -1.042f, 1.042f,  3.125f,  5.208f,  -5.208f,
                           -3.125f, -1.042f, 1.042f,  3.125f,  5.208f,  -5.208f, -3.125f, -1.042f,
                           1.042f,  3.125f,  5.208f,  -5.208f, -3.125f, -1.042f, -1.042f, -1.042f,
                           -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f};

    num_lasers_ = 64;
}

int Pandar64::ParseData(HS_LIDAR_Packet *packet, const uint8_t *recvbuf, const int len) {
    if (len != HS_LIDAR_L64_PACKET_SIZE && len != HS_LIDAR_L64_PACKET_WITHOUT_UDPSEQ_SIZE) {
        std::cout << "Packet Size Mismatch (Pandar64): " << len << "\n";
        return -1;
    }

    int index = 0;

    // Parse 8 Bytes Header
    uint16_t sop = (recvbuf[index] & 0xff) << 8 | ((recvbuf[index + 1] & 0xff));
    uint8_t nLasers = recvbuf[index + 2] & 0xff; // 64
    uint8_t nBlocks = recvbuf[index + 3] & 0xff; // 6
    uint8_t disUnit = recvbuf[index + 5] & 0xff; // 4(mm)
    index += HS_LIDAR_L64_HEAD_SIZE;

    if (sop != 0xEEFF) {
        std::cout << "Error Start of Packet!\n";
        return -1;
    }
    if (nLasers != 0x40) {
        std::cout << "Error nLasers!\n";
        return -1;
    }
    if (nBlocks != 0x06) {
        std::cout << "Error nBlocks!\n";
        return -1;
    }
    if (disUnit != 0x04) {
        std::cout << "Error disUnit!\n";
        return -1;
    }

    packet->blocks.resize(nBlocks);
    for (int block = 0; block < nBlocks; block++) {
        packet->blocks[block].azimuth =
            (recvbuf[index] & 0xff) | ((recvbuf[index + 1] & 0xff) << 8);
        index += HS_LIDAR_L64_BLOCK_HEADER_AZIMUTH;

        packet->blocks[block].units.resize(nLasers);
        for (int unit = 0; unit < nLasers; unit++) {
            uint16_t unRange = (recvbuf[index] & 0xff) | ((recvbuf[index + 1] & 0xff) << 8);

            packet->blocks[block].units[unit].distance =
                static_cast<double>(unRange) * static_cast<double>(disUnit) / 1000.0;
            packet->blocks[block].units[unit].intensity = (recvbuf[index + 2] & 0xff);
            index += HS_LIDAR_L64_UNIT_SIZE;
        }
    }

    index += HS_LIDAR_L64_RESERVED_SIZE;
    index += HS_LIDAR_L64_ENGINE_VELOCITY;

    packet->timestamp = (recvbuf[index] & 0xff) | (recvbuf[index + 1] & 0xff) << 8 |
                        ((recvbuf[index + 2] & 0xff) << 16) | ((recvbuf[index + 3] & 0xff) << 24);
    index += HS_LIDAR_L64_TIMESTAMP_SIZE;

    packet->returnMode = recvbuf[index] & 0xff;
    index += HS_LIDAR_L64_RETURN_MODE_SIZE;
    index += HS_LIDAR_L64_FACTORY_SIZE;

    packet->UTC[0] = recvbuf[index] & 0xff;
    packet->UTC[1] = recvbuf[index + 1] & 0xff;
    packet->UTC[2] = recvbuf[index + 2] & 0xff;
    packet->UTC[3] = recvbuf[index + 3] & 0xff;
    packet->UTC[4] = recvbuf[index + 4] & 0xff;
    packet->UTC[5] = recvbuf[index + 5] & 0xff;
    index += HS_LIDAR_L64_TIME_SIZE;

    return 0;
}

void Pandar64::CalcPointXYZIT(
    HS_LIDAR_Packet *pkt,
    int blockid,
    std::shared_ptr<PPointCloud> cld) {

    HS_LIDAR_Block *block = &pkt->blocks[blockid];

    tm tTm;
    // UTC's year only include 0 - 99 year , which indicate 2000 to 2099.
    // and mktime's year start from 1900 which is 0. so we need add 100 year.
    tTm.tm_year = pkt->UTC[0] + 100;

    // in case of time error
    if (tTm.tm_year >= 200) {
        tTm.tm_year -= 100;
    }

    // UTC's month start from 1, but mktime only accept month from 0.
    tTm.tm_mon = pkt->UTC[1] - 1;
    tTm.tm_mday = pkt->UTC[2];
    tTm.tm_hour = pkt->UTC[3];
    tTm.tm_min = pkt->UTC[4];
    tTm.tm_sec = pkt->UTC[5];
    tTm.tm_isdst = 0;

    double unix_second = static_cast<double>(mktime(&tTm) + tz_second_);

    for (int i = 0; i < num_lasers_; ++i) {
        /* for all the units in a block */
        HS_LIDAR_Unit &unit = block->units[i];
        PPoint point;

        /* skip wrong points */
        if (unit.distance <= 0.1 || unit.distance > 200.0) {
            continue;
        }

        double xyDistance = unit.distance * cosf(degreeToRadian(elev_angle_map_[i]));
        point.x = static_cast<float>(
            xyDistance *
            sinf(degreeToRadian(
                azimuth_offset_map_[i] + (static_cast<double>(block->azimuth)) / 100.0)));
        point.y = static_cast<float>(
            xyDistance *
            cosf(degreeToRadian(
                azimuth_offset_map_[i] + (static_cast<double>(block->azimuth)) / 100.0)));
        point.z = static_cast<float>(unit.distance * sinf(degreeToRadian(elev_angle_map_[i])));

        point.intensity = unit.intensity;

        if ("realtime" == m_sTimestampType) {
            point.timestamp = m_dPktTimestamp;
        } else {
            point.timestamp = unix_second + (static_cast<double>(pkt->timestamp)) / 1000000.0;

            if (pkt->returnMode >= 0x39) {
                // dual return, block 0&1 (2&3 , 4*5 ...)'s timestamp is the same.
                point.timestamp =
                    point.timestamp -
                    (static_cast<double>(blockOffsetDual_[blockid] + laserOffset_[i]) /
                     1000000.0f);
            } else {
                point.timestamp =
                    point.timestamp -
                    (static_cast<double>(blockOffsetSingle_[blockid] + laserOffset_[i]) /
                     1000000.0f);
            }
        }

        point.ring = i;

        if (pcl_type_) {
            PointCloudList[i].push_back(point);
        } else {
            cld->push_back(point);
        }
    }
}
