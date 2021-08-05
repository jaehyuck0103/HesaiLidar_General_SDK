#include "pandar/pandarXT.h"

void PandarXT::Init() {

    if (m_sLidarType == "PandarXT-32") {
        elev_angle_map_.resize(32);
        azimuth_offset_map_.resize(32);
        laserOffset_.resize(32);
        for (int i = 0; i < HS_LIDAR_XT_UNIT_NUM; i++) {
            elev_angle_map_[i] = pandarXT_elev_angle_map[i];
            azimuth_offset_map_[i] = pandarXT_horizatal_azimuth_offset_map[i];
            laserOffset_[i] = laserXTOffset[i];
        }
    }

    if (m_sLidarType == "PandarXT-16") {
        elev_angle_map_.resize(16);
        azimuth_offset_map_.resize(16);
        laserOffset_.resize(16);
        for (int i = 0; i < HS_LIDAR_XT16_UNIT_NUM; i++) {
            elev_angle_map_[i] = pandarXT_elev_angle_map[i * 2];
            azimuth_offset_map_[i] = pandarXT_horizatal_azimuth_offset_map[i * 2];
            laserOffset_[i] = laserXTOffset[i * 2];
        }
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

int PandarXT::ParseData(HS_LIDAR_Packet *packet, const uint8_t *recvbuf, const int len) {
    if (len != HS_LIDAR_XT_PACKET_SIZE && len != HS_LIDAR_XT16_PACKET_SIZE) {
        std::cout << "Packet Size Mismatch (PandarXT): " << len << "\n";
        return -1;
    }

    int index = 0;
    // Parse 12 Bytes Header
    uint16_t sop = (recvbuf[index] & 0xff) << 8 | ((recvbuf[index + 1] & 0xff));
    uint8_t protocolVerMajor = recvbuf[index + 2] & 0xff;
    uint8_t nLasers = recvbuf[index + 6] & 0xff; // 32 or 16
    uint8_t nBlocks = recvbuf[index + 7] & 0xff; // 8
    uint8_t disUnit = recvbuf[index + 9] & 0xff; // 4 (mm)
    index += HS_LIDAR_XT_HEAD_SIZE;

    if (sop != 0xEEFF) {
        std::cout << "Error Start of Packet!\n";
        return -1;
    }
    if (protocolVerMajor != 0x06) {
        std::cout << "Error protocalVerMajor!\n";
        return -1;
    }
    if (nLasers != 0x20 && nLasers != 0x10) {
        std::cout << "Error nLasers!\n";
        return -1;
    }
    if (nBlocks != 0x08) {
        std::cout << "Error nBlocks!\n";
        return -1;
    }
    if (disUnit != 0x04) {
        std::cout << "Error disUnit!\n";
        return -1;
    }

    num_lasers_ = static_cast<int>(nLasers);

    packet->blocks.resize(nBlocks);
    for (int block = 0; block < nBlocks; block++) {
        packet->blocks[block].azimuth =
            (recvbuf[index] & 0xff) | ((recvbuf[index + 1] & 0xff) << 8);
        index += HS_LIDAR_XT_BLOCK_HEADER_AZIMUTH;

        packet->blocks[block].units.resize(nLasers);
        for (int unit = 0; unit < nLasers; unit++) {
            uint16_t unRange = (recvbuf[index] & 0xff) | ((recvbuf[index + 1] & 0xff) << 8);

            packet->blocks[block].units[unit].distance =
                static_cast<double>(unRange) * static_cast<double>(disUnit) / 1000.0;
            packet->blocks[block].units[unit].intensity = (recvbuf[index + 2] & 0xff);
            index += HS_LIDAR_XT_UNIT_SIZE;
        }
    }

    index += HS_LIDAR_XT_RESERVED_SIZE;

    packet->returnMode = recvbuf[index] & 0xff;

    index += HS_LIDAR_XT_ECHO_SIZE;
    index += HS_LIDAR_XT_ENGINE_VELOCITY;

    packet->UTC[0] = recvbuf[index] & 0xff;
    packet->UTC[1] = recvbuf[index + 1] & 0xff;
    packet->UTC[2] = recvbuf[index + 2] & 0xff;
    packet->UTC[3] = recvbuf[index + 3] & 0xff;
    packet->UTC[4] = recvbuf[index + 4] & 0xff;
    packet->UTC[5] = recvbuf[index + 5] & 0xff;
    index += HS_LIDAR_XT_UTC_SIZE;

    packet->timestamp = (recvbuf[index] & 0xff) | (recvbuf[index + 1] & 0xff) << 8 |
                        ((recvbuf[index + 2] & 0xff) << 16) | ((recvbuf[index + 3] & 0xff) << 24);

    return 0;
}

void PandarXT::CalcPointXYZIT(
    HS_LIDAR_Packet *pkt,
    int blockid,
    std::shared_ptr<PPointCloud> cld) {

    HS_LIDAR_Block *block = &pkt->blocks[blockid];

    tm tTm;
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
                point.timestamp =
                    point.timestamp +
                    (static_cast<double>(blockOffsetDual_[blockid] + laserOffset_[i]) /
                     1000000.0f);
            } else {
                point.timestamp =
                    point.timestamp +
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
