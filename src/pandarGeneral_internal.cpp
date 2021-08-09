/******************************************************************************
 * Copyright 2019 The Hesai Technology Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "pandar/pandarGeneral_internal.h"

#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>

PandarGeneral_Internal::PandarGeneral_Internal(
    uint16_t lidar_port,
    uint16_t gps_port,
    std::function<void(std::vector<PointXYZIT>, double)> pcl_callback,
    std::function<void(double)> gps_callback,
    uint16_t start_angle,
    int tz,
    std::string lidar_type,
    std::string frame_id,
    std::string timestampType) {

    input_ = std::make_unique<Input>(lidar_port, gps_port);

    start_angle_ = start_angle;
    pcl_callback_ = pcl_callback;
    gps_callback_ = gps_callback;
    m_sLidarType = lidar_type;
    frame_id_ = frame_id;
    tz_second_ = tz * 3600;
    m_sTimestampType = timestampType;

    InitLUT();
}

PandarGeneral_Internal::PandarGeneral_Internal(
    std::string pcap_path,
    std::function<void(std::vector<PointXYZIT>, double)> pcl_callback,
    uint16_t start_angle,
    int tz,
    std::string lidar_type,
    std::string frame_id,
    std::string timestampType) {

    pcap_reader_ = std::make_unique<PcapReader>(pcap_path, lidar_type);

    start_angle_ = start_angle;
    pcl_callback_ = pcl_callback;
    gps_callback_ = NULL;
    m_sLidarType = lidar_type;
    frame_id_ = frame_id;
    tz_second_ = tz * 3600;
    m_sTimestampType = timestampType;

    InitLUT();
}

PandarGeneral_Internal::~PandarGeneral_Internal() { Stop(); }

void PandarGeneral_Internal::InitLUT() {
    for (uint16_t rotIndex = 0; rotIndex < ROTATION_MAX_UNITS; ++rotIndex) {
        float rotation = degreeToRadian(0.01 * static_cast<double>(rotIndex));
        cos_lookup_table_[rotIndex] = cosf(rotation);
        sin_lookup_table_[rotIndex] = sinf(rotation);
    }
}

int PandarGeneral_Internal::LoadCorrectionFile(std::string correction_content) {
    std::istringstream ifs(correction_content);

    std::string line;
    if (std::getline(ifs, line)) { // first line "Laser id,Elevation,Azimuth"
        std::cout << "Parse Lidar Correction..." << std::endl;
    }

    constexpr int HS_LIDAR_L64_UNIT_NUM = 64; // temp
    double azimuthOffset[HS_LIDAR_L64_UNIT_NUM];
    double elev_angle[HS_LIDAR_L64_UNIT_NUM];

    int lineCounter = 0;
    while (std::getline(ifs, line)) {
        // correction file has 3 columns, min length is len(0,0,0)
        if (line.length() < 5) {
            break;
        }
        lineCounter++;

        int lineId = 0;
        double elev, azimuth;

        std::stringstream ss(line);
        std::string subline;
        std::getline(ss, subline, ',');
        std::stringstream(subline) >> lineId;
        std::getline(ss, subline, ',');
        std::stringstream(subline) >> elev;
        std::getline(ss, subline, ',');
        std::stringstream(subline) >> azimuth;

        if (lineId != lineCounter) {
            break;
        }

        elev_angle[lineId - 1] = elev;
        azimuthOffset[lineId - 1] = azimuth;
    }

    for (int i = 0; i < lineCounter; ++i) {
        /* for all the laser offset */
        elev_angle_map_[i] = elev_angle[i];
        azimuth_offset_map_[i] = azimuthOffset[i];
    }

    return 0;
}

void PandarGeneral_Internal::Start() {
    Stop();
    enable_lidar_recv_thr_ = true;

    if (pcap_reader_) {
        pcap_reader_->start([this](auto a, auto b, auto c) { FillPacket(a, b, c); });
    } else {
        lidar_recv_thr_ = std::make_unique<std::thread>(&PandarGeneral_Internal::RecvTask, this);
    }
}

void PandarGeneral_Internal::Stop() {
    enable_lidar_recv_thr_ = false;

    if (lidar_recv_thr_) {
        lidar_recv_thr_->join();
    }
    if (pcap_reader_) {
        pcap_reader_->stop();
    }

    return;
}

void PandarGeneral_Internal::RecvTask() {
    while (enable_lidar_recv_thr_) {
        PandarPacket pkt;
        int rc = input_->getPacket(&pkt);
        if (rc == -1) {
            continue;
        }

        if (pkt.size == GPS_PACKET_SIZE) {
            PandarGPS gpsMsg;
            int ret = ParseGPS(&gpsMsg, pkt.data, pkt.size);
            if (ret == 0) {
                ProcessGps(gpsMsg);
            }
        } else {
            ProcessLidarPacket(pkt);
        }
    }
}

void PandarGeneral_Internal::FillPacket(const uint8_t *buf, const int len, double timestamp) {
    if (len != GPS_PACKET_SIZE) {
        PandarPacket pkt;
        memcpy(pkt.data, buf, len);
        pkt.size = len;
        pkt.stamp = timestamp;
        ProcessLidarPacket(pkt);
    }
}

void PandarGeneral_Internal::ProcessLidarPacket(const PandarPacket &packet) {

    static int last_azimuth = 0;

    HS_LIDAR_Packet pkt;
    int ret = ParseData(&pkt, packet.data, packet.size);
    if (ret != 0) {
        return;
    }

    for (size_t i = 0; i < pkt.blocks.size(); ++i) {

        int curr_azimuth = static_cast<int>(pkt.blocks[i].azimuth);

        if ((last_azimuth < start_angle_ && start_angle_ <= curr_azimuth) ||
            (start_angle_ <= curr_azimuth && curr_azimuth < last_azimuth) ||
            (curr_azimuth < last_azimuth && last_azimuth < start_angle_)) {

            if (pcl_callback_ && PointCloudList.size() > 0) {
                pcl_callback_(PointCloudList, PointCloudList[0].timestamp);
                PointCloudList.clear();
            }
        }

        CalcPointXYZIT(pkt, i, packet.stamp);
        last_azimuth = curr_azimuth;
    }
}

void PandarGeneral_Internal::ProcessGps(const PandarGPS &gpsMsg) {
    tm t;
    t.tm_sec = gpsMsg.second;
    t.tm_min = gpsMsg.minute;

    t.tm_hour = gpsMsg.hour;
    t.tm_mday = gpsMsg.day;

    // UTC's month start from 1, but mktime only accept month from 0.
    t.tm_mon = gpsMsg.month - 1;
    // UTC's year only include 0 - 99 year , which indicate 2000 to 2099.
    // and mktime's year start from 1900 which is 0. so we need add 100 year.
    t.tm_year = gpsMsg.year + 100;
    t.tm_isdst = 0;

    if (gps_callback_) {
        gps_callback_(static_cast<double>(mktime(&t) + tz_second_));
    }
}

int PandarGeneral_Internal::ParseGPS(PandarGPS *packet, const uint8_t *recvbuf, const int size) {
    if (size != GPS_PACKET_SIZE) {
        return -1;
    }
    int index = 0;
    packet->flag = (recvbuf[index] & 0xff) | ((recvbuf[index + 1] & 0xff) << 8);
    index += GPS_PACKET_FLAG_SIZE;
    packet->year = (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
    index += GPS_PACKET_YEAR_SIZE;
    packet->month = (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
    index += GPS_PACKET_MONTH_SIZE;
    packet->day = (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
    index += GPS_PACKET_DAY_SIZE;
    packet->second = (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
    index += GPS_PACKET_SECOND_SIZE;
    packet->minute = (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
    index += GPS_PACKET_MINUTE_SIZE;
    packet->hour = (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
    index += GPS_PACKET_HOUR_SIZE;
    packet->fineTime = (recvbuf[index] & 0xff) | (recvbuf[index + 1] & 0xff) << 8 |
                       ((recvbuf[index + 2] & 0xff) << 16) | ((recvbuf[index + 3] & 0xff) << 24);

    return 0;
}

void PandarGeneral_Internal::CalcPointXYZIT(
    const HS_LIDAR_Packet &pkt,
    int blockid,
    double pktRcvTimestamp) {

    const HS_LIDAR_Block &block = pkt.blocks[blockid];

    tm tTm;
    // UTC's year only include 0 - 99 year , which indicate 2000 to 2099.
    // and mktime's year start from 1900 which is 0. so we need add 100 year.
    tTm.tm_year = pkt.UTC[0] + 100;

    // in case of time error
    if (tTm.tm_year >= 200) {
        tTm.tm_year -= 100;
    }

    // UTC's month start from 1, but mktime only accept month from 0.
    tTm.tm_mon = pkt.UTC[1] - 1;
    tTm.tm_mday = pkt.UTC[2];
    tTm.tm_hour = pkt.UTC[3];
    tTm.tm_min = pkt.UTC[4];
    tTm.tm_sec = pkt.UTC[5];
    tTm.tm_isdst = 0;

    double unix_second = static_cast<double>(mktime(&tTm) + tz_second_);

    for (int i = 0; i < num_lasers_; ++i) {
        /* for all the units in a block */
        const HS_LIDAR_Unit &unit = block.units[i];
        PointXYZIT point;

        /* skip wrong points */
        if (unit.distance <= 0.1 || unit.distance > 200.0) {
            continue;
        }

        double xyDistance = unit.distance * cosf(degreeToRadian(elev_angle_map_[i]));
        point.x = static_cast<float>(
            xyDistance *
            sinf(degreeToRadian(
                azimuth_offset_map_[i] + (static_cast<double>(block.azimuth)) / 100.0)));
        point.y = static_cast<float>(
            xyDistance *
            cosf(degreeToRadian(
                azimuth_offset_map_[i] + (static_cast<double>(block.azimuth)) / 100.0)));
        point.z = static_cast<float>(unit.distance * sinf(degreeToRadian(elev_angle_map_[i])));

        point.intensity = unit.intensity;

        if ("realtime" == m_sTimestampType) {
            point.timestamp = pktRcvTimestamp;
        } else {
            point.timestamp = unix_second + (static_cast<double>(pkt.timestamp)) / 1000000.0;

            if (pkt.returnMode >= 0x39) {
                // dual return, block 0&1 (2&3 , 4*5 ...)'s timestamp is the same.
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

        PointCloudList.push_back(point);
    }
}
