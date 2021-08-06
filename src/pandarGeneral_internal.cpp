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
#include "pandar/log.h"

#include <sstream>

PandarGeneral_Internal::PandarGeneral_Internal(
    uint16_t lidar_port,
    uint16_t gps_port,
    std::function<void(std::shared_ptr<PPointCloud>, double)> pcl_callback,
    std::function<void(double)> gps_callback,
    uint16_t start_angle,
    int tz,
    int pcl_type,
    std::string lidar_type,
    std::string frame_id,
    std::string timestampType) {

    pthread_mutex_init(&lidar_lock_, NULL);
    sem_init(&lidar_sem_, 0, 0);

    input_ = std::make_unique<Input>(lidar_port, gps_port);

    start_angle_ = start_angle;
    pcl_callback_ = pcl_callback;
    gps_callback_ = gps_callback;
    m_sLidarType = lidar_type;
    frame_id_ = frame_id;
    tz_second_ = tz * 3600;
    pcl_type_ = pcl_type;
    m_sTimestampType = timestampType;

    InitLUT();
}

PandarGeneral_Internal::PandarGeneral_Internal(
    std::string pcap_path,
    std::function<void(std::shared_ptr<PPointCloud>, double)> pcl_callback,
    uint16_t start_angle,
    int tz,
    int pcl_type,
    std::string lidar_type,
    std::string frame_id,
    std::string timestampType) {

    pthread_mutex_init(&lidar_lock_, NULL);
    sem_init(&lidar_sem_, 0, 0);

    pcap_reader_ = std::make_unique<PcapReader>(pcap_path, lidar_type);

    start_angle_ = start_angle;
    pcl_callback_ = pcl_callback;
    gps_callback_ = NULL;
    m_sLidarType = lidar_type;
    frame_id_ = frame_id;
    tz_second_ = tz * 3600;
    pcl_type_ = pcl_type;
    m_sTimestampType = timestampType;

    InitLUT();
}

PandarGeneral_Internal::~PandarGeneral_Internal() {
    Stop();
    sem_destroy(&lidar_sem_);
    pthread_mutex_destroy(&lidar_lock_);
}

void PandarGeneral_Internal::InitLUT() {
    for (uint16_t rotIndex = 0; rotIndex < ROTATION_MAX_UNITS; ++rotIndex) {
        float rotation = degreeToRadian(0.01 * static_cast<double>(rotIndex));
        cos_lookup_table_[rotIndex] = cosf(rotation);
        sin_lookup_table_[rotIndex] = sinf(rotation);
    }
}

/**
 * @brief load the correction file
 * @param file The path of correction file
 */
int PandarGeneral_Internal::LoadCorrectionFile(std::string correction_content) {
    // LOG_FUNC();
    // LOG_D("stirng:[%s]",correction_content.c_str());
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

void PandarGeneral_Internal::ResetStartAngle(uint16_t start_angle) { start_angle_ = start_angle; }

void PandarGeneral_Internal::Start() {
    // LOG_FUNC();
    Stop();
    enable_lidar_recv_thr_ = true;
    enable_lidar_process_thr_ = true;
    lidar_process_thr_ =
        std::make_unique<std::thread>(&PandarGeneral_Internal::ProcessLidarPacket, this);

    if (pcap_reader_) {
        pcap_reader_->start([this](auto a, auto b, auto c) { FillPacket(a, b, c); });
    } else {
        lidar_recv_thr_ = std::make_unique<std::thread>(&PandarGeneral_Internal::RecvTask, this);
    }
}

void PandarGeneral_Internal::Stop() {
    enable_lidar_recv_thr_ = false;
    enable_lidar_process_thr_ = false;

    if (lidar_recv_thr_) {
        lidar_recv_thr_->join();
    }
    if (lidar_process_thr_) {
        lidar_process_thr_->join();
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
            PushLidarData(pkt);
        }
    }
}

void PandarGeneral_Internal::FillPacket(const uint8_t *buf, const int len, double timestamp) {
    if (len != GPS_PACKET_SIZE) {
        PandarPacket pkt;
        memcpy(pkt.data, buf, len);
        pkt.size = len;
        pkt.stamp = timestamp;
        PushLidarData(pkt);
    }
}

void PandarGeneral_Internal::ProcessLidarPacket() {

    std::shared_ptr<PPointCloud> outMsg(new PPointCloud());
    int last_azimuth = 0;

    while (enable_lidar_process_thr_) {

        timespec ts;
        if (clock_gettime(CLOCK_REALTIME, &ts) == -1) {
            std::cout << "get time error" << std::endl;
        }

        ts.tv_sec += 1;
        if (sem_timedwait(&lidar_sem_, &ts) == -1) {
            continue;
        }

        pthread_mutex_lock(&lidar_lock_);
        PandarPacket packet = lidar_packets_.front();
        lidar_packets_.pop_front();
        pthread_mutex_unlock(&lidar_lock_);

        HS_LIDAR_Packet pkt;
        int ret = ParseData(&pkt, packet.data, packet.size);
        if (ret != 0) {
            continue;
        }

        for (size_t i = 0; i < pkt.blocks.size(); ++i) {

            int curr_azimuth = static_cast<int>(pkt.blocks[i].azimuth);

            int azimuthGap = curr_azimuth - last_azimuth;
            if (azimuthGap < 0) {
                azimuthGap += 36000;
            }

            if (last_azimuth != curr_azimuth && azimuthGap < 600 /* 6 degree*/) {
                if ((last_azimuth > curr_azimuth && start_angle_ <= curr_azimuth) ||
                    (last_azimuth < start_angle_ && start_angle_ <= curr_azimuth)) {

                    if (pcl_callback_ &&
                        (outMsg->points.size() > 0 || PointCloudList[0].size() > 0)) {
                        EmitBackMessege(num_lasers_, outMsg);
                        outMsg.reset(new PPointCloud());
                    }
                }
            }
            CalcPointXYZIT(pkt, i, outMsg, packet.stamp);
            last_azimuth = curr_azimuth;
        }

        outMsg->header.frame_id = frame_id_;
        outMsg->height = 1;
    }
}

void PandarGeneral_Internal::PushLidarData(PandarPacket packet) {
    pthread_mutex_lock(&lidar_lock_);
    lidar_packets_.push_back(packet);
    sem_post(&lidar_sem_);
    pthread_mutex_unlock(&lidar_lock_);
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

void PandarGeneral_Internal::EmitBackMessege(
    char chLaserNumber,
    std::shared_ptr<PPointCloud> cld) {

    if (pcl_type_) {
        for (int i = 0; i < chLaserNumber; i++) {
            for (size_t j = 0; j < PointCloudList[i].size(); j++) {
                cld->push_back(PointCloudList[i][j]);
            }
        }
    }

    pcl_callback_(cld, cld->points[0].timestamp);
    // cld.reset(new PPointCloud());

    if (pcl_type_) {
        for (int i = 0; i < 128; i++) {
            PointCloudList[i].clear();
        }
    }
}

void PandarGeneral_Internal::CalcPointXYZIT(
    const HS_LIDAR_Packet &pkt,
    int blockid,
    std::shared_ptr<PPointCloud> cld,
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
        PPoint point;

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

        point.ring = i;

        if (pcl_type_) {
            PointCloudList[i].push_back(point);
        } else {
            cld->push_back(point);
        }
    }
}
