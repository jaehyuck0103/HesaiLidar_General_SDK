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

#pragma once

#include <asio.hpp>

#include <functional>
#include <string>
#include <thread>
#include <vector>

using asio::ip::udp;

inline double parseUTC(const uint8_t *buf) {
    std::tm tTm;
    tTm.tm_year = buf[0] + 100; // years since 1900
    tTm.tm_mon = buf[1] - 1;    // years since 1900
    tTm.tm_mday = buf[2];
    tTm.tm_hour = buf[3];
    tTm.tm_min = buf[4];
    tTm.tm_sec = buf[5];
    tTm.tm_isdst = 0;

    return static_cast<double>(std::mktime(&tTm));
}

struct HS_LIDAR_Block {
    uint16_t azimuth;             // Azimuth = RealAzimuth * 100
    std::vector<uint8_t> payload; // num_lasers * 3 bytes (2bytes raw_distance + 1byte intensity)
};

struct HS_LIDAR_Packet {
    std::vector<HS_LIDAR_Block> blocks;
    double timestamp;
};

#define GPS_PACKET_SIZE (512)
#define GPS_PACKET_FLAG_SIZE (2)
#define GPS_PACKET_YEAR_SIZE (2)
#define GPS_PACKET_MONTH_SIZE (2)
#define GPS_PACKET_DAY_SIZE (2)
#define GPS_PACKET_HOUR_SIZE (2)
#define GPS_PACKET_MINUTE_SIZE (2)
#define GPS_PACKET_SECOND_SIZE (2)
#define GPS_ITEM_NUM (7)

struct PandarGPS {
    uint16_t flag;
    uint16_t year;
    uint16_t month;
    uint16_t day;
    uint16_t second;
    uint16_t minute;
    uint16_t hour;
    uint32_t fineTime;
};

class PandarGeneral_Internal {
  public:
    PandarGeneral_Internal(
        uint16_t lidar_port,
        uint16_t gps_port,
        std::function<void(const std::vector<uint8_t> &, double)> pcl_callback,
        std::function<void(double)> gps_callback,
        uint16_t start_azimuth,
        std::string lidar_type,
        std::string frame_id,
        std::string timestampType,
        int fps,
        bool dualReturnMode);

    virtual ~PandarGeneral_Internal();

    void Start();
    void Stop();

  private:
    void processGps(const std::vector<uint8_t> &packet);
    void processLidarPacket(const std::vector<uint8_t> &packet);

    std::optional<PandarGPS> parseGPS(const std::vector<uint8_t> &recvbuf);

    std::function<void(const std::vector<uint8_t> &cld, double timestamp)> pcl_callback_;
    std::function<void(double timestamp)> gps_callback_;

    std::string frame_id_;

    static constexpr float disUnit_ = 0.004; // 4mm

    std::vector<uint8_t> frame_buffer_;

  private: // asio
    asio::io_context io_context_;
    udp::socket lidarRcvSocket_;
    udp::socket gpsRcvSocket_;

    std::unique_ptr<std::thread> contextThr_;
    bool enableRecvThr_ = false;

    std::vector<uint8_t> lidarRcvBuffer_;
    std::vector<uint8_t> gpsRcvBuffer_;

    udp::endpoint lidarRemoteEndpoint_;
    udp::endpoint gpsRemoteEndpoint_;

    void rcvLidarHandler();
    void rcvGpsHandler();

  protected:
    std::vector<float> blockOffsetSingle_;
    std::vector<float> blockOffsetDual_;
    std::vector<float> laserOffset_;

    std::string m_sTimestampType;
    std::string m_sLidarType;

    bool isValidAzimuth(uint16_t azimuth) {
        if (azimuth < 36000 && azimuth % azimuth_res_ == 0) {
            return true;
        } else {
            return false;
        }
    }

    uint16_t start_azimuth_;
    uint16_t azimuth_res_;
    int fps_;

    virtual std::optional<HS_LIDAR_Packet>
    parseLidarPacket(const std::vector<uint8_t> &packet) = 0;

    int num_lasers_ = 0;
    bool dualReturnMode_;
};
