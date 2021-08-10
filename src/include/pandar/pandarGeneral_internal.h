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

#include "input.h"
#include "pcap_reader.h"

#include "pandarGeneral_sdk/point_types.h"

#include <list>
#include <string>
#include <thread>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

inline constexpr double degreeToRadian(double degree) { return degree * M_PI / 180; }

struct HS_LIDAR_Unit {
    double distance;
    uint8_t intensity;
};

struct HS_LIDAR_Block {
    uint16_t azimuth; // Azimuth = RealAzimuth * 100
    std::vector<HS_LIDAR_Unit> units;
};

struct HS_LIDAR_Packet {
    std::vector<HS_LIDAR_Block> blocks;
    std::array<uint8_t, 6> UTC; // Year, month, date, hour, minitiue, second
    uint32_t timestamp;         // 0 ~ 1000000 us (1s)
    uint8_t returnMode;
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
        std::string pcap_path,
        std::function<void(std::vector<PointXYZIT>, double)> pcl_callback,
        std::function<void(double)> gps_callback,
        uint16_t start_angle,
        int tz,
        std::string lidar_type,
        std::string frame_id,
        std::string timestampType);

    virtual ~PandarGeneral_Internal();

    bool updateAngleCorrection(std::string correction_content);

    void Start();
    void Stop();

  private:
    void RecvTask();
    void ProcessGps(const PandarGPS &gpsMsg);
    void ProcessLidarPacket(const PandarPacket &packet);

    int ParseGPS(PandarGPS *packet, const uint8_t *recvbuf, const int size);

    void FillPacket(const uint8_t *buf, const int len, double timestamp);

    std::unique_ptr<std::thread> lidar_recv_thr_;
    bool enable_lidar_recv_thr_ = false;
    int start_angle_;

    std::list<PandarPacket> lidar_packets_;

    std::unique_ptr<Input> input_;
    std::function<void(std::vector<PointXYZIT> cld, double timestamp)> pcl_callback_;
    std::function<void(double timestamp)> gps_callback_;

    std::string frame_id_;
    std::unique_ptr<PcapReader> pcap_reader_;

  protected:
    std::vector<float> elev_angle_map_;
    std::vector<float> azimuth_offset_map_;

    std::vector<float> blockOffsetSingle_;
    std::vector<float> blockOffsetDual_;
    std::vector<float> laserOffset_;

    int tz_second_;
    std::string m_sTimestampType;
    std::string m_sLidarType;

    virtual std::optional<HS_LIDAR_Packet>
    parseLidarPacket(const uint8_t *recvbuf, const int len) = 0;

    void CalcPointXYZIT(const HS_LIDAR_Packet &pkt, int blockid, double pktRcvTimestamp);

    int num_lasers_ = 0;

    std::vector<PointXYZIT> PointCloudList;
};
