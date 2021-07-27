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
#include "pandar40.h"
#include "pandar64.h"
#include "pandarQT.h"
#include "pandarXT.h"
#include "pcap_reader.h"
#include "point_types.h"

#include <semaphore.h>
#include <thread>

#include <list>
#include <string>

/**
 *
 */
#define GPS_PACKET_SIZE (512)
#define GPS_PACKET_FLAG_SIZE (2)
#define GPS_PACKET_YEAR_SIZE (2)
#define GPS_PACKET_MONTH_SIZE (2)
#define GPS_PACKET_DAY_SIZE (2)
#define GPS_PACKET_HOUR_SIZE (2)
#define GPS_PACKET_MINUTE_SIZE (2)
#define GPS_PACKET_SECOND_SIZE (2)
#define GPS_ITEM_NUM (7)

#define HesaiLidarSDK_DEFAULT_LIDAR_RECV_PORT 8080
#define HesaiLidarSDK_DEFAULT_GPS_RECV_PORT 10110

#define MAX_LASER_NUM (256)

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

#define ROTATION_MAX_UNITS (36001)

class PandarGeneral_Internal {
  public:
    /**
     * @brief Constructor
     * @param device_ip  				The ip of the device
     *        lidar_port 				The port number of lidar data
     *        gps_port   				The port number of gps data
     *        pcl_callback      The callback of PCL data structure
     *        gps_callback      The callback of GPS structure
     *        type       				The device type
     */
    PandarGeneral_Internal(
        std::string device_ip,
        uint16_t lidar_port,
        uint16_t gps_port,
        std::function<void(std::shared_ptr<PPointCloud>, double)> pcl_callback,
        std::function<void(double)> gps_callback,
        uint16_t start_angle,
        int tz,
        int pcl_type,
        std::string lidar_type,
        std::string frame_id,
        std::string timestampType);

    /**
     * @brief Constructor
     * @param pcap_path         The path of pcap file
     *        pcl_callback      The callback of PCL data structure
     *        start_angle       The start angle of frame
     *        tz                The timezone
     *        pcl_type          Structured Pointcloud
     *        frame_id          The frame id of pcd
     */
    PandarGeneral_Internal(
        std::string pcap_path,
        std::function<void(std::shared_ptr<PPointCloud>, double)> pcl_callback,
        uint16_t start_angle,
        int tz,
        int pcl_type,
        std::string lidar_type,
        std::string frame_id,
        std::string timestampType); // the default timestamp type is LiDAR time
    ~PandarGeneral_Internal();

    /**
     * @brief load the correction file
     * @param correction The path of correction file
     */
    int LoadCorrectionFile(std::string correction);

    /**
     * @brief load the correction file
     * @param angle The start angle
     */
    void ResetStartAngle(uint16_t start_angle);

    void Start();
    void Stop();

  private:
    void Init();
    void RecvTask();
    void ProcessGps(const PandarGPS &gpsMsg);
    void ProcessLidarPacket();
    void PushLidarData(PandarPacket packet);

    int ParseL40Data(HS_LIDAR_L40_Packet *packet, const uint8_t *buf, const int len);
    int ParseL64Data(HS_LIDAR_L64_Packet *packet, const uint8_t *recvbuf, const int len);
    int ParseQTData(HS_LIDAR_QT_Packet *packet, const uint8_t *recvbuf, const int len);
    int ParseXTData(HS_LIDAR_XT_Packet *packet, const uint8_t *recvbuf, const int len);
    int ParseGPS(PandarGPS *packet, const uint8_t *recvbuf, const int size);

    void
    CalcL40PointXYZIT(HS_LIDAR_L40_Packet *pkt, int blockid, std::shared_ptr<PPointCloud> cld);
    void CalcL64PointXYZIT(
        HS_LIDAR_L64_Packet *pkt,
        int blockid,
        char chLaserNumber,
        std::shared_ptr<PPointCloud> cld);
    void CalcQTPointXYZIT(
        HS_LIDAR_QT_Packet *pkt,
        int blockid,
        char chLaserNumber,
        std::shared_ptr<PPointCloud> cld);
    void CalcXTPointXYZIT(
        HS_LIDAR_XT_Packet *pkt,
        int blockid,
        char chLaserNumber,
        std::shared_ptr<PPointCloud> cld);

    void FillPacket(const uint8_t *buf, const int len, double timestamp);
    void EmitBackMessege(char chLaserNumber, std::shared_ptr<PPointCloud> cld);

    pthread_mutex_t lidar_lock_;
    sem_t lidar_sem_;
    std::thread *lidar_recv_thr_;
    std::thread *lidar_process_thr_;
    bool enable_lidar_recv_thr_;
    bool enable_lidar_process_thr_;
    int start_angle_;
    std::string m_sTimestampType;
    double m_dPktTimestamp;

    std::list<PandarPacket> lidar_packets_;

    std::unique_ptr<Input> input_;
    std::function<void(std::shared_ptr<PPointCloud> cld, double timestamp)> pcl_callback_;
    std::function<void(double timestamp)> gps_callback_;

    float sin_lookup_table_[ROTATION_MAX_UNITS];
    float cos_lookup_table_[ROTATION_MAX_UNITS];

    uint16_t last_azimuth_;

    float General_elev_angle_map_[MAX_LASER_NUM];
    float General_horizatal_azimuth_offset_map_[MAX_LASER_NUM];

    float block64OffsetSingle_[HS_LIDAR_L64_BLOCK_NUMBER_6];
    float block64OffsetDual_[HS_LIDAR_L64_BLOCK_NUMBER_6];
    float laser64Offset_[HS_LIDAR_L64_UNIT_NUM];

    float block40OffsetSingle_[HS_LIDAR_L40_BLOCKS_PER_PACKET];
    float block40OffsetDual_[HS_LIDAR_L40_BLOCKS_PER_PACKET];
    float laser40Offset_[HS_LIDAR_L40_LASER_COUNT];

    float blockQTOffsetSingle_[HS_LIDAR_QT_BLOCK_NUMBER];
    float blockQTOffsetDual_[HS_LIDAR_QT_BLOCK_NUMBER];
    float laserQTOffset_[HS_LIDAR_QT_UNIT_NUM];

    float blockXTOffsetSingle_[HS_LIDAR_XT_BLOCK_NUMBER];
    float blockXTOffsetDual_[HS_LIDAR_XT_BLOCK_NUMBER];
    float laserXTOffset_[HS_LIDAR_XT_UNIT_NUM];

    int tz_second_;
    std::string frame_id_;
    int pcl_type_;
    PcapReader *pcap_reader_;
    bool connect_lidar_;
    std::string m_sLidarType;
};
