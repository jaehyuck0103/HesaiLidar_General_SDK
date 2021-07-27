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
#include "pandarQT.h"
#include "pandarXT.h"
#include "pcap_reader.h"
#include "point_types.h"

#include <semaphore.h>
#include <thread>

#include <list>
#include <string>

#define SOB_ANGLE_SIZE (4)
#define RAW_MEASURE_SIZE (3)
#define LASER_COUNT (40)
#define BLOCKS_PER_PACKET (10)
#define BLOCK_SIZE (RAW_MEASURE_SIZE * LASER_COUNT + SOB_ANGLE_SIZE)
#define TIMESTAMP_SIZE (4)
#define FACTORY_INFO_SIZE (1)
#define ECHO_SIZE (1)
#define RESERVE_SIZE (8)
#define REVOLUTION_SIZE (2)
#define INFO_SIZE (TIMESTAMP_SIZE + FACTORY_INFO_SIZE + ECHO_SIZE + RESERVE_SIZE + REVOLUTION_SIZE)
#define UTC_TIME (6)
#define PACKET_SIZE (BLOCK_SIZE * BLOCKS_PER_PACKET + INFO_SIZE + UTC_TIME)
#define LASER_RETURN_TO_DISTANCE_RATE (0.004)
#define SEQ_NUM_SIZE (4)

/**
 * Pandar 64
 */
#define HS_LIDAR_TIME_SIZE (6)
// Each Packet have 8 byte
#define HS_LIDAR_L64_HEAD_SIZE (8)
// Block number 6 or 7
#define HS_LIDAR_L64_BLOCK_NUMBER_6 (6)
#define HS_LIDAR_L64_BLOCK_NUMBER_7 (7)

// each block first 2 byte  is azimuth
#define HS_LIDAR_L64_BLOCK_HEADER_AZIMUTH (2)
// each block have  64 Unit
#define HS_LIDAR_L64_UNIT_NUM (64)
// each Unit have 3 byte :2 bytes(distance) + 1 byte(intensity)
#define HS_LIDAR_L64_UNIT_SIZE (3)
// total block size 194
#define HS_LIDAR_L64_BLOCK_SIZE                                                                   \
    (HS_LIDAR_L64_UNIT_SIZE * HS_LIDAR_L64_UNIT_NUM + HS_LIDAR_L64_BLOCK_HEADER_AZIMUTH)

// Block tail = timestamp ( 4 bytes ) + factory num (2 bytes)
#define HS_LIDAR_L64_TIMESTAMP_SIZE (4)
#define HS_LIDAR_L64_ECHO_SIZE (1)
#define HS_LIDAR_L64_FACTORY_SIZE (1)
#define HS_LIDAR_L64_RESERVED_SIZE (8)
#define HS_LIDAR_L64_ENGINE_VELOCITY (2)

// packet body size two type
#define HS_LIDAR_L64_6_BLOCK_PACKET_BODY_SIZE                                                     \
    (HS_LIDAR_L64_BLOCK_SIZE * HS_LIDAR_L64_BLOCK_NUMBER_6)
#define HS_LIDAR_L64_7_BLOCK_PACKET_BODY_SIZE                                                     \
    (HS_LIDAR_L64_BLOCK_SIZE * HS_LIDAR_L64_BLOCK_NUMBER_7)

// packet tail size
#define HS_LIDAR_L64_PACKET_TAIL_SIZE (26)
#define HS_LIDAR_L64_PACKET_TAIL_WITHOUT_UDPSEQ_SIZE (22)

// total packet size two type,length: 1198 and 1392
#define HS_LIDAR_L64_6PACKET_SIZE                                                                 \
    (HS_LIDAR_L64_HEAD_SIZE + HS_LIDAR_L64_6_BLOCK_PACKET_BODY_SIZE +                             \
     HS_LIDAR_L64_PACKET_TAIL_SIZE)
#define HS_LIDAR_L64_7PACKET_SIZE                                                                 \
    (HS_LIDAR_L64_HEAD_SIZE + HS_LIDAR_L64_7_BLOCK_PACKET_BODY_SIZE +                             \
     HS_LIDAR_L64_PACKET_TAIL_SIZE)

#define HS_LIDAR_L64_6PACKET_WITHOUT_UDPSEQ_SIZE                                                  \
    (HS_LIDAR_L64_HEAD_SIZE + HS_LIDAR_L64_6_BLOCK_PACKET_BODY_SIZE +                             \
     HS_LIDAR_L64_PACKET_TAIL_WITHOUT_UDPSEQ_SIZE)
#define HS_LIDAR_L64_7PACKET_WITHOUT_UDPSEQ_SIZE                                                  \
    (HS_LIDAR_L64_HEAD_SIZE + HS_LIDAR_L64_7_BLOCK_PACKET_BODY_SIZE +                             \
     HS_LIDAR_L64_PACKET_TAIL_WITHOUT_UDPSEQ_SIZE)

/**
 * Pandar20
 */
#define HS_LIDAR_L20_HEAD_SIZE (8)
#define HS_LIDAR_L20_BLOCK_NUMBER (20)
#define HS_LIDAR_L20_BLOCK_HEADER_AZIMUTH (2)
#define HS_LIDAR_L20_UNIT_NUM (20)
#define HS_LIDAR_L20_UNIT_SIZE (3)
#define HS_LIDAR_L20_BLOCK_SIZE                                                                   \
    (HS_LIDAR_L20_UNIT_SIZE * HS_LIDAR_L20_UNIT_NUM + HS_LIDAR_L20_BLOCK_HEADER_AZIMUTH)
#define HS_LIDAR_L20_TIMESTAMP_SIZE (4)
#define HS_LIDAR_L20_ECHO_SIZE (1)
#define HS_LIDAR_L20_FACTORY_SIZE (1)
#define HS_LIDAR_L20_RESERVED_SIZE (8)
#define HS_LIDAR_L20_ENGINE_VELOCITY (2)
#define HS_LIDAR_L20_BLOCK_PACKET_BODY_SIZE (HS_LIDAR_L20_BLOCK_SIZE * HS_LIDAR_L20_BLOCK_NUMBER)
#define HS_LIDAR_L20_PACKET_TAIL_SIZE (22)
#define HS_LIDAR_L20_PACKET_SIZE                                                                  \
    (HS_LIDAR_L20_HEAD_SIZE + HS_LIDAR_L20_BLOCK_PACKET_BODY_SIZE + HS_LIDAR_L20_PACKET_TAIL_SIZE)

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

struct Pandar40PUnit {
    uint8_t intensity;
    double distance;
};

struct Pandar40PBlock {
    uint16_t azimuth;
    uint16_t sob;
    Pandar40PUnit units[LASER_COUNT];
};

struct Pandar40PPacket {
    Pandar40PBlock blocks[BLOCKS_PER_PACKET];
    struct tm t;
    uint32_t usec;
    int echo;
};

/************Pandar64*******************************/
struct HS_LIDAR_L64_Header {
    unsigned short sob = 0; // 0xFFEE 2bytes
    char chLaserNumber = 0; // laser number 1byte
    char chBlockNumber = 0; // block number 1byte
    char chReturnType = 0;  // return mode 1 byte  when dual return 0-Single Return
                            // 1-The first block is the 1 st return.
                            // 2-The first block is the 2 nd return
    char chDisUnit = 0;     // Distance unit, 6mm/5mm/4mm
};

struct HS_LIDAR_L64_Unit {
    double distance;
    unsigned short intensity;
};

struct HS_LIDAR_L64_Block {
    unsigned short azimuth; // packet angle  ,Azimuth = RealAzimuth * 100
    HS_LIDAR_L64_Unit units[HS_LIDAR_L64_UNIT_NUM];
};

struct HS_LIDAR_L64_Packet {
    HS_LIDAR_L64_Header header;
    HS_LIDAR_L64_Block blocks[HS_LIDAR_L64_BLOCK_NUMBER_7];
    unsigned int timestamp; // ms
    unsigned int echo;
    unsigned char addtime[6];
};
/***************Pandar64****************************/

/************Pandar20A/B*******************************/
struct HS_LIDAR_L20_Header {
    unsigned short sob = 0; // 0xFFEE 2bytes
    char chLaserNumber = 0; // laser number 1byte
    char chBlockNumber = 0; // block number 1byte
    char chReturnType = 0;  // return mode 1 byte  when dual return 0-Single Return
                            // 1-The first block is the 1 st return.
                            // 2-The first block is the 2 nd return
    char chDisUnit = 0;     // Distance unit, 6mm/5mm/4mm
};

struct HS_LIDAR_L20_Unit {
    double distance;
    unsigned short intensity;
};

struct HS_LIDAR_L20_Block {
    unsigned short azimuth;
    HS_LIDAR_L20_Unit units[HS_LIDAR_L20_UNIT_NUM];
};

struct HS_LIDAR_L20_Packet {
    HS_LIDAR_L20_Header header;
    HS_LIDAR_L20_Block blocks[HS_LIDAR_L20_BLOCK_NUMBER];
    unsigned int timestamp; // ms
    unsigned int echo;
    unsigned char addtime[6];
};
/************Pandar20A/B*******************************/

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

    int Start();
    void Stop();

  private:
    void Init();
    void RecvTask();
    void ProcessGps(const PandarGPS &gpsMsg);
    void ProcessLidarPacket();
    void PushLidarData(PandarPacket packet);
    int ParseRawData(Pandar40PPacket *packet, const uint8_t *buf, const int len);
    int ParseL64Data(HS_LIDAR_L64_Packet *packet, const uint8_t *recvbuf, const int len);
    int ParseL20Data(HS_LIDAR_L20_Packet *packet, const uint8_t *recvbuf, const int len);
    int ParseQTData(HS_LIDAR_QT_Packet *packet, const uint8_t *recvbuf, const int len);
    int ParseXTData(HS_LIDAR_XT_Packet *packet, const uint8_t *recvbuf, const int len);

    int ParseGPS(PandarGPS *packet, const uint8_t *recvbuf, const int size);
    void CalcPointXYZIT(Pandar40PPacket *pkt, int blockid, std::shared_ptr<PPointCloud> cld);
    void CalcL64PointXYZIT(
        HS_LIDAR_L64_Packet *pkt,
        int blockid,
        char chLaserNumber,
        std::shared_ptr<PPointCloud> cld);
    void CalcL20PointXYZIT(
        HS_LIDAR_L20_Packet *pkt,
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

    std::list<struct PandarPacket> lidar_packets_;

    std::shared_ptr<Input> input_;
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

    float block40OffsetSingle_[BLOCKS_PER_PACKET];
    float block40OffsetDual_[BLOCKS_PER_PACKET];
    float laser40Offset_[LASER_COUNT];

    float block20OffsetSingle_[HS_LIDAR_L20_BLOCK_NUMBER];
    float block20OffsetDual_[HS_LIDAR_L20_BLOCK_NUMBER];
    float laser20AOffset_[HS_LIDAR_L20_UNIT_NUM];
    float laser20BOffset_[HS_LIDAR_L20_UNIT_NUM];

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
