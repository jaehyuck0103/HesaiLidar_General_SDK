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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

double degreeToRadian(double degree) { return degree * M_PI / 180; }

static std::vector<std::vector<PPoint>> PointCloudList(128);

PandarGeneral_Internal::PandarGeneral_Internal(
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
    std::string timestampType) {
    // LOG_FUNC();
    pthread_mutex_init(&lidar_lock_, NULL);
    sem_init(&lidar_sem_, 0, 0);

    lidar_recv_thr_ = NULL;
    lidar_process_thr_ = NULL;

    enable_lidar_recv_thr_ = false;
    enable_lidar_process_thr_ = false;

    input_ = std::make_unique<Input>(lidar_port, gps_port);

    start_angle_ = start_angle;
    pcl_callback_ = pcl_callback;
    gps_callback_ = gps_callback;
    last_azimuth_ = 0;
    m_sLidarType = lidar_type;
    frame_id_ = frame_id;
    tz_second_ = tz * 3600;
    pcl_type_ = pcl_type;
    connect_lidar_ = true;
    pcap_reader_ = NULL;
    m_sTimestampType = timestampType;
    m_dPktTimestamp = 0.0f;

    Init();
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

    lidar_recv_thr_ = NULL;
    lidar_process_thr_ = NULL;

    enable_lidar_recv_thr_ = false;
    enable_lidar_process_thr_ = false;

    pcap_reader_ = new PcapReader(pcap_path, lidar_type);

    start_angle_ = start_angle;
    pcl_callback_ = pcl_callback;
    gps_callback_ = NULL;
    last_azimuth_ = 0;
    m_sLidarType = lidar_type;
    frame_id_ = frame_id;
    tz_second_ = tz * 3600;
    pcl_type_ = pcl_type;
    connect_lidar_ = false;
    m_sTimestampType = timestampType;
    m_dPktTimestamp = 0.0f;

    Init();
}

PandarGeneral_Internal::~PandarGeneral_Internal() {
    Stop();
    sem_destroy(&lidar_sem_);
    pthread_mutex_destroy(&lidar_lock_);

    if (pcap_reader_ != NULL) {
        delete pcap_reader_;
        pcap_reader_ = NULL;
    }
}

void PandarGeneral_Internal::Init() {
    for (uint16_t rotIndex = 0; rotIndex < ROTATION_MAX_UNITS; ++rotIndex) {
        float rotation = degreeToRadian(0.01 * static_cast<double>(rotIndex));
        cos_lookup_table_[rotIndex] = cosf(rotation);
        sin_lookup_table_[rotIndex] = sinf(rotation);
    }

    // laser40
    //  init the block time offset, us
    block40OffsetSingle_[9] = 55.56f * 0.0f + 28.58f;
    block40OffsetSingle_[8] = 55.56f * 1.0f + 28.58f;
    block40OffsetSingle_[7] = 55.56f * 2.0f + 28.58f;
    block40OffsetSingle_[6] = 55.56f * 3.0f + 28.58f;
    block40OffsetSingle_[5] = 55.56f * 4.0f + 28.58f;
    block40OffsetSingle_[4] = 55.56f * 5.0f + 28.58f;
    block40OffsetSingle_[3] = 55.56f * 6.0f + 28.58f;
    block40OffsetSingle_[2] = 55.56f * 7.0f + 28.58f;
    block40OffsetSingle_[1] = 55.56f * 8.0f + 28.58f;
    block40OffsetSingle_[0] = 55.56f * 9.0f + 28.58f;

    block40OffsetDual_[9] = 55.56f * 0.0f + 28.58f;
    block40OffsetDual_[8] = 55.56f * 0.0f + 28.58f;
    block40OffsetDual_[7] = 55.56f * 1.0f + 28.58f;
    block40OffsetDual_[6] = 55.56f * 1.0f + 28.58f;
    block40OffsetDual_[5] = 55.56f * 2.0f + 28.58f;
    block40OffsetDual_[4] = 55.56f * 2.0f + 28.58f;
    block40OffsetDual_[3] = 55.56f * 3.0f + 28.58f;
    block40OffsetDual_[2] = 55.56f * 3.0f + 28.58f;
    block40OffsetDual_[1] = 55.56f * 4.0f + 28.58f;
    block40OffsetDual_[0] = 55.56f * 4.0f + 28.58f;

    // init the laser shot time offset, us
    laser40Offset_[3] = 3.62f;
    laser40Offset_[39] = 3.62f;
    laser40Offset_[35] = 4.92f;
    laser40Offset_[27] = 6.23f;
    laser40Offset_[11] = 8.19f;
    laser40Offset_[15] = 8.19f;
    laser40Offset_[31] = 9.5f;
    laser40Offset_[23] = 11.47f;
    laser40Offset_[28] = 12.77f;
    laser40Offset_[16] = 14.74f;
    laser40Offset_[2] = 16.04f;
    laser40Offset_[38] = 16.04f;
    laser40Offset_[34] = 17.35f;
    laser40Offset_[24] = 18.65f;
    laser40Offset_[8] = 20.62f;
    laser40Offset_[12] = 20.62f;
    laser40Offset_[30] = 21.92f;
    laser40Offset_[20] = 23.89f;
    laser40Offset_[25] = 25.19f;
    laser40Offset_[13] = 27.16f;
    laser40Offset_[1] = 28.47f;
    laser40Offset_[37] = 28.47f;
    laser40Offset_[33] = 29.77f;
    laser40Offset_[5] = 31.74f;
    laser40Offset_[21] = 31.7447f;
    laser40Offset_[9] = 33.71f;
    laser40Offset_[29] = 35.01f;
    laser40Offset_[17] = 36.98f;
    laser40Offset_[22] = 38.95f;
    laser40Offset_[10] = 40.91f;
    laser40Offset_[0] = 42.22f;
    laser40Offset_[36] = 42.22f;
    laser40Offset_[32] = 43.52f;
    laser40Offset_[4] = 45.49f;
    laser40Offset_[18] = 45.49f;
    laser40Offset_[6] = 47.46f;
    laser40Offset_[26] = 48.76f;
    laser40Offset_[14] = 50.73f;
    laser40Offset_[19] = 52.7f;
    laser40Offset_[7] = 54.67f;

    // laser64 init the laser shot time offset, us
    //  init the block time offset, us
    block64OffsetSingle_[5] = 55.56f * 0.0f + 42.58f;
    block64OffsetSingle_[4] = 55.56f * 1.0f + 42.58f;
    block64OffsetSingle_[3] = 55.56f * 2.0f + 42.58f;
    block64OffsetSingle_[2] = 55.56f * 3.0f + 42.58f;
    block64OffsetSingle_[1] = 55.56f * 4.0f + 42.58f;
    block64OffsetSingle_[0] = 55.56f * 5.0f + 42.58f;

    block64OffsetDual_[5] = 55.56f * 0.0f + 42.58f;
    block64OffsetDual_[4] = 55.56f * 0.0f + 42.58f;
    block64OffsetDual_[3] = 55.56f * 1.0f + 42.58f;
    block64OffsetDual_[2] = 55.56f * 1.0f + 42.58f;
    block64OffsetDual_[1] = 55.56f * 2.0f + 42.58f;
    block64OffsetDual_[0] = 55.56f * 2.0f + 42.58f;

    laser64Offset_[50] = 1.304f * 0.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[60] = 1.304f * 0.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[44] = 1.304f * 1.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[59] = 1.304f * 1.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[38] = 1.304f * 2.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[56] = 1.304f * 2.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[8] = 1.304f * 3.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[54] = 1.304f * 3.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[48] = 1.304f * 4.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[62] = 1.304f * 4.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[42] = 1.304f * 5.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[58] = 1.304f * 5.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[6] = 1.304f * 6.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[55] = 1.304f * 6.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[52] = 1.304f * 7.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[63] = 1.304f * 7.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[46] = 1.304f * 8.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[61] = 1.304f * 8.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[40] = 1.304f * 9.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[57] = 1.304f * 9.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[5] = 1.304f * 10.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[53] = 1.304f * 10.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[4] = 1.304f * 11.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[47] = 1.304f * 11.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[3] = 1.304f * 12.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[49] = 1.304f * 12.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[2] = 1.304f * 13.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[51] = 1.304f * 13.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[1] = 1.304f * 14.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[45] = 1.304f * 14.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[0] = 1.304f * 15.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[43] = 1.304f * 15.0f + 1.968f * 0.0f + 3.62f;
    laser64Offset_[23] = 1.304f * 15.0f + 1.968f * 1.0f + 3.62f;
    laser64Offset_[32] = 1.304f * 15.0f + 1.968f * 1.0f + 3.62f;
    laser64Offset_[26] = 1.304f * 15.0f + 1.968f * 2.0f + 3.62f;
    laser64Offset_[41] = 1.304f * 15.0f + 1.968f * 2.0f + 3.62f;
    laser64Offset_[20] = 1.304f * 15.0f + 1.968f * 3.0f + 3.62f;
    laser64Offset_[35] = 1.304f * 15.0f + 1.968f * 3.0f + 3.62f;
    laser64Offset_[14] = 1.304f * 15.0f + 1.968f * 4.0f + 3.62f;
    laser64Offset_[29] = 1.304f * 15.0f + 1.968f * 4.0f + 3.62f;
    laser64Offset_[21] = 1.304f * 15.0f + 1.968f * 5.0f + 3.62f;
    laser64Offset_[36] = 1.304f * 15.0f + 1.968f * 5.0f + 3.62f;
    laser64Offset_[15] = 1.304f * 15.0f + 1.968f * 6.0f + 3.62f;
    laser64Offset_[30] = 1.304f * 15.0f + 1.968f * 6.0f + 3.62f;
    laser64Offset_[9] = 1.304f * 15.0f + 1.968f * 7.0f + 3.62f;
    laser64Offset_[24] = 1.304f * 15.0f + 1.968f * 7.0f + 3.62f;
    laser64Offset_[18] = 1.304f * 15.0f + 1.968f * 8.0f + 3.62f;
    laser64Offset_[33] = 1.304f * 15.0f + 1.968f * 8.0f + 3.62f;
    laser64Offset_[12] = 1.304f * 15.0f + 1.968f * 9.0f + 3.62f;
    laser64Offset_[27] = 1.304f * 15.0f + 1.968f * 9.0f + 3.62f;
    laser64Offset_[19] = 1.304f * 15.0f + 1.968f * 10.0f + 3.62f;
    laser64Offset_[34] = 1.304f * 15.0f + 1.968f * 10.0f + 3.62f;
    laser64Offset_[13] = 1.304f * 15.0f + 1.968f * 11.0f + 3.62f;
    laser64Offset_[28] = 1.304f * 15.0f + 1.968f * 11.0f + 3.62f;
    laser64Offset_[7] = 1.304f * 15.0f + 1.968f * 12.0f + 3.62f;
    laser64Offset_[22] = 1.304f * 15.0f + 1.968f * 12.0f + 3.62f;
    laser64Offset_[16] = 1.304f * 15.0f + 1.968f * 13.0f + 3.62f;
    laser64Offset_[31] = 1.304f * 15.0f + 1.968f * 13.0f + 3.62f;
    laser64Offset_[10] = 1.304f * 15.0f + 1.968f * 14.0f + 3.62f;
    laser64Offset_[25] = 1.304f * 15.0f + 1.968f * 14.0f + 3.62f;
    laser64Offset_[17] = 1.304f * 15.0f + 1.968f * 15.0f + 3.62f;
    laser64Offset_[37] = 1.304f * 15.0f + 1.968f * 15.0f + 3.62f;
    laser64Offset_[11] = 1.304f * 15.0f + 1.968f * 16.0f + 3.62f;
    laser64Offset_[39] = 1.304f * 15.0f + 1.968f * 16.0f + 3.62f;

    // QT
    blockQTOffsetSingle_[0] = 25.71f;
    blockQTOffsetSingle_[1] = 25.71f + 166.67f;
    blockQTOffsetSingle_[2] = 25.71f + 333.33f;
    blockQTOffsetSingle_[3] = 25.71f + 500.00f;

    blockQTOffsetDual_[0] = 25.71f;
    blockQTOffsetDual_[1] = 25.71f;
    blockQTOffsetDual_[2] = 25.71f + 166.67f;
    blockQTOffsetDual_[3] = 25.71f + 166.67f;

    laserQTOffset_[0] = 10.0f + 2.31f;
    laserQTOffset_[1] = 10.0f + 4.37f;
    laserQTOffset_[2] = 10.0f + 6.43f;
    laserQTOffset_[3] = 10.0f + 8.49f;
    laserQTOffset_[4] = 10.0f + 10.54f;
    laserQTOffset_[5] = 10.0f + 12.60f;
    laserQTOffset_[6] = 10.0f + 14.66f;
    laserQTOffset_[7] = 10.0f + 16.71f;
    laserQTOffset_[8] = 10.0f + 19.16f;
    laserQTOffset_[9] = 10.0f + 21.22f;
    laserQTOffset_[10] = 10.0f + 23.28f;
    laserQTOffset_[11] = 10.0f + 25.34f;
    laserQTOffset_[12] = 10.0f + 27.39f;
    laserQTOffset_[13] = 10.0f + 29.45f;
    laserQTOffset_[14] = 10.0f + 31.50f;
    laserQTOffset_[15] = 10.0f + 33.56f;

    laserQTOffset_[16] = 10.0f + 36.61f;
    laserQTOffset_[17] = 10.0f + 38.67f;
    laserQTOffset_[18] = 10.0f + 40.73f;
    laserQTOffset_[19] = 10.0f + 42.78f;
    laserQTOffset_[20] = 10.0f + 44.84f;
    laserQTOffset_[21] = 10.0f + 46.90f;
    laserQTOffset_[22] = 10.0f + 48.95f;
    laserQTOffset_[23] = 10.0f + 51.01f;
    laserQTOffset_[24] = 10.0f + 53.45f;
    laserQTOffset_[25] = 10.0f + 55.52f;
    laserQTOffset_[26] = 10.0f + 57.58f;
    laserQTOffset_[27] = 10.0f + 59.63f;
    laserQTOffset_[28] = 10.0f + 61.69f;
    laserQTOffset_[29] = 10.0f + 63.74f;
    laserQTOffset_[30] = 10.0f + 65.80f;
    laserQTOffset_[31] = 10.0f + 67.86f;

    laserQTOffset_[32] = 10.0f + 70.90f;
    laserQTOffset_[33] = 10.0f + 72.97f;
    laserQTOffset_[34] = 10.0f + 75.02f;
    laserQTOffset_[35] = 10.0f + 77.08f;
    laserQTOffset_[36] = 10.0f + 79.14f;
    laserQTOffset_[37] = 10.0f + 81.19f;
    laserQTOffset_[38] = 10.0f + 83.25f;
    laserQTOffset_[39] = 10.0f + 85.30f;
    laserQTOffset_[40] = 10.0f + 87.75f;
    laserQTOffset_[41] = 10.0f + 89.82f;
    laserQTOffset_[42] = 10.0f + 91.87f;
    laserQTOffset_[43] = 10.0f + 93.93f;
    laserQTOffset_[44] = 10.0f + 95.98f;
    laserQTOffset_[45] = 10.0f + 98.04f;
    laserQTOffset_[46] = 10.0f + 100.10f;
    laserQTOffset_[47] = 10.0f + 102.15f;

    laserQTOffset_[48] = 10.0f + 105.20f;
    laserQTOffset_[49] = 10.0f + 107.26f;
    laserQTOffset_[50] = 10.0f + 109.32f;
    laserQTOffset_[51] = 10.0f + 111.38f;
    laserQTOffset_[52] = 10.0f + 113.43f;
    laserQTOffset_[53] = 10.0f + 115.49f;
    laserQTOffset_[54] = 10.0f + 117.54f;
    laserQTOffset_[55] = 10.0f + 119.60f;
    laserQTOffset_[56] = 10.0f + 122.05f;
    laserQTOffset_[57] = 10.0f + 124.11f;
    laserQTOffset_[58] = 10.0f + 126.17f;
    laserQTOffset_[59] = 10.0f + 128.22f;
    laserQTOffset_[60] = 10.0f + 130.28f;
    laserQTOffset_[61] = 10.0f + 132.34f;
    laserQTOffset_[62] = 10.0f + 134.39f;
    laserQTOffset_[63] = 10.0f + 136.45f;

    if (m_sLidarType == "Pandar40P" || m_sLidarType == "Pandar40M") {
        for (int i = 0; i < HS_LIDAR_L40_LASER_COUNT; i++) {
            elev_angle_map_[i] = pandar40p_elev_angle_map[i];
            azimuth_offset_map_[i] = pandar40p_horizatal_azimuth_offset_map[i];
        }
    }

    if (m_sLidarType == "Pandar64") {
        for (int i = 0; i < HS_LIDAR_L64_UNIT_NUM; i++) {
            elev_angle_map_[i] = pandar64_elev_angle_map[i];
            azimuth_offset_map_[i] = pandar64_horizatal_azimuth_offset_map[i];
        }
    }

    if (m_sLidarType == "PandarQT") {
        for (int i = 0; i < HS_LIDAR_QT_UNIT_NUM; i++) {
            elev_angle_map_[i] = pandarQT_elev_angle_map[i];
            azimuth_offset_map_[i] = pandarQT_horizatal_azimuth_offset_map[i];
        }
    }

    if (m_sLidarType == "PandarXT-32") {
        for (int i = 0; i < HS_LIDAR_XT_UNIT_NUM; i++) {
            elev_angle_map_[i] = pandarXT_elev_angle_map[i];
            azimuth_offset_map_[i] = pandarXT_horizatal_azimuth_offset_map[i];
            laserXTOffset_[i] = laserXTOffset[i];
        }
    }

    if (m_sLidarType == "PandarXT-16") {
        for (int i = 0; i < HS_LIDAR_XT16_UNIT_NUM; i++) {
            elev_angle_map_[i] = pandarXT_elev_angle_map[i * 2];
            azimuth_offset_map_[i] = pandarXT_horizatal_azimuth_offset_map[i * 2];
            laserXTOffset_[i] = laserXTOffset[i * 2];
        }
    }

    for (int i = 0; i < HS_LIDAR_XT_BLOCK_NUMBER; i++) {
        blockXTOffsetSingle_[i] = blockXTOffsetSingle[i];
        blockXTOffsetDual_[i] = blockXTOffsetDual[i];
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

/**
 * @brief load the correction file
 * @param angle The start angle
 */
void PandarGeneral_Internal::ResetStartAngle(uint16_t start_angle) { start_angle_ = start_angle; }

void PandarGeneral_Internal::Start() {
    // LOG_FUNC();
    Stop();
    enable_lidar_recv_thr_ = true;
    enable_lidar_process_thr_ = true;
    lidar_process_thr_ =
        new std::thread(std::bind(&PandarGeneral_Internal::ProcessLidarPacket, this));

    if (connect_lidar_) {
        lidar_recv_thr_ = new std::thread(std::bind(&PandarGeneral_Internal::RecvTask, this));
    } else {
        pcap_reader_->start(std::bind(
            &PandarGeneral_Internal::FillPacket,
            this,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3));
    }
}

void PandarGeneral_Internal::Stop() {
    enable_lidar_recv_thr_ = false;
    enable_lidar_process_thr_ = false;

    if (lidar_process_thr_) {
        lidar_process_thr_->join();
        delete lidar_process_thr_;
        lidar_process_thr_ = NULL;
    }

    if (lidar_recv_thr_) {
        lidar_recv_thr_->join();
        delete lidar_recv_thr_;
        lidar_recv_thr_ = NULL;
    }

    if (pcap_reader_ != NULL) {
        pcap_reader_->stop();
    }

    return;
}

void PandarGeneral_Internal::RecvTask() {
    // LOG_FUNC();
    int ret = 0;
    while (enable_lidar_recv_thr_) {
        PandarPacket pkt;
        int rc = input_->getPacket(&pkt);
        if (rc == -1) {
            continue;
        }

        if (pkt.size == GPS_PACKET_SIZE) {
            PandarGPS gpsMsg;
            ret = ParseGPS(&gpsMsg, pkt.data, pkt.size);
            if (ret == 0) {
                ProcessGps(gpsMsg);
            }
            continue;
        }

        PushLidarData(pkt);
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
    // LOG_FUNC();
    timespec ts;
    int ret = 0;

    std::shared_ptr<PPointCloud> outMsg(new PPointCloud());

    while (enable_lidar_process_thr_) {
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
        m_dPktTimestamp = packet.stamp;

        if (packet.size == HS_LIDAR_L40_PACKET_SIZE ||
            packet.size == HS_LIDAR_L40_PACKET_SIZE + HS_LIDAR_L40_SEQ_NUM_SIZE) {
            HS_LIDAR_L40_Packet pkt;
            ret = ParseL40Data(&pkt, packet.data, packet.size);

            if (ret != 0) {
                continue;
            }

            for (int i = 0; i < HS_LIDAR_L40_BLOCKS_PER_PACKET; ++i) {
                int azimuthGap = 0; /* To do */

                if (last_azimuth_ > pkt.blocks[i].azimuth) {
                    azimuthGap = static_cast<int>(pkt.blocks[i].azimuth) +
                                 (36000 - static_cast<int>(last_azimuth_));
                } else {
                    azimuthGap =
                        static_cast<int>(pkt.blocks[i].azimuth) - static_cast<int>(last_azimuth_);
                }

                if (last_azimuth_ != pkt.blocks[i].azimuth && azimuthGap < 600 /* 6 degree*/) {
                    /* for all the blocks */
                    if ((last_azimuth_ > pkt.blocks[i].azimuth &&
                         start_angle_ <= pkt.blocks[i].azimuth) ||
                        (last_azimuth_ < start_angle_ && start_angle_ <= pkt.blocks[i].azimuth)) {
                        if (pcl_callback_ &&
                            (outMsg->points.size() > 0 || PointCloudList[0].size() > 0)) {
                            EmitBackMessege(HS_LIDAR_L40_LASER_COUNT, outMsg);
                            outMsg.reset(new PPointCloud());
                        }
                    }
                }
                CalcL40PointXYZIT(&pkt, i, outMsg);
                last_azimuth_ = pkt.blocks[i].azimuth;
            }
        } else if (
            packet.size == HS_LIDAR_L64_PACKET_SIZE ||
            packet.size == HS_LIDAR_L64_PACKET_WITHOUT_UDPSEQ_SIZE) {
            HS_LIDAR_L64_Packet pkt;
            ret = ParseL64Data(&pkt, packet.data, packet.size);

            if (ret != 0) {
                continue;
            }

            for (int i = 0; i < pkt.header.chBlockNumber; ++i) {
                int azimuthGap = 0; /* To do */
                if (last_azimuth_ > pkt.blocks[i].azimuth) {
                    azimuthGap = static_cast<int>(pkt.blocks[i].azimuth) +
                                 (36000 - static_cast<int>(last_azimuth_));
                } else {
                    azimuthGap =
                        static_cast<int>(pkt.blocks[i].azimuth) - static_cast<int>(last_azimuth_);
                }

                if (last_azimuth_ != pkt.blocks[i].azimuth && azimuthGap < 600 /* 6 degree*/) {
                    /* for all the blocks */
                    if ((last_azimuth_ > pkt.blocks[i].azimuth &&
                         start_angle_ <= pkt.blocks[i].azimuth) ||
                        (last_azimuth_ < start_angle_ && start_angle_ <= pkt.blocks[i].azimuth)) {
                        if (pcl_callback_ &&
                            (outMsg->points.size() > 0 || PointCloudList[0].size() > 0)) {
                            EmitBackMessege(pkt.header.chLaserNumber, outMsg);
                            outMsg.reset(new PPointCloud());
                        }
                    }
                } else {
                    // printf("last_azimuth_:%d pkt.blocks[i].azimuth:%d
                    // *******azimuthGap:%d\n", last_azimuth_, pkt.blocks[i].azimuth,
                    // azimuthGap);
                }

                CalcL64PointXYZIT(&pkt, i, pkt.header.chLaserNumber, outMsg);
                last_azimuth_ = pkt.blocks[i].azimuth;
            }
        } else if (
            packet.size == HS_LIDAR_QT_PACKET_SIZE ||
            packet.size == HS_LIDAR_QT_PACKET_WITHOUT_UDPSEQ_SIZE) {
            HS_LIDAR_QT_Packet pkt;
            ret = ParseQTData(&pkt, packet.data, packet.size);
            if (ret != 0) {
                continue;
            }

            for (int i = 0; i < pkt.header.chBlockNumber; ++i) {
                int azimuthGap = 0; /* To do */
                if (last_azimuth_ > pkt.blocks[i].azimuth) {
                    azimuthGap = static_cast<int>(pkt.blocks[i].azimuth) +
                                 (36000 - static_cast<int>(last_azimuth_));
                } else {
                    azimuthGap =
                        static_cast<int>(pkt.blocks[i].azimuth) - static_cast<int>(last_azimuth_);
                }

                if (last_azimuth_ != pkt.blocks[i].azimuth && azimuthGap < 600 /* 6 degree*/) {
                    /* for all the blocks */
                    if ((last_azimuth_ > pkt.blocks[i].azimuth &&
                         start_angle_ <= pkt.blocks[i].azimuth) ||
                        (last_azimuth_ < start_angle_ && start_angle_ <= pkt.blocks[i].azimuth)) {
                        if (pcl_callback_ &&
                            (outMsg->points.size() > 0 || PointCloudList[0].size() > 0)) {
                            EmitBackMessege(pkt.header.chLaserNumber, outMsg);
                            outMsg.reset(new PPointCloud());
                        }
                    }
                } else {
                    // printf("last_azimuth_:%d pkt.blocks[i].azimuth:%d
                    // *******azimuthGap:%d\n", last_azimuth_, pkt.blocks[i].azimuth,
                    // azimuthGap);
                }
                CalcQTPointXYZIT(&pkt, i, pkt.header.chLaserNumber, outMsg);
                last_azimuth_ = pkt.blocks[i].azimuth;
            }
        } else if (
            (packet.size == HS_LIDAR_XT_PACKET_SIZE &&
             (m_sLidarType == "XT" || m_sLidarType == "PandarXT-32")) ||
            (packet.size == HS_LIDAR_XT16_PACKET_SIZE && (m_sLidarType == "PandarXT-16"))) {
            HS_LIDAR_XT_Packet pkt;
            ret = ParseXTData(&pkt, packet.data, packet.size);
            if (ret != 0) {
                continue;
            }

            for (int i = 0; i < pkt.header.chBlockNumber; ++i) {
                int azimuthGap = 0; /* To do */
                if (last_azimuth_ > pkt.blocks[i].azimuth) {
                    azimuthGap = static_cast<int>(pkt.blocks[i].azimuth) +
                                 (36000 - static_cast<int>(last_azimuth_));
                } else {
                    azimuthGap =
                        static_cast<int>(pkt.blocks[i].azimuth) - static_cast<int>(last_azimuth_);
                }

                if (last_azimuth_ != pkt.blocks[i].azimuth && azimuthGap < 600 /* 6 degree*/) {
                    /* for all the blocks */
                    if ((last_azimuth_ > pkt.blocks[i].azimuth &&
                         start_angle_ <= pkt.blocks[i].azimuth) ||
                        (last_azimuth_ < start_angle_ && start_angle_ <= pkt.blocks[i].azimuth)) {
                        if (pcl_callback_ &&
                            (outMsg->points.size() > 0 || PointCloudList[0].size() > 0)) {
                            EmitBackMessege(pkt.header.chLaserNumber, outMsg);
                            outMsg.reset(new PPointCloud());
                        }
                    }
                } else {
                    // printf("last_azimuth_:%d pkt.blocks[i].azimuth:%d
                    // *******azimuthGap:%d\n", last_azimuth_, pkt.blocks[i].azimuth,
                    // azimuthGap);
                }
                CalcXTPointXYZIT(&pkt, i, pkt.header.chLaserNumber, outMsg);
                last_azimuth_ = pkt.blocks[i].azimuth;
            }
        } else {
            continue;
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

int PandarGeneral_Internal::ParseL40Data(
    HS_LIDAR_L40_Packet *packet,
    const uint8_t *recvbuf,
    const int len) {
    if (len != HS_LIDAR_L40_PACKET_SIZE &&
        len != HS_LIDAR_L40_PACKET_SIZE + HS_LIDAR_L40_SEQ_NUM_SIZE) {
        std::cout << "packet size mismatch PandarGeneral_Internal " << len << ","
                  << HS_LIDAR_L40_PACKET_SIZE << std::endl;
        return -1;
    }

    int index = 0;
    // 10 BLOCKs
    for (int i = 0; i < HS_LIDAR_L40_BLOCKS_PER_PACKET; i++) {
        HS_LIDAR_L40_Block &block = packet->blocks[i];

        block.sob = (recvbuf[index] & 0xff) | ((recvbuf[index + 1] & 0xff) << 8);
        block.azimuth = (recvbuf[index + 2] & 0xff) | ((recvbuf[index + 3] & 0xff) << 8);
        index += HS_LIDAR_L40_SOB_SIZE;
        // 40x units
        for (int j = 0; j < HS_LIDAR_L40_LASER_COUNT; j++) {
            HS_LIDAR_L40_Unit &unit = block.units[j];
            uint32_t range = (recvbuf[index] & 0xff) | ((recvbuf[index + 1] & 0xff) << 8);

            // distance is M.
            unit.distance =
                (static_cast<double>(range)) * HS_LIDAR_L40_LASER_RETURN_TO_DISTANCE_RATE;
            unit.intensity = (recvbuf[index + 2] & 0xff);

            // TODO(Philip.Pi): Filtering wrong data for LiDAR.
            if ((unit.distance == 0x010101 && unit.intensity == 0x0101) ||
                unit.distance > (200. * 1000 / 2 /* 200m -> 2mm */)) {
                unit.distance = 0;
                unit.intensity = 0;
            }

            index += HS_LIDAR_L40_RAW_MEASURE_SIZE;
        }
    }

    index += HS_LIDAR_L40_RESERVE_SIZE; // skip reserved bytes

    index += HS_LIDAR_L40_REVOLUTION_SIZE;

    packet->timestamp = (recvbuf[index] & 0xff) | (recvbuf[index + 1] & 0xff) << 8 |
                        ((recvbuf[index + 2] & 0xff) << 16) | ((recvbuf[index + 3] & 0xff) << 24);

    index += HS_LIDAR_L40_TIMESTAMP_SIZE;
    packet->echo = recvbuf[index] & 0xff;

    index += HS_LIDAR_L40_FACTORY_INFO_SIZE + HS_LIDAR_L40_ECHO_SIZE;

    packet->addtime[0] = recvbuf[index] & 0xff;
    packet->addtime[1] = recvbuf[index + 1] & 0xff;
    packet->addtime[2] = recvbuf[index + 2] & 0xff;
    packet->addtime[3] = recvbuf[index + 3] & 0xff;
    packet->addtime[4] = recvbuf[index + 4] & 0xff;
    packet->addtime[5] = recvbuf[index + 5] & 0xff;

    index += HS_LIDAR_L40_UTC_TIME;

    return 0;
}

int PandarGeneral_Internal::ParseL64Data(
    HS_LIDAR_L64_Packet *packet,
    const uint8_t *recvbuf,
    const int len) {
    if (len != HS_LIDAR_L64_PACKET_SIZE && len != HS_LIDAR_L64_PACKET_WITHOUT_UDPSEQ_SIZE &&
        len != 1270) {
        std::cout << "packet size mismatch PandarGeneral_Internal " << len << "," << len
                  << std::endl;
        return -1;
    }

    int index = 0;
    int block = 0;
    // Parse 8 Bytes Header
    packet->header.sob = (recvbuf[index] & 0xff) << 8 | ((recvbuf[index + 1] & 0xff));
    packet->header.chLaserNumber = recvbuf[index + 2] & 0xff;
    packet->header.chBlockNumber = recvbuf[index + 3] & 0xff;
    packet->header.chReturnType = recvbuf[index + 4] & 0xff;
    packet->header.chDisUnit = recvbuf[index + 5] & 0xff;
    index += HS_LIDAR_L64_HEAD_SIZE;

    if (packet->header.sob != 0xEEFF) {
        std::cout << "Error Start of Packet!\n";
        return -1;
    }

    for (block = 0; block < packet->header.chBlockNumber; block++) {
        packet->blocks[block].azimuth =
            (recvbuf[index] & 0xff) | ((recvbuf[index + 1] & 0xff) << 8);
        index += HS_LIDAR_L64_BLOCK_HEADER_AZIMUTH;

        int unit;

        for (unit = 0; unit < packet->header.chLaserNumber; unit++) {
            unsigned int unRange = (recvbuf[index] & 0xff) | ((recvbuf[index + 1] & 0xff) << 8);

            packet->blocks[block].units[unit].distance =
                (static_cast<double>(unRange * packet->header.chDisUnit)) / (double)1000;
            packet->blocks[block].units[unit].intensity = (recvbuf[index + 2] & 0xff);
            index += HS_LIDAR_L64_UNIT_SIZE;
        }
    }

    index += HS_LIDAR_L64_RESERVED_SIZE; // skip reserved bytes
    index += HS_LIDAR_L64_ENGINE_VELOCITY;

    packet->timestamp = (recvbuf[index] & 0xff) | (recvbuf[index + 1] & 0xff) << 8 |
                        ((recvbuf[index + 2] & 0xff) << 16) | ((recvbuf[index + 3] & 0xff) << 24);
    // printf("timestamp %u \n", packet->timestamp);
    index += HS_LIDAR_L64_TIMESTAMP_SIZE;

    packet->echo = recvbuf[index] & 0xff;

    index += HS_LIDAR_L64_ECHO_SIZE;
    index += HS_LIDAR_L64_FACTORY_SIZE;

    packet->addtime[0] = recvbuf[index] & 0xff;
    packet->addtime[1] = recvbuf[index + 1] & 0xff;
    packet->addtime[2] = recvbuf[index + 2] & 0xff;
    packet->addtime[3] = recvbuf[index + 3] & 0xff;
    packet->addtime[4] = recvbuf[index + 4] & 0xff;
    packet->addtime[5] = recvbuf[index + 5] & 0xff;

    index += HS_LIDAR_L64_TIME_SIZE;

    return 0;
}

/**
 * Pandar QT
 */
int PandarGeneral_Internal::ParseQTData(
    HS_LIDAR_QT_Packet *packet,
    const uint8_t *recvbuf,
    const int len) {
    if (len != HS_LIDAR_QT_PACKET_SIZE && len != HS_LIDAR_QT_PACKET_WITHOUT_UDPSEQ_SIZE) {
        std::cout << "packet size mismatch PandarQT " << len << "," << len << std::endl;
        return -1;
    }

    int index = 0;
    int block = 0;
    // Parse 12 Bytes Header
    packet->header.sob = (recvbuf[index] & 0xff) << 8 | ((recvbuf[index + 1] & 0xff));
    packet->header.chProtocolMajor = recvbuf[index + 2] & 0xff;
    packet->header.chProtocolMinor = recvbuf[index + 3] & 0xff;
    packet->header.chLaserNumber = recvbuf[index + 6] & 0xff;
    packet->header.chBlockNumber = recvbuf[index + 7] & 0xff;
    packet->header.chReturnType = recvbuf[index + 8] & 0xff;
    packet->header.chDisUnit = recvbuf[index + 9] & 0xff;
    index += HS_LIDAR_QT_HEAD_SIZE;

    if (packet->header.sob != 0xEEFF) {
        printf("Error Start of Packet!\n");
        return -1;
    }

    for (block = 0; block < packet->header.chBlockNumber; block++) {
        packet->blocks[block].azimuth =
            (recvbuf[index] & 0xff) | ((recvbuf[index + 1] & 0xff) << 8);
        index += HS_LIDAR_QT_BLOCK_HEADER_AZIMUTH;

        int unit;

        for (unit = 0; unit < packet->header.chLaserNumber; unit++) {
            unsigned int unRange = (recvbuf[index] & 0xff) | ((recvbuf[index + 1] & 0xff) << 8);

            packet->blocks[block].units[unit].distance =
                (static_cast<double>(unRange * packet->header.chDisUnit)) / (double)1000;
            packet->blocks[block].units[unit].intensity = (recvbuf[index + 2] & 0xff);
            packet->blocks[block].units[unit].confidence = (recvbuf[index + 3] & 0xff);
            index += HS_LIDAR_QT_UNIT_SIZE;
        }
    }

    index += HS_LIDAR_QT_RESERVED_SIZE; // skip reserved bytes
    index += HS_LIDAR_QT_ENGINE_VELOCITY;

    packet->timestamp = (recvbuf[index] & 0xff) | (recvbuf[index + 1] & 0xff) << 8 |
                        ((recvbuf[index + 2] & 0xff) << 16) | ((recvbuf[index + 3] & 0xff) << 24);
    // printf("timestamp %u \n", packet->timestamp);
    index += HS_LIDAR_QT_TIMESTAMP_SIZE;

    packet->echo = recvbuf[index] & 0xff;

    index += HS_LIDAR_QT_ECHO_SIZE;
    index += HS_LIDAR_QT_FACTORY_SIZE;

    packet->addtime[0] = recvbuf[index] & 0xff;
    packet->addtime[1] = recvbuf[index + 1] & 0xff;
    packet->addtime[2] = recvbuf[index + 2] & 0xff;
    packet->addtime[3] = recvbuf[index + 3] & 0xff;
    packet->addtime[4] = recvbuf[index + 4] & 0xff;
    packet->addtime[5] = recvbuf[index + 5] & 0xff;

    index += HS_LIDAR_QT_UTC_SIZE;

    return 0;
}

int PandarGeneral_Internal::ParseXTData(
    HS_LIDAR_XT_Packet *packet,
    const uint8_t *recvbuf,
    const int len) {
    if (len != HS_LIDAR_XT_PACKET_SIZE && len != HS_LIDAR_XT16_PACKET_SIZE) {
        std::cout << "packet size mismatch PandarXT " << len << "," << len << std::endl;
        return -1;
    }

    int index = 0;
    int block = 0;
    // Parse 12 Bytes Header
    packet->header.sob = (recvbuf[index] & 0xff) << 8 | ((recvbuf[index + 1] & 0xff));
    packet->header.chProtocolMajor = recvbuf[index + 2] & 0xff;
    packet->header.chProtocolMinor = recvbuf[index + 3] & 0xff;
    packet->header.chLaserNumber = recvbuf[index + 6] & 0xff;
    packet->header.chBlockNumber = recvbuf[index + 7] & 0xff;
    packet->header.chReturnType = recvbuf[index + 8] & 0xff;
    packet->header.chDisUnit = recvbuf[index + 9] & 0xff;
    index += HS_LIDAR_XT_HEAD_SIZE;

    if (packet->header.sob != 0xEEFF) {
        printf("Error Start of Packet!\n");
        return -1;
    }

    for (block = 0; block < packet->header.chBlockNumber; block++) {
        packet->blocks[block].azimuth =
            (recvbuf[index] & 0xff) | ((recvbuf[index + 1] & 0xff) << 8);
        index += HS_LIDAR_XT_BLOCK_HEADER_AZIMUTH;

        int unit;

        for (unit = 0; unit < packet->header.chLaserNumber; unit++) {
            unsigned int unRange = (recvbuf[index] & 0xff) | ((recvbuf[index + 1] & 0xff) << 8);

            packet->blocks[block].units[unit].distance =
                (static_cast<double>(unRange * packet->header.chDisUnit)) / (double)1000;
            packet->blocks[block].units[unit].intensity = (recvbuf[index + 2] & 0xff);
            packet->blocks[block].units[unit].confidence = (recvbuf[index + 3] & 0xff);
            index += HS_LIDAR_XT_UNIT_SIZE;
        }
    }

    index += HS_LIDAR_XT_RESERVED_SIZE; // skip reserved bytes

    packet->echo = recvbuf[index] & 0xff;

    index += HS_LIDAR_XT_ECHO_SIZE;
    index += HS_LIDAR_XT_ENGINE_VELOCITY;

    packet->addtime[0] = recvbuf[index] & 0xff;
    packet->addtime[1] = recvbuf[index + 1] & 0xff;
    packet->addtime[2] = recvbuf[index + 2] & 0xff;
    packet->addtime[3] = recvbuf[index + 3] & 0xff;
    packet->addtime[4] = recvbuf[index + 4] & 0xff;
    packet->addtime[5] = recvbuf[index + 5] & 0xff;

    index += HS_LIDAR_XT_UTC_SIZE;

    packet->timestamp = (recvbuf[index] & 0xff) | (recvbuf[index + 1] & 0xff) << 8 |
                        ((recvbuf[index + 2] & 0xff) << 16) | ((recvbuf[index + 3] & 0xff) << 24);
    // printf("timestamp %u \n", packet->timestamp);
    return 0;
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

void PandarGeneral_Internal::CalcL40PointXYZIT(
    HS_LIDAR_L40_Packet *pkt,
    int blockid,
    std::shared_ptr<PPointCloud> cld) {
    HS_LIDAR_L40_Block *block = &pkt->blocks[blockid];

    tm tTm;
    // UTC's year only include 0 - 99 year , which indicate 2000 to 2099.
    // and mktime's year start from 1900 which is 0. so we need add 100 year.
    tTm.tm_year = pkt->addtime[0] + 100;

    // in case of time error
    if (tTm.tm_year >= 200) {
        tTm.tm_year -= 100;
    }

    // UTC's month start from 1, but mktime only accept month from 0.
    tTm.tm_mon = pkt->addtime[1] - 1;
    tTm.tm_mday = pkt->addtime[2];
    tTm.tm_hour = pkt->addtime[3];
    tTm.tm_min = pkt->addtime[4];
    tTm.tm_sec = pkt->addtime[5];
    tTm.tm_isdst = 0;

    double unix_second = static_cast<double>(mktime(&tTm) + tz_second_);

    for (int i = 0; i < HS_LIDAR_L40_LASER_COUNT; ++i) {
        /* for all the units in a block */
        HS_LIDAR_L40_Unit &unit = block->units[i];
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

            if (pkt->echo == 0x39) {
                // dual return, block 0&1 (2&3 , 4*5 ...)'s timestamp is the same.
                point.timestamp =
                    point.timestamp -
                    (static_cast<double>(block40OffsetDual_[blockid] + laser40Offset_[i]) /
                     1000000.0f);
            } else {
                point.timestamp =
                    point.timestamp -
                    (static_cast<double>(block40OffsetSingle_[blockid] + laser40Offset_[i]) /
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

void PandarGeneral_Internal::CalcL64PointXYZIT(
    HS_LIDAR_L64_Packet *pkt,
    int blockid,
    char chLaserNumber,
    std::shared_ptr<PPointCloud> cld) {
    HS_LIDAR_L64_Block *block = &pkt->blocks[blockid];

    tm tTm;
    // UTC's year only include 0 - 99 year , which indicate 2000 to 2099.
    // and mktime's year start from 1900 which is 0. so we need add 100 year.
    tTm.tm_year = pkt->addtime[0] + 100;

    // in case of time error
    if (tTm.tm_year >= 200) {
        tTm.tm_year -= 100;
    }

    // UTC's month start from 1, but mktime only accept month from 0.
    tTm.tm_mon = pkt->addtime[1] - 1;
    tTm.tm_mday = pkt->addtime[2];
    tTm.tm_hour = pkt->addtime[3];
    tTm.tm_min = pkt->addtime[4];
    tTm.tm_sec = pkt->addtime[5];
    tTm.tm_isdst = 0;

    double unix_second = static_cast<double>(mktime(&tTm) + tz_second_);

    for (int i = 0; i < chLaserNumber; ++i) {
        /* for all the units in a block */
        HS_LIDAR_L64_Unit &unit = block->units[i];
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

            if (pkt->echo == 0x39) {
                // dual return, block 0&1 (2&3 , 4*5 ...)'s timestamp is the same.
                point.timestamp =
                    point.timestamp -
                    (static_cast<double>(block64OffsetDual_[blockid] + laser64Offset_[i]) /
                     1000000.0f);
            } else {
                point.timestamp =
                    point.timestamp -
                    (static_cast<double>(block64OffsetSingle_[blockid] + laser64Offset_[i]) /
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

// QT
void PandarGeneral_Internal::CalcQTPointXYZIT(
    HS_LIDAR_QT_Packet *pkt,
    int blockid,
    char chLaserNumber,
    std::shared_ptr<PPointCloud> cld) {
    HS_LIDAR_QT_Block *block = &pkt->blocks[blockid];

    tm tTm;
    tTm.tm_year = pkt->addtime[0] + 100;

    // in case of time error
    if (tTm.tm_year >= 200) {
        tTm.tm_year -= 100;
    }

    // UTC's month start from 1, but mktime only accept month from 0.
    tTm.tm_mon = pkt->addtime[1] - 1;
    tTm.tm_mday = pkt->addtime[2];
    tTm.tm_hour = pkt->addtime[3];
    tTm.tm_min = pkt->addtime[4];
    tTm.tm_sec = pkt->addtime[5];
    tTm.tm_isdst = 0;

    double unix_second = static_cast<double>(mktime(&tTm) + tz_second_);

    for (int i = 0; i < chLaserNumber; ++i) {
        /* for all the units in a block */
        HS_LIDAR_QT_Unit &unit = block->units[i];
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

            if (pkt->echo == 0x05) {
                // dual return, block 0&1 (2&3 , 4*5 ...)'s timestamp is the same.
                // dual:0x05, single:0x00
                point.timestamp =
                    point.timestamp +
                    (static_cast<double>(blockQTOffsetDual_[blockid] + laserQTOffset_[i]) /
                     1000000.0f);
            } else {
                point.timestamp =
                    point.timestamp +
                    (static_cast<double>(blockQTOffsetSingle_[blockid] + laserQTOffset_[i]) /
                     1000000.0f);
            }
        }

        point.ring = i;
        if (pcl_type_)
            PointCloudList[i].push_back(point);
        else
            cld->push_back(point);
    }
}

void PandarGeneral_Internal::CalcXTPointXYZIT(
    HS_LIDAR_XT_Packet *pkt,
    int blockid,
    char chLaserNumber,
    std::shared_ptr<PPointCloud> cld) {
    HS_LIDAR_XT_Block *block = &pkt->blocks[blockid];

    tm tTm;
    tTm.tm_year = pkt->addtime[0] + 100;

    // in case of time error
    if (tTm.tm_year >= 200) {
        tTm.tm_year -= 100;
    }

    // UTC's month start from 1, but mktime only accept month from 0.
    tTm.tm_mon = pkt->addtime[1] - 1;
    tTm.tm_mday = pkt->addtime[2];
    tTm.tm_hour = pkt->addtime[3];
    tTm.tm_min = pkt->addtime[4];
    tTm.tm_sec = pkt->addtime[5];
    tTm.tm_isdst = 0;

    double unix_second = static_cast<double>(mktime(&tTm) + tz_second_);

    for (int i = 0; i < chLaserNumber; ++i) {
        /* for all the units in a block */
        HS_LIDAR_XT_Unit &unit = block->units[i];
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

            if (pkt->echo == 0x39) {
                point.timestamp =
                    point.timestamp +
                    (static_cast<double>(blockXTOffsetDual_[blockid] + laserXTOffset_[i]) /
                     1000000.0f);
            } else {
                point.timestamp =
                    point.timestamp +
                    (static_cast<double>(blockXTOffsetSingle_[blockid] + laserXTOffset_[i]) /
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
