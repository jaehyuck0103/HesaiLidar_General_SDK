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

#include "pandarGeneral_sdk/pandarGeneral_sdk.h"
#include "pandar/pandar40P.h"
#include "pandar/pandar64.h"
#include "pandar/pandarQT.h"
#include "pandar/pandarXT.h"
#include "pandar/tcp_command_client.h"

#include <cstdlib>
#include <iostream>

constexpr int PANDAR_TCP_COMMAND_PORT = 9347;

PandarGeneralSDK::PandarGeneralSDK(
    std::string device_ip,
    const uint16_t lidar_port,
    uint16_t gps_port,
    std::function<void(std::vector<PointXYZIT>, double)> pcl_callback,
    std::function<void(double)> gps_callback,
    uint16_t start_angle,
    int tz,
    std::string lidar_type,
    std::string frame_id,
    std::string timestampType) {

    if (lidar_type == "Pandar40P") {
        internal_ = std::make_unique<Pandar40P>(
            lidar_port,
            gps_port,
            pcl_callback,
            gps_callback,
            start_angle,
            tz,
            lidar_type,
            frame_id,
            timestampType);
    } else if (lidar_type == "Pandar64") {
        internal_ = std::make_unique<Pandar64>(
            lidar_port,
            gps_port,
            pcl_callback,
            gps_callback,
            start_angle,
            tz,
            lidar_type,
            frame_id,
            timestampType);
    } else if (lidar_type == "PandarQT") {
        internal_ = std::make_unique<PandarQT>(
            lidar_port,
            gps_port,
            pcl_callback,
            gps_callback,
            start_angle,
            tz,
            lidar_type,
            frame_id,
            timestampType);
    } else if (lidar_type == "PandarXT-32" || lidar_type == "PandarXT-16") {
        internal_ = std::make_unique<PandarXT>(
            lidar_port,
            gps_port,
            pcl_callback,
            gps_callback,
            start_angle,
            tz,
            lidar_type,
            frame_id,
            timestampType);
    } else {
        abort();
    }

    // Try to get calibration.
    for (int i = 0; i < 5; ++i) {
        if (getCalibrationFromDevice(device_ip)) {
            return;
        }
    }
    abort();
}

PandarGeneralSDK::PandarGeneralSDK(
    std::string pcap_path,
    std::function<void(std::vector<PointXYZIT>, double)> pcl_callback,
    uint16_t start_angle,
    int tz,
    std::string lidar_type,
    std::string frame_id,
    std::string timestampType) {

    if (lidar_type == "Pandar40P") {
        internal_ = std::make_unique<Pandar40P>(
            pcap_path,
            pcl_callback,
            start_angle,
            tz,
            lidar_type,
            frame_id,
            timestampType);
    } else if (lidar_type == "Pandar64") {
        internal_ = std::make_unique<Pandar64>(
            pcap_path,
            pcl_callback,
            start_angle,
            tz,
            lidar_type,
            frame_id,
            timestampType);
    } else if (lidar_type == "PandarQT") {
        internal_ = std::make_unique<PandarQT>(
            pcap_path,
            pcl_callback,
            start_angle,
            tz,
            lidar_type,
            frame_id,
            timestampType);
    } else if (lidar_type == "PandarXT-32" || lidar_type == "PandarXT-16") {
        internal_ = std::make_unique<PandarXT>(
            pcap_path,
            pcl_callback,
            start_angle,
            tz,
            lidar_type,
            frame_id,
            timestampType);
    } else {
        abort();
    }
}

PandarGeneralSDK::~PandarGeneralSDK() { Stop(); }

int PandarGeneralSDK::LoadLidarCorrectionFile(std::string file) {
    return internal_->LoadCorrectionFile(file);
}

std::string PandarGeneralSDK::GetLidarCalibration() { return correction_content_; }

void PandarGeneralSDK::Start() {
    Stop();
    internal_->Start();
}
void PandarGeneralSDK::Stop() { internal_->Stop(); }

bool PandarGeneralSDK::getCalibrationFromDevice(const std::string &device_ip) {

    // Dual Check
    auto [ec, feedback] = TcpCommand::getLidarCalibration(device_ip, PANDAR_TCP_COMMAND_PORT);
    auto [ec2, feedback2] = TcpCommand::getLidarCalibration(device_ip, PANDAR_TCP_COMMAND_PORT);

    if (ec != TcpCommand::PTC_ErrCode::NO_ERROR || ec2 != TcpCommand::PTC_ErrCode::NO_ERROR) {
        std::cout << "Fail to get correction content\n";
        return false;
    }

    std::string correction_content(feedback.payload.begin(), feedback.payload.end());
    std::string correction_content2(feedback2.payload.begin(), feedback2.payload.end());
    if (correction_content != correction_content2) {
        std::cout << "Fail to pass the dual check\n";
        return false;
    }

    int ret = internal_->LoadCorrectionFile(correction_content);
    if (ret != 0) {
        std::cout << "Fail to parse lidar correction\n";
        return false;
    } else {
        std::cout << "Parse Lidar Correction Success\n";
        correction_content_ = correction_content;
        return true;
    }
}
