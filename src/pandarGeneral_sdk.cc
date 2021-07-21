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
#include "tcp_command_client.h"

#include <cstdlib>

constexpr int PANDAR_TCP_COMMAND_PORT = 9347;

PandarGeneralSDK::PandarGeneralSDK(
    std::string device_ip,
    const uint16_t lidar_port,
    uint16_t lidar_algorithm_port,
    uint16_t gps_port,
    std::function<void(std::shared_ptr<PPointCloud>, double)> pcl_callback,
    std::function<void(HS_Object3D_Object_List *)> algorithm_callback,
    std::function<void(double)> gps_callback,
    uint16_t start_angle,
    int tz,
    int pcl_type,
    std::string lidar_type,
    std::string frame_id,
    std::string timestampType) {

    pandarGeneral_ = std::make_unique<PandarGeneral>(
        device_ip,
        lidar_port,
        lidar_algorithm_port,
        gps_port,
        pcl_callback,
        algorithm_callback,
        gps_callback,
        start_angle,
        tz,
        pcl_type,
        lidar_type,
        frame_id,
        timestampType);

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
    std::function<void(std::shared_ptr<PPointCloud>, double)> pcl_callback,
    uint16_t start_angle,
    int tz,
    int pcl_type,
    std::string lidar_type,
    std::string frame_id,
    std::string timestampType) {

    pandarGeneral_ = std::make_unique<PandarGeneral>(
        pcap_path,
        pcl_callback,
        start_angle,
        tz,
        pcl_type,
        lidar_type,
        frame_id,
        timestampType);
}

PandarGeneralSDK::~PandarGeneralSDK() { Stop(); }

int PandarGeneralSDK::LoadLidarCorrectionFile(std::string file) {
    return pandarGeneral_->LoadCorrectionFile(file);
}

void PandarGeneralSDK::ResetLidarStartAngle(uint16_t start_angle) {
    pandarGeneral_->ResetStartAngle(start_angle);
}

std::string PandarGeneralSDK::GetLidarCalibration() { return correction_content_; }

void PandarGeneralSDK::Start() {
    Stop();
    pandarGeneral_->Start();
}
void PandarGeneralSDK::Stop() { pandarGeneral_->Stop(); }

int PandarGeneralSDK::getMajorVersion() { return pandarGeneral_->getMajorVersion(); }
int PandarGeneralSDK::getMinorVersion() { return pandarGeneral_->getMinorVersion(); }

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

    int ret = pandarGeneral_->LoadCorrectionFile(correction_content);
    if (ret != 0) {
        std::cout << "Fail to parse lidar correction\n";
        return false;
    } else {
        std::cout << "Parse Lidar Correction Success\n";
        correction_content_ = correction_content;
        return true;
    }
}
