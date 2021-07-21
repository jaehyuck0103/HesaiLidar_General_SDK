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
#include "log.h"
#include "tcp_command_client.h"
#include "version.h"
#include "yaml-cpp/yaml.h"

constexpr int PANDAR_GENERALSDK_TCP_COMMAND_PORT = 9347;

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

    printVersion();
    pandarGeneral_ = NULL;

    pandarGeneral_ = new PandarGeneral(
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

    get_calibration_thr_ = NULL;
    enable_get_calibration_thr_ = false;
    got_lidar_calibration_ = false;
    device_ip_ = device_ip;
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
    printVersion();
    pandarGeneral_ = NULL;

    pandarGeneral_ = new PandarGeneral(
        pcap_path,
        pcl_callback,
        start_angle,
        tz,
        pcl_type,
        lidar_type,
        frame_id,
        timestampType);

    get_calibration_thr_ = NULL;
    enable_get_calibration_thr_ = false;
    got_lidar_calibration_ = false;
}

PandarGeneralSDK::~PandarGeneralSDK() {
    Stop();
    if (pandarGeneral_) {
        delete pandarGeneral_;
    }
}

/**
 * @brief load the correction file
 * @param file The path of correction file
 */
int PandarGeneralSDK::LoadLidarCorrectionFile(std::string correction_content) {
    return pandarGeneral_->LoadCorrectionFile(correction_content);
}

/**
 * @brief load the correction file
 * @param angle The start angle
 */
void PandarGeneralSDK::ResetLidarStartAngle(uint16_t start_angle) {
    if (!pandarGeneral_)
        return;
    pandarGeneral_->ResetStartAngle(start_angle);
}

std::string PandarGeneralSDK::GetLidarCalibration() { return correction_content_; }

int PandarGeneralSDK::Start() {
    // LOG_FUNC();
    Stop();

    if (pandarGeneral_) {
        pandarGeneral_->Start();
    }

    enable_get_calibration_thr_ = true;
    get_calibration_thr_ =
        new std::thread(std::bind(&PandarGeneralSDK::GetCalibrationFromDevice, this));
}

void PandarGeneralSDK::Stop() {
    if (pandarGeneral_)
        pandarGeneral_->Stop();

    enable_get_calibration_thr_ = false;
    if (get_calibration_thr_) {
        get_calibration_thr_->join();
    }
}

void PandarGeneralSDK::GetCalibrationFromDevice() {
    // LOG_FUNC();
    if (device_ip_.empty()) {
        return;
    }

    while (enable_get_calibration_thr_ && !got_lidar_calibration_) {
        if (!got_lidar_calibration_) {
            // get lidar calibration.
            auto [ec, feedback] =
                TcpCommand::getLidarCalibration(device_ip_, PANDAR_GENERALSDK_TCP_COMMAND_PORT);
            if (ec == TcpCommand::PTC_ErrCode::NO_ERROR) {
                // success;
                std::string correction_content(feedback.payload.begin(), feedback.payload.end());
                if (pandarGeneral_) {
                    int ret = pandarGeneral_->LoadCorrectionFile(correction_content);
                    if (ret != 0) {
                        std::cout << "Parse Lidar Correction Error" << std::endl;
                        got_lidar_calibration_ = false;
                    } else {
                        std::cout << "Parse Lidar Correction Success!!!" << std::endl;
                        correction_content_ = correction_content;
                        got_lidar_calibration_ = true;
                    }
                }
            }
        }

        sleep(1);
    }
}

int PandarGeneralSDK::getMajorVersion() {
    if (pandarGeneral_) {
        pandarGeneral_->getMajorVersion();
    }
}

int PandarGeneralSDK::getMinorVersion() {
    if (pandarGeneral_) {
        pandarGeneral_->getMinorVersion();
    }
}
