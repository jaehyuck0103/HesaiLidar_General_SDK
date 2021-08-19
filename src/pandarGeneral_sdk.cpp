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
    std::function<void(const std::vector<uint8_t> &, double)> pcl_callback,
    std::function<void(double)> gps_callback,
    uint16_t start_azimuth,
    std::string lidar_type,
    std::string frame_id,
    std::string timestampType,
    int fps,
    bool dualReturnMode) {

    if (lidar_type == "Pandar40P") {
        internal_ = std::make_unique<Pandar40P>(
            lidar_port,
            gps_port,
            pcl_callback,
            gps_callback,
            start_azimuth,
            lidar_type,
            frame_id,
            timestampType,
            fps,
            dualReturnMode);
    } else if (lidar_type == "Pandar64") {
        internal_ = std::make_unique<Pandar64>(
            lidar_port,
            gps_port,
            pcl_callback,
            gps_callback,
            start_azimuth,
            lidar_type,
            frame_id,
            timestampType,
            fps,
            dualReturnMode);
    } else if (lidar_type == "PandarQT") {
        internal_ = std::make_unique<PandarQT>(
            lidar_port,
            gps_port,
            pcl_callback,
            gps_callback,
            start_azimuth,
            lidar_type,
            frame_id,
            timestampType,
            fps,
            dualReturnMode);
    } else if (lidar_type == "PandarXT-32" || lidar_type == "PandarXT-16") {
        internal_ = std::make_unique<PandarXT>(
            lidar_port,
            gps_port,
            pcl_callback,
            gps_callback,
            start_azimuth,
            lidar_type,
            frame_id,
            timestampType,
            fps,
            dualReturnMode);
    } else {
        std::terminate();
    }
}

PandarGeneralSDK::~PandarGeneralSDK() { Stop(); }

void PandarGeneralSDK::Start() {
    Stop();
    internal_->Start();
}

void PandarGeneralSDK::Stop() { internal_->Stop(); }
