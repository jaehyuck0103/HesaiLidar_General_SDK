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

PandarGeneralSDK::PandarGeneralSDK(
    std::string device_ip,
    const uint16_t lidar_port,
    std::function<void(const std::vector<uint8_t> &, double)> pcl_callback,
    uint16_t start_azimuth,
    std::string lidar_type,
    std::string frame_id,
    std::string timestampType,
    int fps,
    bool dual_return_mode) {

    if (lidar_type == "Pandar40P") {
        internal_ = std::make_unique<Pandar40P>(
            lidar_port,
            pcl_callback,
            start_azimuth,
            lidar_type,
            frame_id,
            timestampType,
            fps,
            dual_return_mode);
    } else if (lidar_type == "Pandar64") {
        internal_ = std::make_unique<Pandar64>(
            lidar_port,
            pcl_callback,
            start_azimuth,
            lidar_type,
            frame_id,
            timestampType,
            fps,
            dual_return_mode);
    } else if (lidar_type == "PandarQT") {
        internal_ = std::make_unique<PandarQT>(
            lidar_port,
            pcl_callback,
            start_azimuth,
            lidar_type,
            frame_id,
            timestampType,
            fps,
            dual_return_mode);
    } else if (lidar_type == "PandarXT-32" || lidar_type == "PandarXT-16") {
        internal_ = std::make_unique<PandarXT>(
            lidar_port,
            pcl_callback,
            start_azimuth,
            lidar_type,
            frame_id,
            timestampType,
            fps,
            dual_return_mode);
    } else {
        std::terminate();
    }
}

PandarGeneralSDK::~PandarGeneralSDK() { stop(); }

void PandarGeneralSDK::start() {
    stop();
    internal_->start();
}

void PandarGeneralSDK::stop() { internal_->stop(); }
