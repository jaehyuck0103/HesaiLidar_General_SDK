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

#include "pandarGeneral_sdk/point_types.h"

#include <functional>
#include <memory>
#include <string>

class PandarGeneral_Internal;

class PandarGeneralSDK {
  public:
    /**
     * @brief Constructor
     * @param device_ip  		The ip of the device
     *        lidar_port 		The port number of lidar data
     *        gps_port   		The port number of gps data
     *        pcl_callback      The callback of PCL data structure
     *        gps_callback      The callback of GPS structure
     *        start_angle       The start angle of every point cloud
     *                          should be <real angle> * 100.
     *        lidar_type        The model of the lidar
     *        frame_id          The id of the point cloud data published to ROS
     */
    PandarGeneralSDK(
        std::string device_ip,
        const uint16_t lidar_port,
        const uint16_t gps_port,
        std::string pcap_path,
        std::function<void(std::vector<PointXYZIT>, double)> pcl_callback,
        std::function<void(double)> gps_callback,
        uint16_t start_angle,
        int tz,
        std::string lidar_type,
        std::string frame_id,
        std::string timestampType); // the default timestamp type is LiDAR time

    ~PandarGeneralSDK();

    bool updateAngleCorrectionByFile(std::string correction_path);
    std::optional<std::string> getAngleCorrectionFromDevice(const std::string &device_ip);

    void Start();
    void Stop();

  private:
    std::unique_ptr<PandarGeneral_Internal> internal_;
};
