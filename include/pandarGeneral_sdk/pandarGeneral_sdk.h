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

#include "pandarGeneral/pandarGeneral_internal.h"
#include "pandarGeneral/point_types.h"

#include <functional>
#include <string>

class PandarGeneralSDK {
  public:
    /**
     * @brief Constructor
     * @param device_ip  				The ip of the device
     *        lidar_port 				The port number of lidar data
     *        gps_port   				The port number of gps data
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
        uint16_t lidar_algorithm_port,
        const uint16_t gps_port,
        std::function<void(std::shared_ptr<PPointCloud>, double)> pcl_callback,
        std::function<void(HS_Object3D_Object_List *)> algorithm_callback,
        std::function<void(double)> gps_callback,
        uint16_t start_angle,
        int tz,
        int pcl_type,
        std::string lidar_type,
        std::string frame_id,
        std::string timestampType); // the default timestamp type is LiDAR time
    /**
     * @brief Constructor
     * @param pcap_path         The path of pcap file
     *        pcl_callback      The callback of PCL data structure
     *        start_angle       The start angle of every point cloud
     *                          should be <real angle> * 100.
     *        tz                The timezone
     *        lidar_type        The model of the lidar
     *        frame_id          The id of the point cloud data published to ROS
     */
    PandarGeneralSDK(
        std::string pcap_path,
        std::function<void(std::shared_ptr<PPointCloud>, double)> pcl_callback,
        uint16_t start_angle,
        int tz,
        int pcl_type,
        std::string lidar_type,
        std::string frame_id,
        std::string timestampType); // the default timestamp type is LiDAR time
    ~PandarGeneralSDK();

    int LoadLidarCorrectionFile(std::string correction_content);
    void ResetLidarStartAngle(uint16_t start_angle);
    std::string GetLidarCalibration();
    void Start();
    void Stop();

    int getMajorVersion();
    int getMinorVersion();

  private:
    std::unique_ptr<PandarGeneral_Internal> internal_;
    std::string correction_content_;
    bool getCalibrationFromDevice(const std::string &device_ip);
};
