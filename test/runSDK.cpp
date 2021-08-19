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

#include <pcl/visualization/cloud_viewer.h>
#include <tbb/concurrent_queue.h>
#include <thread>

pcl::visualization::CloudViewer viewer("Simple");

tbb::concurrent_bounded_queue<std::vector<uint8_t>> con_queue;

double degToRad(double degree) { return degree * M_PI / 180; }

pcl::PointCloud<pcl::PointXYZ>::Ptr calcPointXYZI(const std::vector<uint8_t> &frame_buffer) {

    int index = 0;
    float dis_unit = 0.004;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    for (int azimuth_idx = 0; azimuth_idx < 1800; ++azimuth_idx) {
        for (int dual_mode_idx = 0; dual_mode_idx < 2; ++dual_mode_idx) {
            for (int laser_idx = 0; laser_idx < 64; ++laser_idx) {
                uint16_t raw_distance = frame_buffer[index] | frame_buffer[index + 1] << 8;
                uint8_t intensity = frame_buffer[index + 2];

                float distance = raw_distance * dis_unit;
                if (distance < 0.1 || distance > 200.0) {
                    index += 3;
                    continue;
                }

                /*
                float xyDistance = distance * cosf(degToRad(elev_angle_map_[i]));
                point.x = xyDistance *
                          sinf(degToRad(
                              azimuth_offset_map_[i] + static_cast<float>(block.azimuth) / 100.0));
                point.y = xyDistance *
                          cosf(degToRad(
                              azimuth_offset_map_[i] + static_cast<float>(block.azimuth) / 100.0));
                point.z = distance * sinf(degToRad(elev_angle_map_[i]));
                point.intensity = unit.intensity;
                */

                float x = distance * sinf(degToRad(azimuth_idx * 0.2));
                float y = distance * cosf(degToRad(azimuth_idx * 0.2));
                float z = 0;
                cloud->emplace_back(x, y, z);

                index += 3;
            }
        }
    }
    return cloud;
}

void gpsCallback(int timestamp) { printf("gps: %d\n", timestamp); }

void lidarCallback(const std::vector<uint8_t> &cld, double timestamp) {
    printf("timestamp: %lf,point_size: %ld\n", timestamp, cld.size());

    con_queue.try_push(cld);
}

int main() {

    con_queue.set_capacity(2);

    PandarGeneralSDK pandarGeneral(
        "", // "192.168.1.201",
        2368,
        10110,
        lidarCallback,
        gpsCallback,
        0,
        "Pandar64",
        "frame_id",
        "",
        10,
        true);
    pandarGeneral.Start();

    std::vector<uint8_t> cld;
    while (true) {
        con_queue.pop(cld);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = calcPointXYZI(cld);
        viewer.showCloud(cloud);
    }

    return 0;
}
