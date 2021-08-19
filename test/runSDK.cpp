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
#include "pandarGeneral_sdk/pandar_utils.h"

#include <pcl/visualization/cloud_viewer.h>
#include <tbb/concurrent_queue.h>
#include <thread>

pcl::visualization::CloudViewer viewer("Simple");

tbb::concurrent_bounded_queue<std::vector<uint8_t>> con_queue;

double degToRad(double degree) { return degree * M_PI / 180; }

pcl::PointCloud<pcl::PointXYZ>::Ptr calcPointXYZI(
    const std::vector<uint8_t> &frame_buffer,
    const bool dual_return_mode,
    const int num_lasers,
    const float azimuth_res_deg,
    std::vector<float> elev_angle,
    std::vector<float> azimuth_offset) {

    const float unit_distance = 0.004;
    const int num_returns = dual_return_mode ? 2 : 1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    if (elev_angle.size() != static_cast<size_t>(num_lasers) ||
        azimuth_offset.size() != static_cast<size_t>(num_lasers)) {
        std::cout << "Angle correction size problem\n";
        std::terminate();
    }

    for (int azimuth_idx = 0; azimuth_idx < 1800; ++azimuth_idx) {
        for (int dual_mode_idx = 0; dual_mode_idx < num_returns; ++dual_mode_idx) {
            for (int laser_idx = 0; laser_idx < num_lasers; ++laser_idx) {

                const int index =
                    (azimuth_idx * num_returns + dual_mode_idx) * num_lasers * 3 + laser_idx * 3;
                uint16_t raw_distance = frame_buffer[index] | frame_buffer[index + 1] << 8;
                uint8_t intensity = frame_buffer[index + 2];

                float distance = raw_distance * unit_distance;
                if (distance < 0.1 || distance > 200.0) {
                    continue;
                }

                float xyDistance = distance * cosf(degToRad(elev_angle[laser_idx]));
                float x =
                    xyDistance *
                    sinf(degToRad(azimuth_offset[laser_idx] + azimuth_idx * azimuth_res_deg));
                float y =
                    xyDistance *
                    cosf(degToRad(azimuth_offset[laser_idx] + azimuth_idx * azimuth_res_deg));
                float z = distance * sinf(degToRad(elev_angle[laser_idx]));

                cloud->emplace_back(x, y, z);
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

    std::vector<float> elev_angle;
    std::vector<float> azimuth_offset;
    if (auto ret = PandarUtils::getFallbackAngleCorrection("Pandar64")) {
        elev_angle = ret->first;
        azimuth_offset = ret->second;
    } else {
        std::cout << "Fail to get Angle Correction" << std::endl;
        std::terminate();
    }

    std::vector<uint8_t> cld;
    while (true) {
        con_queue.pop(cld);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud =
            calcPointXYZI(cld, true, 64, 0.2, elev_angle, azimuth_offset);
        viewer.showCloud(cloud);
    }

    return 0;
}
