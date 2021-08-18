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

tbb::concurrent_bounded_queue<std::vector<PointXYZIT>> con_queue;

void gpsCallback(int timestamp) { printf("gps: %d\n", timestamp); }

void lidarCallback(const std::vector<PointXYZIT> &cld, double timestamp) {
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
        "");
    pandarGeneral.Start();

    std::vector<PointXYZIT> cld;
    while (true) {
        con_queue.pop(cld);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud =
            pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        for (const auto &pt : cld) {
            cloud->emplace_back(pt.x, pt.y, pt.z);
        }
        viewer.showCloud(cloud);
    }

    return 0;
}
