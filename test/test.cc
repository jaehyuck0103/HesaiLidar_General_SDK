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

pcl::visualization::CloudViewer viewer("Simple");

void gpsCallback(int timestamp) { printf("gps: %d\n", timestamp); }

void lidarCallback(std::vector<PointXYZIT> cld, double timestamp) {
    printf("timestamp: %lf,point_size: %ld\n", timestamp, cld.size());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (const auto &pt : cld) {
        cloud->emplace_back(pt.x, pt.y, pt.z);
    }

    viewer.showCloud(cloud);
}

int main() {
    /*
    PandarGeneralSDK pandarGeneral(
        std::string("192.168.1.201"),
        2368,
        10110,
        lidarCallback,
        gpsCallback,
        0,
        0,
        std::string("PandarXT-32"),
        std::string("frame_id"),
        "");
    */

    PandarGeneralSDK pandarGeneral(
        "",
        0,
        0,
        "../test/test_data/Turning_Left_Pandar64.pcap",
        lidarCallback,
        nullptr,
        0,
        0,
        "Pandar64",
        "frame_id",
        "");

    std::string correctionFilePath = "../test/test_data/angle_correction_Pandar64.csv";
    if (!pandarGeneral.updateAngleCorrectionByFile(correctionFilePath)) {
        std::terminate();
    }
    pandarGeneral.Start();

    while (true) {
        sleep(100);
    }

    return 0;
}
