#include "pandarGeneral_sdk/pandar_config.h"
#include "pandarGeneral_sdk/pandar_lidar_receiver.h"
#include "pandarGeneral_sdk/pandar_utils.h"

#include <pcl/visualization/cloud_viewer.h>
#include <tbb/concurrent_queue.h>
#include <thread>

pcl::visualization::CloudViewer viewer("Simple");

tbb::concurrent_bounded_queue<std::vector<uint8_t>> con_queue;

double degToRad(double degree) { return degree * M_PI / 180; }

pcl::PointCloud<pcl::PointXYZ>::Ptr
calcPointXYZI(const std::vector<uint8_t> &frame_buffer, const PandarConfig &cfg) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    for (int azimuth_idx = 0; azimuth_idx < cfg.frame_buffer_width(); ++azimuth_idx) {
        for (int return_idx = 0; return_idx < cfg.num_returns(); ++return_idx) {
            for (int laser_idx = 0; laser_idx < cfg.num_lasers(); ++laser_idx) {

                const int bytes_index =
                    cfg.get_frame_buffer_bytes_index(azimuth_idx, return_idx, laser_idx);

                uint16_t raw_distance = frame_buffer[bytes_index] | frame_buffer[bytes_index + 1]
                                                                        << 8;
                // uint8_t intensity = frame_buffer[bytes_index + 2];

                float distance = raw_distance * cfg.unit_distance;
                if (distance < 0.1 || distance > 200.0) {
                    continue;
                }

                float xyDistance = distance * cosf(degToRad(cfg.elev_angle()[laser_idx]));
                float x =
                    xyDistance *
                    sinf(degToRad(
                        cfg.azimuth_offset()[laser_idx] + azimuth_idx * cfg.azimuth_res_deg()));
                float y =
                    xyDistance *
                    cosf(degToRad(
                        cfg.azimuth_offset()[laser_idx] + azimuth_idx * cfg.azimuth_res_deg()));
                float z = distance * sinf(degToRad(cfg.elev_angle()[laser_idx]));

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

    auto angle_corrections = PandarUtils::getFallbackAngleCorrection("Pandar64");
    if (!angle_corrections.has_value()) {
        std::cout << "Fail to get Angle Correction" << std::endl;
        std::abort();
    }

    PandarConfig pandar_config{
        LidarModel::Pandar64,
        true,
        10,
        0,
        angle_corrections->first,
        angle_corrections->second};

    PandarLidarReceiver pandarGeneral(
        // "", // "192.168.1.201",
        2368,
        lidarCallback,
        pandar_config);
    pandarGeneral.start();

    std::vector<uint8_t> cld;
    while (true) {
        con_queue.pop(cld);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = calcPointXYZI(cld, pandar_config);
        viewer.showCloud(cloud);
    }

    return 0;
}
