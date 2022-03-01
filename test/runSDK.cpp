#include "pandar/pandar_config.h"
#include "pandar/pandar_lidar_receiver.h"
#include "pandar/pandar_utils.h"

#include <Eigen/Dense>
#include <pcl/visualization/cloud_viewer.h>
#include <tbb/concurrent_queue.h>
#include <thread>

#include <chrono>

using namespace std::chrono_literals;
using std::chrono::duration;
using std::chrono::system_clock;

pcl::visualization::CloudViewer viewer("Simple");

tbb::concurrent_bounded_queue<std::vector<uint8_t>> con_queue;

double degToRad(double degree) { return degree * M_PI / 180; }

class PCLConverter {
  private:
    Eigen::ArrayXf elev_angle_rad_sin_;
    Eigen::ArrayXf elev_angle_rad_cos_;

    Eigen::ArrayXf azimuth_map_rad_sin_;
    Eigen::ArrayXf azimuth_map_rad_cos_;

    const PandarConfig cfg_;

  public:
    PCLConverter(const PandarConfig &cfg) : cfg_(cfg) {

        const auto elev_angle_rad =
            Eigen::ArrayXf::Map(cfg.elev_angle().data(), cfg.elev_angle().size()) * M_PI / 180.f;

        elev_angle_rad_cos_ =
            elev_angle_rad.cos().replicate(cfg.num_returns() * cfg.frame_buffer_width(), 1);
        elev_angle_rad_sin_ =
            elev_angle_rad.sin().replicate(cfg.num_returns() * cfg.frame_buffer_width(), 1);

        const auto azimuth_offset_rad =
            Eigen::ArrayXf::Map(cfg.azimuth_offset().data(), cfg.azimuth_offset().size()) * M_PI /
            180.f;

        Eigen::ArrayXXf azimuth_rad = Eigen::ArrayXXf::Zero(1, cfg.frame_buffer_width());
        for (int i = 0; i < cfg.frame_buffer_width(); ++i) {
            azimuth_rad(0, i) = degToRad(cfg.azimuthIdxToAzimuth(i) / 100.0f);
        }

        const auto azimuth_map_rad =
            azimuth_rad.replicate(cfg.num_lasers(), 1).colwise() + azimuth_offset_rad;
        azimuth_map_rad_sin_ = azimuth_map_rad.sin().replicate(cfg.num_returns(), 1).reshaped();
        azimuth_map_rad_cos_ = azimuth_map_rad.cos().replicate(cfg.num_returns(), 1).reshaped();
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr calcPointXYZ(const std::vector<uint8_t> &frame_buffer) {

        const auto raw_distance_lower =
            Eigen::Map<const Eigen::ArrayX<uint8_t>, Eigen::Unaligned, Eigen::InnerStride<3>>{
                frame_buffer.data(),
                cfg_.frame_buffer_elems()};

        const auto raw_distance_upper =
            Eigen::Map<const Eigen::ArrayX<uint8_t>, Eigen::Unaligned, Eigen::InnerStride<3>>{
                frame_buffer.data() + 1,
                cfg_.frame_buffer_elems()};

        const auto distance =
            (raw_distance_lower.cast<float>() + raw_distance_upper.cast<float>() * 256) *
            cfg_.unit_distance;

        const auto xyDistance = distance * elev_angle_rad_cos_;

        const auto x = xyDistance * azimuth_map_rad_sin_;
        const auto y = xyDistance * azimuth_map_rad_cos_;
        const auto z = distance * elev_angle_rad_sin_;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud =
            pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        const auto is_valid_distance = ((distance > 0.1) * (distance < 200.0)).cast<int>();
        cloud->reserve(is_valid_distance.sum());
        for (int i = 0; i < distance.size(); ++i) {
            if (is_valid_distance[i] > 0) {
                cloud->emplace_back(x[i], y[i], z[i]);
            }
        }
        return cloud;
    }
};

void gpsCallback(int timestamp) { std::cout << "timestamp: " << timestamp << "\n"; }

void lidarCallback(const std::vector<uint8_t> &cld, time_point<system_clock> &timestamp) {
    std::cout << "timestamp: "
              << std::chrono::duration_cast<std::chrono::microseconds>(
                     timestamp.time_since_epoch())
                     .count()
              << "\n";

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
        "127.0.0.1", // "192.168.1.201",
        2368,
        lidarCallback,
        pandar_config);
    pandarGeneral.start();

    std::vector<uint8_t> cld;
    PCLConverter converter{pandar_config};
    while (true) {
        con_queue.pop(cld);
        system_clock::time_point start = system_clock::now();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = converter.calcPointXYZ(cld);
        const duration<double, std::milli> due = system_clock::now() - start;
        std::cout << due.count() << "\n";
        viewer.showCloud(cloud);
    }

    return 0;
}
