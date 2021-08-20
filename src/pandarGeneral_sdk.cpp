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
    const PandarConfig &cfg) {

    switch (cfg.lidar_model()) {
    case LidarModel::Pandar40P:
        internal_ = std::make_unique<Pandar40P>(lidar_port, pcl_callback, cfg);
        break;
    case LidarModel::Pandar64:
        internal_ = std::make_unique<Pandar64>(lidar_port, pcl_callback, cfg);
        break;
    case LidarModel::PandarQT:
        internal_ = std::make_unique<PandarQT>(lidar_port, pcl_callback, cfg);
        break;
    case LidarModel::PandarXT_16:
    case LidarModel::PandarXT_32:
        internal_ = std::make_unique<PandarXT>(lidar_port, pcl_callback, cfg);
        break;
    }
}

PandarGeneralSDK::~PandarGeneralSDK() { stop(); }

void PandarGeneralSDK::start() {
    stop();
    internal_->start();
}

void PandarGeneralSDK::stop() { internal_->stop(); }
