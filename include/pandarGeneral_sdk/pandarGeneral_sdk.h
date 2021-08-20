#pragma once

#include "pandarGeneral_sdk/pandar_config.h"

#include <functional>
#include <memory>
#include <string>

class PandarLidarReceiver;

class PandarGeneralSDK {
  public:
    PandarGeneralSDK(
        std::string device_ip,
        const uint16_t lidar_port,
        std::function<void(const std::vector<uint8_t> &, double)> pcl_callback,
        const PandarConfig &cfg);

    ~PandarGeneralSDK();

    void start();
    void stop();

  private:
    std::unique_ptr<PandarLidarReceiver> internal_;
};
