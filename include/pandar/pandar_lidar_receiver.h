#pragma once

#include "pandar/pandar_config.h"

#include <asio.hpp>

#include <functional>
#include <string>
#include <thread>
#include <vector>

using asio::ip::udp;
using std::chrono::system_clock;
using std::chrono::time_point;

struct HS_LIDAR_Block {
    uint16_t azimuth;             // Azimuth = RealAzimuth * 100
    std::vector<uint8_t> payload; // num_lasers * 3 bytes (2bytes raw_distance + 1byte intensity)
};

struct HS_LIDAR_Packet {
    std::vector<HS_LIDAR_Block> blocks;
    double timestamp;
};

class PandarLidarReceiver {
  public:
    PandarLidarReceiver(
        uint16_t lidar_port,
        std::function<void(const std::vector<uint8_t> &, time_point<system_clock> &)> pcl_callback,
        const PandarConfig &cfg);

    virtual ~PandarLidarReceiver();

    void start();
    void stop();

  private:
    std::vector<uint8_t> frame_buffer_;
    const std::function<void(const std::vector<uint8_t> &cld, time_point<system_clock> &timestamp)>
        pcl_callback_;

    const PandarConfig cfg_;

    void processLidarPacket(const std::vector<uint8_t> &packet);
    std::function<std::optional<HS_LIDAR_Packet>(const std::vector<uint8_t> &packet)>
        parseLidarPacket;

    bool isValidAzimuth(uint16_t azimuth) {
        if (azimuth < 36000 && azimuth % cfg_.azimuth_res() == 0) {
            return true;
        } else {
            return false;
        }
    }

    time_point<system_clock> timestamp_ = system_clock::time_point::min();

  private: // asio
    asio::io_context io_context_;
    udp::socket socket_;

    std::unique_ptr<std::thread> context_thr_;
    bool enable_recv_flag_ = false;

    std::vector<uint8_t> recv_buffer_;

    udp::endpoint remote_endpoint_;

    void socketRecvHandler();
};
