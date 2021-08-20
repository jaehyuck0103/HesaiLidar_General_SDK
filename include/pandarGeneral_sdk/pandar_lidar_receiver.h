#pragma once

#include "pandarGeneral_sdk//pandar_packet_parsers.h"

#include <asio.hpp>

#include <functional>
#include <string>
#include <thread>
#include <vector>

using asio::ip::udp;

class PandarLidarReceiver {
  public:
    PandarLidarReceiver(
        uint16_t lidar_port,
        std::function<void(const std::vector<uint8_t> &, double)> pcl_callback,
        const PandarConfig &cfg);

    virtual ~PandarLidarReceiver();

    void start();
    void stop();

  private:
    std::vector<uint8_t> frame_buffer_;
    const std::function<void(const std::vector<uint8_t> &cld, double timestamp)> pcl_callback_;
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

  private: // asio
    asio::io_context io_context_;
    udp::socket socket_;

    std::unique_ptr<std::thread> context_thr_;
    bool enable_recv_flag_ = false;

    std::vector<uint8_t> recv_buffer_;

    udp::endpoint remote_endpoint_;

    void socketRecvHandler();
};
