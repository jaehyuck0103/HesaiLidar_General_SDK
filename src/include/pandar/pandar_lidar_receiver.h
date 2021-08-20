#pragma once

#include "pandarGeneral_sdk/pandar_config.h"

#include <asio.hpp>

#include <functional>
#include <string>
#include <thread>
#include <vector>

using asio::ip::udp;

inline double parseUTC(const uint8_t *buf) {
    std::tm tTm;
    tTm.tm_year = buf[0] + 100; // years since 1900
    tTm.tm_mon = buf[1] - 1;    // years since 1900
    tTm.tm_mday = buf[2];
    tTm.tm_hour = buf[3];
    tTm.tm_min = buf[4];
    tTm.tm_sec = buf[5];
    tTm.tm_isdst = 0;

    return static_cast<double>(std::mktime(&tTm));
}

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
        std::function<void(const std::vector<uint8_t> &, double)> pcl_callback,
        const PandarConfig &cfg);

    virtual ~PandarLidarReceiver();

    void start();
    void stop();

  private:
    void processLidarPacket(const std::vector<uint8_t> &packet);

    const std::function<void(const std::vector<uint8_t> &cld, double timestamp)> pcl_callback_;

    std::vector<uint8_t> frame_buffer_;

  protected:
    const PandarConfig cfg_;

    std::vector<float> blockOffsetSingle_;
    std::vector<float> blockOffsetDual_;
    std::vector<float> laserOffset_;

    bool isValidAzimuth(uint16_t azimuth) {
        if (azimuth < 36000 && azimuth % cfg_.azimuth_res() == 0) {
            return true;
        } else {
            return false;
        }
    }

    virtual std::optional<HS_LIDAR_Packet>
    parseLidarPacket(const std::vector<uint8_t> &packet) = 0;

  private: // asio
    asio::io_context io_context_;
    udp::socket socket_;

    std::unique_ptr<std::thread> context_thr_;
    bool enable_recv_flag_ = false;

    std::vector<uint8_t> recv_buffer_;

    udp::endpoint remote_endpoint_;

    void socketRecvHandler();
};
