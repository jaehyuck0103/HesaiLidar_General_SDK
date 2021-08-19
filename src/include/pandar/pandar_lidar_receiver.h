#pragma once

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
        uint16_t start_azimuth,
        std::string lidar_type,
        std::string frame_id,
        std::string timestampType,
        int fps,
        bool dualReturnMode);

    virtual ~PandarLidarReceiver();

    void Start();
    void Stop();

  private:
    void processLidarPacket(const std::vector<uint8_t> &packet);

    std::function<void(const std::vector<uint8_t> &cld, double timestamp)> pcl_callback_;

    std::string frame_id_;

    std::vector<uint8_t> frame_buffer_;

  private: // asio
    asio::io_context io_context_;
    udp::socket lidarRcvSocket_;

    std::unique_ptr<std::thread> contextThr_;
    bool enableRecvThr_ = false;

    std::vector<uint8_t> lidarRcvBuffer_;

    udp::endpoint lidarRemoteEndpoint_;

    void rcvLidarHandler();

  protected:
    std::vector<float> blockOffsetSingle_;
    std::vector<float> blockOffsetDual_;
    std::vector<float> laserOffset_;

    std::string m_sTimestampType;
    std::string m_sLidarType;

    bool isValidAzimuth(uint16_t azimuth) {
        if (azimuth < 36000 && azimuth % azimuth_res_ == 0) {
            return true;
        } else {
            return false;
        }
    }

    uint16_t start_azimuth_;
    uint16_t azimuth_res_;
    int fps_;

    virtual std::optional<HS_LIDAR_Packet>
    parseLidarPacket(const std::vector<uint8_t> &packet) = 0;

    int num_lasers_ = 0;
    bool dualReturnMode_;
};
