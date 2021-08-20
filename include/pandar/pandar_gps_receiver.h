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

struct PandarGpsPacket {
    uint16_t flag;
    uint16_t year;
    uint16_t month;
    uint16_t day;
    uint16_t second;
    uint16_t minute;
    uint16_t hour;
    uint32_t fineTime;
};

class PandarGpsReceiver {
  public:
    PandarGpsReceiver(
        uint16_t recv_port,
        std::function<void(double)> gps_callback,
        std::string frame_id);

    virtual ~PandarGpsReceiver();

    void Start();
    void Stop();

  private:
    void processGps(const std::vector<uint8_t> &packet);

    std::optional<PandarGpsPacket> parseGPS(const std::vector<uint8_t> &recvbuf);

    std::function<void(double timestamp)> gps_callback_;

    std::string frame_id_;

  private: // asio
    asio::io_context io_context_;
    udp::socket gpsRcvSocket_;

    std::unique_ptr<std::thread> contextThr_;
    bool enableRecvThr_ = false;

    std::vector<uint8_t> gpsRcvBuffer_;

    udp::endpoint gpsRemoteEndpoint_;

    void rcvGpsHandler();
};
