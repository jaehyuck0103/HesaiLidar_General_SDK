#include "pandar/pandar_gps_receiver.h"

#include <iostream>

PandarGpsReceiver::PandarGpsReceiver(
    uint16_t recv_port,
    std::function<void(double)> gps_callback,
    std::string frame_id)
    : gpsRcvSocket_(io_context_, udp::endpoint(udp::v4(), recv_port)) {

    gps_callback_ = gps_callback;
    frame_id_ = frame_id;
}

PandarGpsReceiver::~PandarGpsReceiver() { Stop(); }

void PandarGpsReceiver::Start() {
    Stop();

    // Start Thread
    enableRecvThr_ = true;
    if (gps_callback_) {
        rcvGpsHandler();
    }
    contextThr_ = std::make_unique<std::thread>([this]() { io_context_.run(); });
}

void PandarGpsReceiver::Stop() {
    enableRecvThr_ = false;

    if (contextThr_ && contextThr_->joinable()) {
        contextThr_->join();
    }
}

void PandarGpsReceiver::rcvGpsHandler() {

    if (!enableRecvThr_) {
        return;
    }

    gpsRcvBuffer_.resize(1500);
    gpsRcvSocket_.async_receive_from(
        asio::buffer(gpsRcvBuffer_),
        gpsRemoteEndpoint_,
        [this](const asio::error_code &ec, std::size_t bytes) {
            if (ec) {
                std::cout << ec.message() << std::endl;
            } else {
                gpsRcvBuffer_.resize(bytes);
                processGps(gpsRcvBuffer_);
            }

            rcvGpsHandler();
        });
}

void PandarGpsReceiver::processGps(const std::vector<uint8_t> &packet) {

    if (auto gpsMsg = parseGPS(packet)) {
        std::tm t;
        t.tm_year = gpsMsg->year + 100; // years since 1900
        t.tm_mon = gpsMsg->month - 1;   // [0, 11]
        t.tm_mday = gpsMsg->day;
        t.tm_hour = gpsMsg->hour;
        t.tm_min = gpsMsg->minute;
        t.tm_sec = gpsMsg->second;
        t.tm_isdst = 0;

        gps_callback_(static_cast<double>(std::mktime(&t)));
    }
}

std::optional<PandarGpsPacket> PandarGpsReceiver::parseGPS(const std::vector<uint8_t> &recvbuf) {

    constexpr int GPS_PACKET_SIZE = 512;
    constexpr int GPS_PACKET_FLAG_SIZE = 2;
    constexpr int GPS_PACKET_YEAR_SIZE = 2;
    constexpr int GPS_PACKET_MONTH_SIZE = 2;
    constexpr int GPS_PACKET_DAY_SIZE = 2;
    constexpr int GPS_PACKET_HOUR_SIZE = 2;
    constexpr int GPS_PACKET_MINUTE_SIZE = 2;
    constexpr int GPS_PACKET_SECOND_SIZE = 2;

    if (recvbuf.size() != GPS_PACKET_SIZE) {
        return std::nullopt;
    }

    int index = 0;

    PandarGpsPacket packet;
    packet.flag = recvbuf[index] | recvbuf[index + 1] << 8;
    index += GPS_PACKET_FLAG_SIZE;
    packet.year = (recvbuf[index] - 0x30) + (recvbuf[index + 1] - 0x30) * 10;
    index += GPS_PACKET_YEAR_SIZE;
    packet.month = (recvbuf[index] - 0x30) + (recvbuf[index + 1] - 0x30) * 10;
    index += GPS_PACKET_MONTH_SIZE;
    packet.day = (recvbuf[index] - 0x30) + (recvbuf[index + 1] - 0x30) * 10;
    index += GPS_PACKET_DAY_SIZE;
    packet.second = (recvbuf[index] - 0x30) + (recvbuf[index + 1] - 0x30) * 10;
    index += GPS_PACKET_SECOND_SIZE;
    packet.minute = (recvbuf[index] - 0x30) + (recvbuf[index + 1] - 0x30) * 10;
    index += GPS_PACKET_MINUTE_SIZE;
    packet.hour = (recvbuf[index] - 0x30) + (recvbuf[index + 1] - 0x30) * 10;
    index += GPS_PACKET_HOUR_SIZE;
    packet.fineTime = recvbuf[index] | recvbuf[index + 1] << 8 | recvbuf[index + 2] << 16 |
                      recvbuf[index + 3] << 24;

    return packet;
}
