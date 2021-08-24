#include "pandar/pandar_lidar_receiver.h"
#include "pandar_packet_parsers.h"

#include <iostream>

PandarLidarReceiver::PandarLidarReceiver(
    const std::string &lidar_ip,
    uint16_t recv_port,
    std::function<void(const std::vector<uint8_t> &, time_point<system_clock> &)> pcl_callback,
    const PandarConfig &cfg)
    : lidar_ip_(lidar_ip),
      pcl_callback_(pcl_callback),
      cfg_(cfg),
      socket_(io_context_, udp::endpoint(udp::v4(), recv_port)) {

    // Init frame_buffer_
    // W x (1,2) x H x 3bytes
    frame_buffer_ = std::vector<uint8_t>(cfg_.frame_buffer_bytes(), 0);

    switch (cfg_.lidar_model()) {
    case LidarModel::Pandar40P:
        parseLidarPacket = [this](const std::vector<uint8_t> &packet) {
            return PandarPacketParsers::pandar40p(cfg_, packet);
        };
        break;
    case LidarModel::Pandar64:
        parseLidarPacket = [this](const std::vector<uint8_t> &packet) {
            return PandarPacketParsers::pandar64(cfg_, packet);
        };
        break;
    case LidarModel::PandarQT:
        parseLidarPacket = [this](const std::vector<uint8_t> &packet) {
            return PandarPacketParsers::pandarQT(cfg_, packet);
        };
        break;
    case LidarModel::PandarXT_16:
    case LidarModel::PandarXT_32:
        parseLidarPacket = [this](const std::vector<uint8_t> &packet) {
            return PandarPacketParsers::pandarXT(cfg_, packet);
        };
        break;
    }
}

PandarLidarReceiver::~PandarLidarReceiver() { stop(); }

void PandarLidarReceiver::start() {
    stop();

    // Start Thread
    enable_recv_flag_ = true;
    if (pcl_callback_) {
        socketRecvHandler();
    };
    context_thr_ = std::make_unique<std::thread>([this]() { io_context_.run(); });
}

void PandarLidarReceiver::stop() {
    enable_recv_flag_ = false;

    if (context_thr_ && context_thr_->joinable()) {
        context_thr_->join();
    }
}

void PandarLidarReceiver::socketRecvHandler() {

    if (!enable_recv_flag_) {
        return;
    }

    recv_buffer_.resize(1500);
    socket_.async_receive_from(
        asio::buffer(recv_buffer_),
        remote_endpoint_,
        [this](const asio::error_code &ec, std::size_t bytes) {
            if (ec) {
                std::cout << ec.message() << std::endl;
            } else if (!lidar_ip_.empty() && lidar_ip_ != remote_endpoint_.address().to_string()) {
                std::cout << "Packet from unknown ip address: "
                          << remote_endpoint_.address().to_string() << "\n";
            } else {
                recv_buffer_.resize(bytes);
                processLidarPacket(recv_buffer_);
            }

            socketRecvHandler();
        });
}

void PandarLidarReceiver::processLidarPacket(const std::vector<uint8_t> &packet) {

    static uint16_t last_azimuth = 0;
    const uint16_t start_azimuth = cfg_.start_azimuth();

    if (auto pkt = parseLidarPacket(packet)) {

        for (size_t i = 0; i < pkt->blocks.size(); ++i) {

            uint16_t curr_azimuth = pkt->blocks[i].azimuth;

            if (!cfg_.isValidAzimuth(curr_azimuth)) {
                std::cout << "Invalid Azimuth: " << curr_azimuth << "\n";
                break;
            }

            if ((last_azimuth < start_azimuth && start_azimuth <= curr_azimuth) ||
                (start_azimuth <= curr_azimuth && curr_azimuth < last_azimuth) ||
                (curr_azimuth < last_azimuth && last_azimuth < start_azimuth)) {

                if (timestamp_ != system_clock::time_point::min()) {
                    pcl_callback_(frame_buffer_, timestamp_);
                    std::fill(frame_buffer_.begin(), frame_buffer_.end(), 0);
                }
                timestamp_ = system_clock::now();
            }

            // Fill frame_buffer
            const int azimuth_idx = cfg_.azimuthToAzimuthIdx(curr_azimuth);
            const int return_idx = i % cfg_.num_returns();
            const int buffer_idx = cfg_.get_frame_buffer_bytes_index(azimuth_idx, return_idx, 0);
            std::copy(
                pkt->blocks[i].payload.begin(),
                pkt->blocks[i].payload.end(),
                frame_buffer_.begin() + buffer_idx);

            // Save last_azimuth
            last_azimuth = curr_azimuth;
        }
    }
}
