#include "pandar/pandar_lidar_receiver.h"

#include <iostream>

PandarLidarReceiver::PandarLidarReceiver(
    uint16_t lidar_port,
    std::function<void(const std::vector<uint8_t> &, double)> pcl_callback,
    uint16_t start_azimuth,
    std::string lidar_type,
    std::string frame_id,
    std::string timestamp_type,
    int fps,
    bool dual_return_mode)
    : pcl_callback_(pcl_callback),
      start_azimuth_(start_azimuth),
      frame_id_(frame_id),
      timestamp_type_(timestamp_type),
      lidar_type_(lidar_type),
      fps_(fps),
      dual_return_mode_(dual_return_mode),
      socket_(io_context_, udp::endpoint(udp::v4(), lidar_port)) {}

PandarLidarReceiver::~PandarLidarReceiver() { stop(); }

void PandarLidarReceiver::start() {
    stop();

    // Init frame_buffer_
    // W x (1,2) x H x 3bytes
    int frame_buffer_size = (36000 / azimuth_res_) * num_lasers_ * 3;
    if (dual_return_mode_) {
        frame_buffer_size *= 2;
    }
    frame_buffer_ = std::vector<uint8_t>(frame_buffer_size, 0);

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
            } else {
                recv_buffer_.resize(bytes);
                processLidarPacket(recv_buffer_);
            }

            socketRecvHandler();
        });
}

void PandarLidarReceiver::processLidarPacket(const std::vector<uint8_t> &packet) {

    static uint16_t last_azimuth = 0;
    std::chrono::duration<double> timestamp = std::chrono::system_clock::now().time_since_epoch();

    if (auto pkt = parseLidarPacket(packet)) {

        for (size_t i = 0; i < pkt->blocks.size(); ++i) {

            uint16_t curr_azimuth = pkt->blocks[i].azimuth;

            if (!isValidAzimuth(curr_azimuth)) {
                std::cout << "Unvalid Azimuth: " << curr_azimuth << "\n";
                break;
            }

            if ((last_azimuth < start_azimuth_ && start_azimuth_ <= curr_azimuth) ||
                (start_azimuth_ <= curr_azimuth && curr_azimuth < last_azimuth) ||
                (curr_azimuth < last_azimuth && last_azimuth < start_azimuth_)) {

                pcl_callback_(frame_buffer_, timestamp.count());
                std::fill(frame_buffer_.begin(), frame_buffer_.end(), 0);
            }

            // Fill frame_buffer
            const uint16_t azimuth_idx = curr_azimuth / azimuth_res_;
            int buffer_idx = azimuth_idx * num_lasers_ * 3;
            if (dual_return_mode_) {
                buffer_idx = 2 * buffer_idx + (i % 2) * num_lasers_ * 3;
            }
            std::copy(
                pkt->blocks[i].payload.begin(),
                pkt->blocks[i].payload.end(),
                frame_buffer_.begin() + buffer_idx);

            // Save last_azimuth
            last_azimuth = curr_azimuth;
        }
    }
}
