#include "pandar/pandar_lidar_receiver.h"

#include <iostream>

PandarLidarReceiver::PandarLidarReceiver(
    uint16_t lidar_port,
    std::function<void(const std::vector<uint8_t> &, double)> pcl_callback,
    uint16_t start_azimuth,
    std::string lidar_type,
    std::string frame_id,
    std::string timestampType,
    int fps,
    bool dualReturnMode)
    : lidarRcvSocket_(io_context_, udp::endpoint(udp::v4(), lidar_port)) {

    pcl_callback_ = pcl_callback;
    start_azimuth_ = start_azimuth;
    m_sLidarType = lidar_type;
    frame_id_ = frame_id;
    m_sTimestampType = timestampType;
    fps_ = fps;
    dualReturnMode_ = dualReturnMode;
}

PandarLidarReceiver::~PandarLidarReceiver() { Stop(); }

void PandarLidarReceiver::Start() {
    Stop();

    // Init frame_buffer_
    // W x (1,2) x H x 3bytes
    int frame_buffer_size = (36000 / azimuth_res_) * num_lasers_ * 3;
    if (dualReturnMode_) {
        frame_buffer_size *= 2;
    }
    frame_buffer_ = std::vector<uint8_t>(frame_buffer_size, 0);

    // Start Thread
    enableRecvThr_ = true;
    if (pcl_callback_) {
        rcvLidarHandler();
    };
    contextThr_ = std::make_unique<std::thread>([this]() { io_context_.run(); });
}

void PandarLidarReceiver::Stop() {
    enableRecvThr_ = false;

    if (contextThr_ && contextThr_->joinable()) {
        contextThr_->join();
    }
}

void PandarLidarReceiver::rcvLidarHandler() {

    if (!enableRecvThr_) {
        return;
    }

    lidarRcvBuffer_.resize(1500);
    lidarRcvSocket_.async_receive_from(
        asio::buffer(lidarRcvBuffer_),
        lidarRemoteEndpoint_,
        [this](const asio::error_code &ec, std::size_t bytes) {
            if (ec) {
                std::cout << ec.message() << std::endl;
            } else {
                lidarRcvBuffer_.resize(bytes);
                processLidarPacket(lidarRcvBuffer_);
            }

            rcvLidarHandler();
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
            if (dualReturnMode_) {
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
