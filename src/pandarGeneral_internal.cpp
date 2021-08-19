/******************************************************************************
 * Copyright 2019 The Hesai Technology Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "pandar/pandarGeneral_internal.h"

#include <cmath>
#include <iostream>
#include <sstream>

double degToRad(double degree) { return degree * M_PI / 180; }

PandarGeneral_Internal::PandarGeneral_Internal(
    uint16_t lidar_port,
    uint16_t gps_port,
    std::function<void(const std::vector<uint8_t> &, double)> pcl_callback,
    std::function<void(double)> gps_callback,
    uint16_t start_azimuth,
    std::string lidar_type,
    std::string frame_id,
    std::string timestampType,
    int fps,
    bool dualReturnMode)
    : lidarRcvSocket_(io_context_, udp::endpoint(udp::v4(), lidar_port)),
      gpsRcvSocket_(io_context_, udp::endpoint(udp::v4(), gps_port)) {

    pcl_callback_ = pcl_callback;
    gps_callback_ = gps_callback;
    start_azimuth_ = start_azimuth;
    m_sLidarType = lidar_type;
    frame_id_ = frame_id;
    m_sTimestampType = timestampType;
    fps_ = fps;
    dualReturnMode_ = dualReturnMode;
}

PandarGeneral_Internal::~PandarGeneral_Internal() { Stop(); }

void PandarGeneral_Internal::Start() {
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
    if (gps_callback_) {
        rcvGpsHandler();
    }
    contextThr_ = std::make_unique<std::thread>([this]() { io_context_.run(); });
}

void PandarGeneral_Internal::Stop() {
    enableRecvThr_ = false;

    if (contextThr_ && contextThr_->joinable()) {
        contextThr_->join();
    }
}

void PandarGeneral_Internal::rcvLidarHandler() {

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

void PandarGeneral_Internal::rcvGpsHandler() {

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

bool PandarGeneral_Internal::updateAngleCorrection(std::string correction_content) {
    std::cout << "Parse Lidar Correction...\n";

    std::vector<float> elev_angle;
    std::vector<float> azimuth_offset;

    std::istringstream ifs(correction_content);
    std::string line;
    std::getline(ifs, line); // Throw away first line

    for (int lineCounter = 1; std::getline(ifs, line); ++lineCounter) {

        // correction file has 3 columns, min length is 5
        if (line.length() < 5) {
            break;
        }

        std::istringstream ss{line};
        std::string subline;

        std::getline(ss, subline, ',');
        int lineId = stoi(subline);
        std::getline(ss, subline, ',');
        float elev = stof(subline);
        std::getline(ss, subline, ',');
        float azimuth = stof(subline);

        if (lineId != lineCounter) {
            break;
        }

        elev_angle.push_back(elev);
        azimuth_offset.push_back(azimuth);
    }

    if (elev_angle.size() == static_cast<size_t>(num_lasers_) &&
        azimuth_offset.size() == static_cast<size_t>(num_lasers_)) {

        elev_angle_map_ = elev_angle;
        azimuth_offset_map_ = azimuth_offset;
        std::cout << "Parse Lidar Correction Succeed...\n";
        return true;
    } else {
        std::cout << "Parse Lidar Correction Failed...\n";
        return false;
    }
}

void PandarGeneral_Internal::processLidarPacket(const std::vector<uint8_t> &packet) {

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

void PandarGeneral_Internal::processGps(const std::vector<uint8_t> &packet) {

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

std::optional<PandarGPS> PandarGeneral_Internal::parseGPS(const std::vector<uint8_t> &recvbuf) {

    if (recvbuf.size() != GPS_PACKET_SIZE) {
        return std::nullopt;
    }

    int index = 0;

    PandarGPS packet;
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
