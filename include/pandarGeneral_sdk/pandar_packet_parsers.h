#pragma once

#include "pandarGeneral_sdk/pandar_config.h"

#include <cstdint>
#include <optional>
#include <vector>

struct HS_LIDAR_Block {
    uint16_t azimuth;             // Azimuth = RealAzimuth * 100
    std::vector<uint8_t> payload; // num_lasers * 3 bytes (2bytes raw_distance + 1byte intensity)
};

struct HS_LIDAR_Packet {
    std::vector<HS_LIDAR_Block> blocks;
    double timestamp;
};

namespace PandarPacketParsers {

std::optional<HS_LIDAR_Packet>
pandar40p(const PandarConfig &cfg, const std::vector<uint8_t> &recvbuf);

std::optional<HS_LIDAR_Packet>
pandar64(const PandarConfig &cfg, const std::vector<uint8_t> &recvbuf);

std::optional<HS_LIDAR_Packet>
pandarQT(const PandarConfig &cfg, const std::vector<uint8_t> &recvbuf);

std::optional<HS_LIDAR_Packet>
pandarXT(const PandarConfig &cfg, const std::vector<uint8_t> &recvbuf);

} // namespace PandarPacketParsers
