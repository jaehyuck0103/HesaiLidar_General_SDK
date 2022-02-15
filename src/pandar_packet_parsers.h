#pragma once

#include "pandar/pandar_lidar_receiver.h"

#include <cstdint>
#include <optional>
#include <vector>

namespace PandarPacketParsers {

std::optional<HS_LIDAR_Packet>
pandar40p(const PandarConfig &cfg, std::span<const uint8_t> recvbuf);

std::optional<HS_LIDAR_Packet> pandar64(const PandarConfig &cfg, std::span<const uint8_t> recvbuf);

std::optional<HS_LIDAR_Packet> pandarQT(const PandarConfig &cfg, std::span<const uint8_t> recvbuf);

std::optional<HS_LIDAR_Packet> pandarXT(const PandarConfig &cfg, std::span<const uint8_t> recvbuf);

} // namespace PandarPacketParsers
