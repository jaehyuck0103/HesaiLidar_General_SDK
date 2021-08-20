#pragma once

#include <optional>
#include <vector>

namespace PandarUtils {

constexpr int PANDAR_TCP_COMMAND_PORT = 9347;

std::optional<std::string> getAngleCorrectionFromDevice(
    const std::string &device_ip,
    const int device_port = PANDAR_TCP_COMMAND_PORT);

std::optional<std::pair<std::vector<float>, std::vector<float>>>
getFallbackAngleCorrection(std::string lidar_type);

} // namespace PandarUtils
