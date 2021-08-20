#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace TcpCommand {

enum class PTC_Command : uint8_t {
    GET_CALIBRATION = 0,
    SET_CALIBRATION,
    HEARTBEAT,
    RESET_CALIBRATION,
    TEST,
    GET_LIDAR_CALIBRATION,
};

enum class PTC_ErrCode : uint8_t {
    NO_ERROR = 0,
    BAD_PARAMETER,
    CONNECT_SERVER_FAILED,
    TRANSFER_FAILED,
    NO_MEMORY,
    INVALID_RETURN_CODE, // custom add
};

struct TC_Command {
    uint8_t cmd;
    uint8_t ret_code;
    std::vector<uint8_t> payload;
};

std::pair<PTC_ErrCode, TC_Command>
setCalibration(std::string device_ip, int device_port, std::vector<uint8_t> payload);
std::pair<PTC_ErrCode, TC_Command> getCalibration(std::string device_ip, int device_port);
std::pair<PTC_ErrCode, TC_Command> getLidarCalibration(std::string device_ip, int device_port);
std::pair<PTC_ErrCode, TC_Command> resetCalibration(std::string device_ip, int device_port);

} // namespace TcpCommand
