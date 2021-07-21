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
