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

#include "pandar/tcp_command_client.h"
#include "pandar/util.h"
#include <array>
#include <iomanip>
#include <iostream>
#include <unistd.h>

namespace TcpCommand {

void print_mem(std::vector<uint8_t> mem) {
#ifdef DEBUG
    for (uint8_t elem : mem) {
        std::cout << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(elem)
                  << " ";
    }
    std::cout << "\n";
#else
#endif
}

std::pair<PTC_ErrCode, TC_Command> readCommand(int connfd) {
    std::cout << "tcpCommandReadCommand\n";

    std::array<uint8_t, 2> buffer1;
    if (int ret = sys_readn(connfd, buffer1.data(), buffer1.size());
        ret != buffer1.size() || buffer1.at(0) != 0x47 || buffer1.at(1) != 0x74) {
        std::cout << "Receive feedback failed!!!\n";
        return {PTC_ErrCode::TRANSFER_FAILED, {}};
    }

    std::array<uint8_t, 6> buffer2;
    if (int ret = sys_readn(connfd, buffer2.data(), buffer2.size()); ret != buffer2.size()) {
        std::cout << "Receive feedback failed!!!\n";
        return {PTC_ErrCode::TRANSFER_FAILED, {}};
    }

    // Parsing
    TC_Command feedback;
    feedback.cmd = buffer2.at(0);
    feedback.ret_code = buffer2.at(1);
    if (feedback.ret_code != 0) {
        std::cout << "Invalid return code\n";
        return {PTC_ErrCode::TRANSFER_FAILED, {}};
    }

    uint32_t len_payload = ((buffer2.at(2) & 0xff) << 24) | ((buffer2.at(3) & 0xff) << 16) |
                           ((buffer2.at(4) & 0xff) << 8) | ((buffer2.at(5) & 0xff) << 0);
    if (len_payload > 0) {
        feedback.payload.resize(len_payload);
        int ret = sys_readn(connfd, feedback.payload.data(), len_payload);
        if (ret != static_cast<int>(len_payload)) {
            std::cout << "Receive payload failed!!!\n";
            return {PTC_ErrCode::TRANSFER_FAILED, {}};
        }
    }

    print_mem(feedback.payload);

    return {PTC_ErrCode::NO_ERROR, feedback};
}

std::vector<uint8_t> buildHeader(const TC_Command &cmd) {
    std::cout << "TcpCommand_buildHeader\n";
    return {
        0x47,
        0x74,
        cmd.cmd,
        cmd.ret_code,
        static_cast<uint8_t>((cmd.payload.size() >> 24) & 0xff),
        static_cast<uint8_t>((cmd.payload.size() >> 16) & 0xff),
        static_cast<uint8_t>((cmd.payload.size() >> 8) & 0xff),
        static_cast<uint8_t>((cmd.payload.size() >> 0) & 0xff)};
}

std::pair<PTC_ErrCode, TC_Command>
sendCmd(std::string device_ip, int device_port, const TC_Command &cmd) {
    std::cout << "tcpCommandClient_SendCmd\n";

    int fd = tcp_open(device_ip.c_str(), device_port);
    if (fd < 0) {
        return {PTC_ErrCode::CONNECT_SERVER_FAILED, {}};
    }

    // Send Command
    std::vector<uint8_t> buffer = buildHeader(cmd);
    print_mem(buffer);
    int ret = write(fd, buffer.data(), buffer.size());
    if (ret != static_cast<int>(buffer.size())) {
        std::cout << "Write header error\n";
        close(fd);
        return {PTC_ErrCode::TRANSFER_FAILED, {}};
    }

    if (!cmd.payload.empty()) {
        print_mem(cmd.payload);
        ret = write(fd, cmd.payload.data(), cmd.payload.size());
        if (ret != static_cast<int>(cmd.payload.size())) {
            std::cout << "Write Payload error\n";
            close(fd);
            return {PTC_ErrCode::TRANSFER_FAILED, {}};
        }
    }

    // Receive Feedback
    auto ret2 = readCommand(fd);
    close(fd);
    return ret2;
}

std::pair<PTC_ErrCode, TC_Command>
setCalibration(std::string device_ip, int device_port, std::vector<uint8_t> payload) {

    TC_Command cmd;
    cmd.cmd = static_cast<uint8_t>(PTC_Command::SET_CALIBRATION);
    cmd.payload = payload;

    return sendCmd(device_ip, device_port, cmd);
}

std::pair<PTC_ErrCode, TC_Command> getCalibration(std::string device_ip, int device_port) {

    TC_Command cmd;
    cmd.cmd = static_cast<uint8_t>(PTC_Command::GET_CALIBRATION);

    return sendCmd(device_ip, device_port, cmd);
}

std::pair<PTC_ErrCode, TC_Command> getLidarCalibration(std::string device_ip, int device_port) {

    TC_Command cmd;
    cmd.cmd = static_cast<uint8_t>(PTC_Command::GET_LIDAR_CALIBRATION);

    return sendCmd(device_ip, device_port, cmd);
}

std::pair<PTC_ErrCode, TC_Command> resetCalibration(std::string device_ip, int device_port) {

    TC_Command cmd;
    cmd.cmd = static_cast<uint8_t>(PTC_Command::RESET_CALIBRATION);

    return sendCmd(device_ip, device_port, cmd);
}

} // namespace TcpCommand
