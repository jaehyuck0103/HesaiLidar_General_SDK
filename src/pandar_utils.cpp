#include "pandar/pandar_utils.h"
#include "pandar/pandar_config.h"

#include "tcp_command_client.h"

#include <nlohmann/json.hpp>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace PandarUtils {

std::optional<std::pair<std::vector<float>, std::vector<float>>>
parseAngleCorrectionContent(std::string correction_content, int num_lasers) {

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

    if (elev_angle.size() == static_cast<size_t>(num_lasers) &&
        azimuth_offset.size() == static_cast<size_t>(num_lasers)) {

        std::cout << "Parse Lidar Correction Succeed...\n";
        return {{elev_angle, azimuth_offset}};
    } else {
        std::cout << "Parse Lidar Correction Failed...\n";
        return std::nullopt;
    }
}

std::optional<std::string>
getAngleCorrectionFromDevice(const std::string &device_ip, const int device_port) {

    // Dual Check
    auto [ec, feedback] = TcpCommand::getLidarCalibration(device_ip, device_port);
    auto [ec2, feedback2] = TcpCommand::getLidarCalibration(device_ip, device_port);

    if (ec != TcpCommand::PTC_ErrCode::NO_ERROR || ec2 != TcpCommand::PTC_ErrCode::NO_ERROR) {
        std::cout << "Fail to get correction content\n";
        return std::nullopt;
    }

    std::string correction_content(feedback.payload.begin(), feedback.payload.end());
    std::string correction_content2(feedback2.payload.begin(), feedback2.payload.end());
    if (correction_content != correction_content2) {
        std::cout << "Fail to pass the dual check\n";
        return std::nullopt;
    } else {
        return correction_content;
    }
}

std::string getAngleCorrectionFromFile(std::string filePath) {

    std::ifstream fin(filePath);
    std::ostringstream strStream;
    strStream << fin.rdbuf();
    fin.close();

    return strStream.str();
}

std::optional<std::pair<std::vector<float>, std::vector<float>>>
getFallbackAngleCorrection(std::string lidar_type) {

    std::vector<float> elev_angle;
    std::vector<float> azimuth_offset;

    if (lidar_type == "Pandar40P") {
        elev_angle = {15.0f,  11.0f,  8.0f,   5.0f,   3.0f,   2.0f,   1.67f,  1.33f,
                      1.0f,   0.67f,  0.33f,  0.0f,   -0.33f, -0.67f, -1.0f,  -1.33f,
                      -1.66f, -2.0f,  -2.33f, -2.67f, -3.0f,  -3.33f, -3.67f, -4.0f,
                      -4.33f, -4.67f, -5.0f,  -5.33f, -5.67f, -6.0f,  -7.0f,  -8.0f,
                      -9.0f,  -10.0f, -11.0f, -12.0f, -13.0f, -14.0f, -19.0f, -25.0f};
        azimuth_offset = {-1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, 3.125f,  -5.208f,
                          -1.042f, 3.125f,  -5.208f, -1.042f, 3.125f,  -5.208f, -1.042f, 3.125f,
                          -5.208f, -1.042f, 3.125f,  -5.208f, -1.042f, 3.125f,  -5.208f, -1.042f,
                          3.125f,  -5.208f, -1.042f, 3.125f,  -5.208f, -1.042f, -1.042f, -1.042f,
                          -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f};
    } else if (lidar_type == "Pandar64") {
        elev_angle = {14.882f, 11.032f, 8.059f,   5.057f,   3.04f,    2.028f,  1.86f,    1.688f,
                      1.522f,  1.351f,  1.184f,   1.013f,   0.846f,   0.675f,  0.508f,   0.337f,
                      0.169f,  0.0f,    -0.169f,  -0.337f,  -0.508f,  -0.675f, -0.845f,  -1.013f,
                      -1.184f, -1.351f, -1.522f,  -1.688f,  -1.86f,   -2.028f, -2.198f,  -2.365f,
                      -2.536f, -2.7f,   -2.873f,  -3.04f,   -3.21f,   -3.375f, -3.548f,  -3.712f,
                      -3.884f, -4.05f,  -4.221f,  -4.385f,  -4.558f,  -4.72f,  -4.892f,  -5.057f,
                      -5.229f, -5.391f, -5.565f,  -5.726f,  -5.898f,  -6.061f, -7.063f,  -8.059f,
                      -9.06f,  -9.885f, -11.032f, -12.006f, -12.974f, -13.93f, -18.889f, -24.897f};
        azimuth_offset = {-1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, 1.042f,  3.125f,
                          5.208f,  -5.208f, -3.125f, -1.042f, 1.042f,  3.125f,  5.208f,  -5.208f,
                          -3.125f, -1.042f, 1.042f,  3.125f,  5.208f,  -5.208f, -3.125f, -1.042f,
                          1.042f,  3.125f,  5.208f,  -5.208f, -3.125f, -1.042f, 1.042f,  3.125f,
                          5.208f,  -5.208f, -3.125f, -1.042f, 1.042f,  3.125f,  5.208f,  -5.208f,
                          -3.125f, -1.042f, 1.042f,  3.125f,  5.208f,  -5.208f, -3.125f, -1.042f,
                          1.042f,  3.125f,  5.208f,  -5.208f, -3.125f, -1.042f, -1.042f, -1.042f,
                          -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f};
    } else if (lidar_type == "PandarQT") {
        elev_angle = {
            -52.121f, -49.785f, -47.577f, -45.477f, -43.465f, -41.528f, -39.653f, -37.831f,
            -36.055f, -34.320f, -32.619f, -30.950f, -29.308f, -27.690f, -26.094f, -24.517f,
            -22.964f, -21.420f, -19.889f, -18.372f, -16.865f, -15.368f, -13.880f, -12.399f,
            -10.925f, -9.457f,  -7.994f,  -6.535f,  -5.079f,  -3.626f,  -2.175f,  -0.725f,
            0.725f,   2.175f,   3.626f,   5.079f,   6.534f,   7.993f,   9.456f,   10.923f,
            12.397f,  13.877f,  15.365f,  16.861f,  18.368f,  19.885f,  21.415f,  22.959f,
            24.524f,  26.101f,  27.697f,  29.315f,  30.957f,  32.627f,  34.328f,  36.064f,
            37.840f,  39.662f,  41.537f,  43.475f,  45.487f,  47.587f,  49.795f,  52.133f};
        azimuth_offset = {8.736f,  8.314f,  7.964f,  7.669f,  7.417f,  7.198f,  7.007f,  6.838f,
                          6.688f,  6.554f,  6.434f,  6.326f,  6.228f,  6.140f,  6.059f,  5.987f,
                          -5.270f, -5.216f, -5.167f, -5.123f, -5.083f, -5.047f, -5.016f, -4.988f,
                          -4.963f, -4.942f, -4.924f, -4.910f, -4.898f, -4.889f, -4.884f, -4.881f,
                          5.493f,  5.496f,  5.502f,  5.512f,  5.525f,  5.541f,  5.561f,  5.584f,
                          5.611f,  5.642f,  5.676f,  5.716f,  5.759f,  5.808f,  5.862f,  5.921f,
                          -5.330f, -5.396f, -5.469f, -5.550f, -5.640f, -5.740f, -5.850f, -5.974f,
                          -6.113f, -6.269f, -6.447f, -6.651f, -6.887f, -7.163f, -7.493f, -7.892f};
    } else if (lidar_type == "PandarXT-32" || lidar_type == "PandarXT-16") {
        for (int i = 0; i < 32; ++i) {
            elev_angle.push_back(15.0f - i);
        }

        azimuth_offset = std::vector<float>(32, 0.0f);

        if (lidar_type == "PandarXT-16") {
            for (int i = 0; i < 16; i++) {
                elev_angle[i] = elev_angle[i * 2];
                azimuth_offset[i] = azimuth_offset[i * 2];
            }
            elev_angle.resize(16);
            azimuth_offset.resize(16);
        }
    } else {
        return std::nullopt;
    }

    return {{elev_angle, azimuth_offset}};
}

void writePandarConfigJson(const PandarConfig &cfg, const std::string &file_path) {
    nlohmann::json j;

    switch (cfg.lidar_model()) {
    case LidarModel::Pandar64:
        j["lidar_model"] = "Pandar64";
        break;
    case LidarModel::Pandar40P:
        j["lidar_model"] = "Pandar40P";
        break;
    case LidarModel::PandarQT:
        j["lidar_model"] = "PandarQT";
        break;
    case LidarModel::PandarXT_16:
        j["lidar_model"] = "PandarXT_16";
        break;
    case LidarModel::PandarXT_32:
        j["lidar_model"] = "PandarXT_32";
        break;
    }

    j["dual_return_mode"] = cfg.dual_return_mode();
    j["fps"] = cfg.fps();
    j["start_azimuth"] = cfg.start_azimuth();
    j["elev_angle"] = cfg.elev_angle();
    j["azimuth_offset"] = cfg.azimuth_offset();

    std::ofstream o(file_path);
    o << std::setw(4) << j << std::endl;
}

PandarConfig readPandarConfigJson(const std::string &file_path) {
    std::ifstream i(file_path);
    nlohmann::json j;
    i >> j;

    std::string lidar_model_str = j["lidar_model"].get<std::string>();
    LidarModel lidar_model;
    if (lidar_model_str == "Pandar64") {
        lidar_model = LidarModel::Pandar64;
    } else if (lidar_model_str == "Pandar40P") {
        lidar_model = LidarModel::Pandar40P;
    } else if (lidar_model_str == "PandarQT") {
        lidar_model = LidarModel::PandarQT;
    } else if (lidar_model_str == "PandarXT_16") {
        lidar_model = LidarModel::PandarXT_16;
    } else if (lidar_model_str == "PandarXT_32") {
        lidar_model = LidarModel::PandarXT_32;
    } else {
        std::cout << "Unknown LidarModel: " << lidar_model_str << "\n";
        std::abort();
    }

    return {
        lidar_model,
        j["dual_return_mode"].get<bool>(),
        j["fps"].get<int>(),
        j["start_azimuth"].get<uint16_t>(),
        j["elev_angle"].get<std::vector<float>>(),
        j["azimuth_offset"].get<std::vector<float>>()};
}

} // namespace PandarUtils
