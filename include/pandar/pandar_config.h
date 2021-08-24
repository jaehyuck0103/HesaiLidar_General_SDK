#pragma once

#include <cstdint>
#include <iostream>
#include <map>
#include <vector>

enum class LidarModel {
    Pandar64,
    Pandar40P,
    PandarQT,
    PandarXT_16,
    PandarXT_32,
};

inline std::map<LidarModel, std::string> lidar_model_to_str = {
    {LidarModel::Pandar64, "Pandar64"},
    {LidarModel::Pandar40P, "Pandar40P"},
    {LidarModel::PandarQT, "PandarQT"},
    {LidarModel::PandarXT_16, "PandarXT_16"},
    {LidarModel::PandarXT_32, "PandarXT_32"},
};

inline std::map<std::string, LidarModel> lidar_model_from_str = {
    {"Pandar64", LidarModel::Pandar64},
    {"Pandar40P", LidarModel::Pandar40P},
    {"PandarQT", LidarModel::PandarQT},
    {"PandarXT_16", LidarModel::PandarXT_16},
    {"PandarXT_32", LidarModel::PandarXT_32},
};

class PandarConfig {

  public:
    PandarConfig(
        LidarModel model,
        bool dual_return_mode,
        int fps,
        uint16_t start_azimuth,
        std::vector<float> elev_angle,
        std::vector<float> azimuth_offset)
        : lidar_model_(model),
          dual_return_mode_(dual_return_mode),
          fps_(fps),
          start_azimuth_(start_azimuth),
          elev_angle_(elev_angle),
          azimuth_offset_(azimuth_offset) {

        // Set num_lasers_
        switch (lidar_model_) {
        case LidarModel::Pandar64:
            num_lasers_ = 64;
            break;
        case LidarModel::Pandar40P:
            num_lasers_ = 40;
            break;
        case LidarModel::PandarQT:
            num_lasers_ = 64;
            break;
        case LidarModel::PandarXT_16:
            num_lasers_ = 16;
            break;
        case LidarModel::PandarXT_32:
            num_lasers_ = 32;
            break;
        }

        // Set azimuth_res_
        switch (lidar_model_) {
        case LidarModel::Pandar64:
        case LidarModel::Pandar40P:
            if (fps == 10) {
                azimuth_res_ = 20;
            } else if (fps == 20) {
                azimuth_res_ = 40;
            } else {
                std::cout << "Invalid FPS for Pandar64 or Pandar40P: " << fps << "\n";
                std::abort();
            }
            break;
        case LidarModel::PandarQT:
            if (fps == 10) {
                azimuth_res_ = 60;
            } else {
                std::cout << "Invalid FPS for PandarQT: " << fps << "\n";
                std::abort();
            }
            break;
        case LidarModel::PandarXT_16:
        case LidarModel::PandarXT_32:
            if (fps == 5) {
                azimuth_res_ = 9;
            } else if (fps == 10) {
                azimuth_res_ = 18;
            } else if (fps == 20) {
                azimuth_res_ = 36;
            } else {
                std::cout << "Invalid FPS for PandarXT: " << fps << "\n";
                std::abort();
            }
            break;
        }

        // Validation Check
        if (!(start_azimuth_ < 36000 && start_azimuth_ % azimuth_res_ == 0)) {
            std::cout << "Invalid start_azimuth: " << start_azimuth_ << "\n";
            std::abort();
        }
        if (elev_angle_.size() != static_cast<size_t>(num_lasers_)) {
            std::cout << "Invalid elev_angle size: " << elev_angle_.size() << "\n";
            std::abort();
        }
        if (azimuth_offset_.size() != static_cast<size_t>(num_lasers_)) {
            std::cout << "Invalid azimuth_offset size: " << azimuth_offset_.size() << "\n";
            std::abort();
        }
    };

    // getters
    const LidarModel &lidar_model() const { return lidar_model_; }
    const bool &dual_return_mode() const { return dual_return_mode_; }
    const int &fps() const { return fps_; }
    const uint16_t &start_azimuth() const { return start_azimuth_; }
    const int &num_lasers() const { return num_lasers_; }
    const uint16_t &azimuth_res() const { return azimuth_res_; }
    const std::vector<float> &elev_angle() const { return elev_angle_; }
    const std::vector<float> &azimuth_offset() const { return azimuth_offset_; }

    // frame buffer
    float azimuth_res_deg() const { return azimuth_res_ / 100.0f; }
    int frame_buffer_width() const { return 36000 / azimuth_res_; }
    int num_returns() const { return dual_return_mode_ ? 2 : 1; }
    const int &frame_buffer_height() const { return num_lasers_; }
    int frame_buffer_elems() const {
        return frame_buffer_width() * num_returns() * frame_buffer_height();
    }
    int frame_buffer_bytes() const { return frame_buffer_elems() * elem_bytes; }

    int get_frame_buffer_elem_index(int azimuth_idx, int return_idx, int laser_idx) const {
        return (azimuth_idx * num_returns() + return_idx) * num_lasers() + laser_idx;
    }
    int get_frame_buffer_bytes_index(int azimuth_idx, int return_idx, int laser_idx) const {
        return get_frame_buffer_elem_index(azimuth_idx, return_idx, laser_idx) * elem_bytes;
    }
    int azimuthToAzimuthIdx(uint16_t azimuth) const {
        return ((azimuth + 36000 - start_azimuth()) / azimuth_res()) % frame_buffer_width();
    }
    uint16_t azimuthIdxToAzimuth(int azimuth_idx) const {
        return (azimuth_idx * azimuth_res() + start_azimuth()) % 36000;
    }

    // utils
    bool isValidAzimuth(uint16_t azimuth) const {
        if (azimuth < 36000 && azimuth % azimuth_res() == 0) {
            return true;
        } else {
            return false;
        }
    }

    // constexpr
    static constexpr float unit_distance = 0.004; // 4mm
    static constexpr int elem_bytes = 3;          // 2bytes raw_distance + 1byte intensity

  private:
    const LidarModel lidar_model_;
    const bool dual_return_mode_;
    const int fps_;
    const uint16_t start_azimuth_;
    int num_lasers_;
    uint16_t azimuth_res_;

    const std::vector<float> elev_angle_;
    const std::vector<float> azimuth_offset_;
};
