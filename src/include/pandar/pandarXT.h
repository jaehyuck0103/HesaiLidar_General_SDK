#pragma once

#include "pandarGeneral_internal.h"

#include <cstdint>

// Head
#define HS_LIDAR_XT_HEAD_SIZE (12)
// Body
#define HS_LIDAR_XT_BLOCK_NUMBER (8)
#define HS_LIDAR_XT_BLOCK_HEADER_AZIMUTH (2)
#define HS_LIDAR_XT_UNIT_NUM (32)
#define HS_LIDAR_XT_UNIT_SIZE (4)
#define HS_LIDAR_XT_BLOCK_SIZE                                                                    \
    (HS_LIDAR_XT_UNIT_SIZE * HS_LIDAR_XT_UNIT_NUM + HS_LIDAR_XT_BLOCK_HEADER_AZIMUTH)
#define HS_LIDAR_XT_BODY_SIZE (HS_LIDAR_XT_BLOCK_SIZE * HS_LIDAR_XT_BLOCK_NUMBER)
// Tail
#define HS_LIDAR_XT_RESERVED_SIZE (10)
#define HS_LIDAR_XT_ENGINE_VELOCITY (2)
#define HS_LIDAR_XT_TIMESTAMP_SIZE (4)
#define HS_LIDAR_XT_ECHO_SIZE (1)
#define HS_LIDAR_XT_FACTORY_SIZE (1)
#define HS_LIDAR_XT_UTC_SIZE (6)
#define HS_LIDAR_XT_SEQUENCE_SIZE (4)
#define HS_LIDAR_XT_PACKET_TAIL_SIZE (28)
// All
#define HS_LIDAR_XT_PACKET_SIZE                                                                   \
    (HS_LIDAR_XT_HEAD_SIZE + HS_LIDAR_XT_BODY_SIZE + HS_LIDAR_XT_PACKET_TAIL_SIZE)
// XT16
#define HS_LIDAR_XT16_UNIT_NUM (16)
#define HS_LIDAR_XT16_PACKET_SIZE (568)

const float pandarXT_elev_angle_map[] = {
    15.0f, 14.0f, 13.0f, 12.0f,  11.0f,  10.0f,  9.0f,   8.0f,   7.0f,   6.0f,  5.0f,
    4.0f,  3.0f,  2.0f,  1.0f,   0.0f,   -1.0f,  -2.0f,  -3.0f,  -4.0f,  -5.0f, -6.0f,
    -7.0f, -8.0f, -9.0f, -10.0f, -11.0f, -12.0f, -13.0f, -14.0f, -15.0f, -16.0f};

const float pandarXT_horizatal_azimuth_offset_map[] = {
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

const float laserXTOffset[HS_LIDAR_XT_UNIT_NUM] = {
    1.512f * 0.0f + 0.28f,  1.512f * 1.0f + 0.28f,  1.512f * 2.0f + 0.28f,
    1.512f * 3.0f + 0.28f,  1.512f * 4.0f + 0.28f,  1.512f * 5.0f + 0.28f,
    1.512f * 6.0f + 0.28f,  1.512f * 7.0f + 0.28f,

    1.512f * 8.0f + 0.28f,  1.512f * 9.0f + 0.28f,  1.512f * 10.0f + 0.28f,
    1.512f * 11.0f + 0.28f, 1.512f * 12.0f + 0.28f, 1.512f * 13.0f + 0.28f,
    1.512f * 14.0f + 0.28f, 1.512f * 15.0f + 0.28f,

    1.512f * 16.0f + 0.28f, 1.512f * 17.0f + 0.28f, 1.512f * 18.0f + 0.28f,
    1.512f * 19.0f + 0.28f, 1.512f * 20.0f + 0.28f, 1.512f * 21.0f + 0.28f,
    1.512f * 22.0f + 0.28f, 1.512f * 23.0f + 0.28f,

    1.512f * 24.0f + 0.28f, 1.512f * 25.0f + 0.28f, 1.512f * 26.0f + 0.28f,
    1.512f * 27.0f + 0.28f, 1.512f * 28.0f + 0.28f, 1.512f * 29.0f + 0.28f,
    1.512f * 30.0f + 0.28f, 1.512f * 31.0f + 0.28f};

class PandarXT : public PandarGeneral_Internal {
  public:
    template <typename... Args>
    PandarXT(Args &&...args) : PandarGeneral_Internal(std::forward<Args>(args)...) {
        Init();
    }

  private:
    void Init();

    virtual std::optional<HS_LIDAR_Packet>
    parseLidarPacket(const std::vector<uint8_t> &recvbuf) final;
};
