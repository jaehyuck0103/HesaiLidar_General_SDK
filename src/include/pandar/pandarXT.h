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
