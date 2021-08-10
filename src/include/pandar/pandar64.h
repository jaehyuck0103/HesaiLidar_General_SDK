#pragma once

#include "pandarGeneral_internal.h"

#include <cstdint>

#define HS_LIDAR_L64_TIME_SIZE (6)
#define HS_LIDAR_L64_HEAD_SIZE (8)
#define HS_LIDAR_L64_BLOCK_NUMBER (6)

// each block first 2 byte is azimuth
#define HS_LIDAR_L64_BLOCK_HEADER_AZIMUTH (2)
// each block have  64 Unit
#define HS_LIDAR_L64_UNIT_NUM (64)
// each Unit have 3 byte :2 bytes(distance) + 1 byte(intensity)
#define HS_LIDAR_L64_UNIT_SIZE (3)
// total block size 194
#define HS_LIDAR_L64_BLOCK_SIZE                                                                   \
    (HS_LIDAR_L64_UNIT_SIZE * HS_LIDAR_L64_UNIT_NUM + HS_LIDAR_L64_BLOCK_HEADER_AZIMUTH)

// Block tail = timestamp ( 4 bytes ) + factory num (2 bytes)
#define HS_LIDAR_L64_TIMESTAMP_SIZE (4)
#define HS_LIDAR_L64_RETURN_MODE_SIZE (1)
#define HS_LIDAR_L64_FACTORY_SIZE (1)
#define HS_LIDAR_L64_RESERVED_SIZE (8)
#define HS_LIDAR_L64_ENGINE_VELOCITY (2)

// packet body size 1164
#define HS_LIDAR_L64_PACKET_BODY_SIZE (HS_LIDAR_L64_BLOCK_SIZE * HS_LIDAR_L64_BLOCK_NUMBER)

// packet tail size
#define HS_LIDAR_L64_PACKET_TAIL_SIZE (26)
#define HS_LIDAR_L64_PACKET_TAIL_WITHOUT_UDPSEQ_SIZE (22)

// total packet size two type,length: 1198 and 1194
#define HS_LIDAR_L64_PACKET_SIZE                                                                  \
    (HS_LIDAR_L64_HEAD_SIZE + HS_LIDAR_L64_PACKET_BODY_SIZE + HS_LIDAR_L64_PACKET_TAIL_SIZE)

#define HS_LIDAR_L64_PACKET_WITHOUT_UDPSEQ_SIZE                                                   \
    (HS_LIDAR_L64_HEAD_SIZE + HS_LIDAR_L64_PACKET_BODY_SIZE +                                     \
     HS_LIDAR_L64_PACKET_TAIL_WITHOUT_UDPSEQ_SIZE)

class Pandar64 : public PandarGeneral_Internal {

  public:
    template <typename... Args>
    Pandar64(Args &&...args) : PandarGeneral_Internal(std::forward<Args>(args)...) {
        Init();
    }

  private:
    void Init();

    virtual std::optional<HS_LIDAR_Packet>
    parseLidarPacket(const uint8_t *recvbuf, const int len) final;
};
