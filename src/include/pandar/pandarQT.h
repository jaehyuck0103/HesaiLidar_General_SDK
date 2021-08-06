#pragma once

#include "pandarGeneral_internal.h"

#include <cstdint>

// Head
#define HS_LIDAR_QT_HEAD_SIZE (12)
#define HS_LIDAR_QT_PRE_HEADER_SIZE (6)
#define HS_LIDAR_QT_HEADER_SIZE (6)
// Body
#define HS_LIDAR_QT_BLOCK_NUMBER (4)
#define HS_LIDAR_QT_BLOCK_HEADER_AZIMUTH (2)
#define HS_LIDAR_QT_UNIT_NUM (64)
#define HS_LIDAR_QT_UNIT_SIZE (4)
#define HS_LIDAR_QT_BLOCK_SIZE                                                                    \
    (HS_LIDAR_QT_UNIT_SIZE * HS_LIDAR_QT_UNIT_NUM + HS_LIDAR_QT_BLOCK_HEADER_AZIMUTH)
#define HS_LIDAR_QT_BODY_SIZE (HS_LIDAR_QT_BLOCK_SIZE * HS_LIDAR_QT_BLOCK_NUMBER)
// Tail
#define HS_LIDAR_QT_RESERVED_SIZE (10)
#define HS_LIDAR_QT_ENGINE_VELOCITY (2)
#define HS_LIDAR_QT_TIMESTAMP_SIZE (4)
#define HS_LIDAR_QT_ECHO_SIZE (1)
#define HS_LIDAR_QT_FACTORY_SIZE (1)
#define HS_LIDAR_QT_UTC_SIZE (6)
#define HS_LIDAR_QT_SEQUENCE_SIZE (4)
#define HS_LIDAR_QT_PACKET_TAIL_SIZE (28)
#define HS_LIDAR_QT_PACKET_TAIL_WITHOUT_UDPSEQ_SIZE (24)
// All
#define HS_LIDAR_QT_PACKET_SIZE                                                                   \
    (HS_LIDAR_QT_HEAD_SIZE + HS_LIDAR_QT_BODY_SIZE + HS_LIDAR_QT_PACKET_TAIL_SIZE)
#define HS_LIDAR_QT_PACKET_WITHOUT_UDPSEQ_SIZE                                                    \
    (HS_LIDAR_QT_HEAD_SIZE + HS_LIDAR_QT_BODY_SIZE + HS_LIDAR_QT_PACKET_TAIL_WITHOUT_UDPSEQ_SIZE)

class PandarQT : public PandarGeneral_Internal {

  public:
    template <typename... Args>
    PandarQT(Args &&...args) : PandarGeneral_Internal(std::forward<Args>(args)...) {
        Init();
    }

  private:
    void Init();

    virtual int ParseData(HS_LIDAR_Packet *packet, const uint8_t *recvbuf, const int len) final;
};
