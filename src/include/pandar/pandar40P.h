#pragma once

#include "pandar_lidar_receiver.h"

#include <cstdint>

#define HS_LIDAR_L40_SOB_SIZE (4)
#define HS_LIDAR_L40_RAW_MEASURE_SIZE (3)
#define HS_LIDAR_L40_LASER_COUNT (40)
#define HS_LIDAR_L40_BLOCKS_PER_PACKET (10)
#define HS_LIDAR_L40_BLOCK_SIZE                                                                   \
    (HS_LIDAR_L40_RAW_MEASURE_SIZE * HS_LIDAR_L40_LASER_COUNT + HS_LIDAR_L40_SOB_SIZE)
#define HS_LIDAR_L40_TIMESTAMP_SIZE (4)
#define HS_LIDAR_L40_FACTORY_INFO_SIZE (1)
#define HS_LIDAR_L40_ECHO_SIZE (1)
#define HS_LIDAR_L40_RESERVE_SIZE (8)
#define HS_LIDAR_L40_REVOLUTION_SIZE (2)
#define HS_LIDAR_L40_INFO_SIZE                                                                    \
    (HS_LIDAR_L40_TIMESTAMP_SIZE + HS_LIDAR_L40_FACTORY_INFO_SIZE + HS_LIDAR_L40_ECHO_SIZE +      \
     HS_LIDAR_L40_RESERVE_SIZE + HS_LIDAR_L40_REVOLUTION_SIZE)
#define HS_LIDAR_L40_UTC_TIME (6)
#define HS_LIDAR_L40_PACKET_SIZE                                                                  \
    (HS_LIDAR_L40_BLOCK_SIZE * HS_LIDAR_L40_BLOCKS_PER_PACKET + HS_LIDAR_L40_INFO_SIZE +          \
     HS_LIDAR_L40_UTC_TIME)
#define HS_LIDAR_L40_SEQ_NUM_SIZE (4)

class Pandar40P : public PandarLidarReceiver {

  public:
    template <typename... Args>
    Pandar40P(Args &&...args) : PandarLidarReceiver(std::forward<Args>(args)...) {}

  private:
    virtual std::optional<HS_LIDAR_Packet>
    parseLidarPacket(const std::vector<uint8_t> &recvbuf) final;
};
