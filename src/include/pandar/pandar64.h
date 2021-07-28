#pragma once

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
#define HS_LIDAR_L64_ECHO_SIZE (1)
#define HS_LIDAR_L64_FACTORY_SIZE (1)
#define HS_LIDAR_L64_RESERVED_SIZE (8)
#define HS_LIDAR_L64_ENGINE_VELOCITY (2)

// packet body size two type
#define HS_LIDAR_L64_PACKET_BODY_SIZE (HS_LIDAR_L64_BLOCK_SIZE * HS_LIDAR_L64_BLOCK_NUMBER)

// packet tail size
#define HS_LIDAR_L64_PACKET_TAIL_SIZE (26)
#define HS_LIDAR_L64_PACKET_TAIL_WITHOUT_UDPSEQ_SIZE (22)

// total packet size two type,length: 1198 and 1392
#define HS_LIDAR_L64_PACKET_SIZE                                                                  \
    (HS_LIDAR_L64_HEAD_SIZE + HS_LIDAR_L64_PACKET_BODY_SIZE + HS_LIDAR_L64_PACKET_TAIL_SIZE)

#define HS_LIDAR_L64_PACKET_WITHOUT_UDPSEQ_SIZE                                                   \
    (HS_LIDAR_L64_HEAD_SIZE + HS_LIDAR_L64_PACKET_BODY_SIZE +                                     \
     HS_LIDAR_L64_PACKET_TAIL_WITHOUT_UDPSEQ_SIZE)

struct HS_LIDAR_L64_Header {
    uint16_t sob = 0;          // 0xFFEE 2bytes
    uint8_t chLaserNumber = 0; // laser number 1byte
    uint8_t chBlockNumber = 0; // block number 1byte
    uint8_t chReturnType = 0;  // return mode 1 byte  when dual return 0-Single Return
                               // 1-The first block is the 1 st return.
                               // 2-The first block is the 2 nd return
    uint8_t chDisUnit = 0;     // Distance unit, 6mm/5mm/4mm
};

struct HS_LIDAR_L64_Unit {
    double distance;
    uint16_t intensity;
};

struct HS_LIDAR_L64_Block {
    uint16_t azimuth; // Azimuth = RealAzimuth * 100
    HS_LIDAR_L64_Unit units[HS_LIDAR_L64_UNIT_NUM];
};

struct HS_LIDAR_L64_Packet {
    HS_LIDAR_L64_Header header;
    HS_LIDAR_L64_Block blocks[HS_LIDAR_L64_BLOCK_NUMBER];
    uint32_t timestamp;
    uint32_t echo;
    uint8_t addtime[6];
};

const float pandar64_elev_angle_map[] = {
    14.882f,  11.032f, 8.059f,   5.057f,  3.04f,   2.028f,  1.86f,   1.688f,  1.522f,   1.351f,
    1.184f,   1.013f,  0.846f,   0.675f,  0.508f,  0.337f,  0.169f,  0.0f,    -0.169f,  -0.337f,
    -0.508f,  -0.675f, -0.845f,  -1.013f, -1.184f, -1.351f, -1.522f, -1.688f, -1.86f,   -2.028f,
    -2.198f,  -2.365f, -2.536f,  -2.7f,   -2.873f, -3.04f,  -3.21f,  -3.375f, -3.548f,  -3.712f,
    -3.884f,  -4.05f,  -4.221f,  -4.385f, -4.558f, -4.72f,  -4.892f, -5.057f, -5.229f,  -5.391f,
    -5.565f,  -5.726f, -5.898f,  -6.061f, -7.063f, -8.059f, -9.06f,  -9.885f, -11.032f, -12.006f,
    -12.974f, -13.93f, -18.889f, -24.897f};

const float pandar64_horizatal_azimuth_offset_map[] = {
    -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, 1.042f,  3.125f,  5.208f,  -5.208f,
    -3.125f, -1.042f, 1.042f,  3.125f,  5.208f,  -5.208f, -3.125f, -1.042f, 1.042f,  3.125f,
    5.208f,  -5.208f, -3.125f, -1.042f, 1.042f,  3.125f,  5.208f,  -5.208f, -3.125f, -1.042f,
    1.042f,  3.125f,  5.208f,  -5.208f, -3.125f, -1.042f, 1.042f,  3.125f,  5.208f,  -5.208f,
    -3.125f, -1.042f, 1.042f,  3.125f,  5.208f,  -5.208f, -3.125f, -1.042f, 1.042f,  3.125f,
    5.208f,  -5.208f, -3.125f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f,
    -1.042f, -1.042f, -1.042f, -1.042f};
