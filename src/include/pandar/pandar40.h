#pragma once

#include <cstdint>
#include <ctime>

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
#define HS_LIDAR_L40_LASER_RETURN_TO_DISTANCE_RATE (0.004)
#define HS_LIDAR_L40_SEQ_NUM_SIZE (4)

struct HS_LIDAR_L40_Unit {
    uint8_t intensity;
    double distance;
};

struct HS_LIDAR_L40_Block {
    uint16_t azimuth;
    uint16_t sob;
    HS_LIDAR_L40_Unit units[HS_LIDAR_L40_LASER_COUNT];
};

struct HS_LIDAR_L40_Packet {
    HS_LIDAR_L40_Block blocks[HS_LIDAR_L40_BLOCKS_PER_PACKET];
    tm t;
    uint32_t usec;
    int echo;
};

// elevation angle of each line for HS Line 40 Lidar, Line 1 - Line 40
const float pandar40p_elev_angle_map[] = {
    15.0f, 11.0f,  8.0f,   5.0f,   3.0f,   2.0f,   1.67f,  1.33f,  1.0f,   0.67f,
    0.33f, 0.0f,   -0.33f, -0.67f, -1.0f,  -1.33f, -1.66f, -2.0f,  -2.33f, -2.67f,
    -3.0f, -3.33f, -3.67f, -4.0f,  -4.33f, -4.67f, -5.0f,  -5.33f, -5.67f, -6.0f,
    -7.0f, -8.0f,  -9.0f,  -10.0f, -11.0f, -12.0f, -13.0f, -14.0f, -19.0f, -25.0f};

// Line 40 Lidar azimuth Horizatal offset ,  Line 1 - Line 40
const float pandar40p_horizatal_azimuth_offset_map[] = {
    -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, 3.125f,  -5.208f, -1.042f, 3.125f,
    -5.208f, -1.042f, 3.125f,  -5.208f, -1.042f, 3.125f,  -5.208f, -1.042f, 3.125f,  -5.208f,
    -1.042f, 3.125f,  -5.208f, -1.042f, 3.125f,  -5.208f, -1.042f, 3.125f,  -5.208f, -1.042f,
    -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f};
