#pragma once

#include "pandar_lidar_receiver.h"

#include <cstdint>

class Pandar40P : public PandarLidarReceiver {

  public:
    template <typename... Args>
    Pandar40P(Args &&...args) : PandarLidarReceiver(std::forward<Args>(args)...) {}

  private:
    virtual std::optional<HS_LIDAR_Packet>
    parseLidarPacket(const std::vector<uint8_t> &recvbuf) final;
};
