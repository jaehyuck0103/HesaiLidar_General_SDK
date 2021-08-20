#pragma once

#include "pandar_lidar_receiver.h"

#include <cstdint>

class PandarXT : public PandarLidarReceiver {
  public:
    template <typename... Args>
    PandarXT(Args &&...args) : PandarLidarReceiver(std::forward<Args>(args)...) {}

  private:
    virtual std::optional<HS_LIDAR_Packet>
    parseLidarPacket(const std::vector<uint8_t> &recvbuf) final;
};
