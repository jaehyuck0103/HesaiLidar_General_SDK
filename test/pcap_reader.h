#include "pcap.h"

#include <asio.hpp>

#include <string>
#include <thread>

using asio::ip::udp;

class PcapReader {
  public:
    PcapReader(std::string pcapPath, std::string dstAddr, int dstPort);
    ~PcapReader();

    void start();
    void stop();

  private:
    std::string pcapPath_;

    void parsePcap();

    asio::io_context io_context_;
    udp::socket socket_;
};
