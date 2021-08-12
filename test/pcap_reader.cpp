#include "pcap_reader.h"

#include <chrono>
#include <iostream>
#include <map>

using namespace std::chrono_literals;

constexpr int PKT_HEADER_SIZE = 42;

PcapReader::PcapReader(std::string pcapPath, std::string dstAddr, int dstPort)
    : socket_(io_context_) {

    socket_.connect(udp::endpoint(asio::ip::make_address_v4(dstAddr), dstPort));

    pcapPath_ = pcapPath;
}

PcapReader::~PcapReader() { stop(); }

void PcapReader::start() { parsePcap(); }

void PcapReader::stop() {}

void PcapReader::parsePcap() {

    char pcapBuf[PCAP_ERRBUF_SIZE];
    pcap_t *pcapFile = pcap_open_offline(pcapPath_.c_str(), pcapBuf);
    if (pcapFile == NULL) {
        std::cout << "open pcap file " << pcapPath_ << " fail\n";
        return;
    }

    bpf_program filter;
    if (pcap_compile(pcapFile, &filter, "udp", 0, 0xffffffff) == -1) {
        std::cout << "compile pcap file fail\n";
        return;
    }
    if (pcap_setfilter(pcapFile, &filter) == -1) {
        std::cout << "pcap set filter fail\n";
        return;
    }

    pcap_pkthdr *pktHeader;
    const uint8_t *packetBuf;
    while (pcap_next_ex(pcapFile, &pktHeader, &packetBuf) >= 0) {
        const uint8_t *packet = packetBuf + PKT_HEADER_SIZE;
        int pktSize = pktHeader->len - PKT_HEADER_SIZE;

        asio::error_code ec;
        socket_.send(asio::buffer(packet, pktSize), 0, ec);
        if (ec) {
            std::cout << ec.message() << std::endl;
        }

        std::this_thread::sleep_for(1ms);
    }

    if (pcapFile != NULL) {
        pcap_close(pcapFile);
        pcapFile = NULL;
    }
}
