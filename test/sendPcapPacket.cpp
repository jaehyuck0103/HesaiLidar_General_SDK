#include "pcap_reader.h"

void pcapTest(std::string pcapPath) {

    PcapReader reader{pcapPath, "127.0.0.1", 2368};

    reader.start();
}

int main() {

    pcapTest("../test/test_data/Turning_Left_Pandar64.pcap");

    // pcapTest("../test/test_data/Freeway_Pandar40P.pcap");

    // pcapTest("../test/test_data/PandarQT_Crossroad.pcap");

    return 0;
}
