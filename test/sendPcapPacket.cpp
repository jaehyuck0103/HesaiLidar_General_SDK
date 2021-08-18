#include "pcap_reader.h"

void pcapTest(std::string pcapPath, std::string angleCorrectionPath) {

    PcapReader reader{pcapPath, "127.0.0.1", 2368};

    reader.start();
}

int main() {

    pcapTest(
        "../test/test_data/Turning_Left_Pandar64.pcap",
        "../test/test_data/angle_correction_Pandar64.csv");

    /*
    pcapTest(
        "../test/test_data/Freeway_Pandar40P.pcap",
        "../test/test_data/correction file_Pandar40P.csv");
    */

    /*
    pcapTest(
        "../test/test_data/PandarQT_Crossroad.pcap",
        "../test/test_data/PandarQT Correction file.csv");
    */

    return 0;
}
