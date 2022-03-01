#include "dynamic_sender.hpp"
// #include <gflags/gflags.h>

// DEFINE_uint64(num_threads, 4, "Number of sender threads");
// DEFINE_uint64(num_masters, 1, "Number of master threads");
// DEFINE_uint64(core_offset, 0, "Core ID of the first sender thread");
// DEFINE_uint64(frame_duration, 5000, "Frame duration in microseconds");
// DEFINE_string(server_mac_addr, "ff:ff:ff:ff:ff:ff",
//     "MAC address of the remote Agora server to send data to");
// DEFINE_string(conf_file,
//     TOSTRING(PROJECT_DIRECTORY) "/data/tddconfig-sim-ul.json",
//     "Config filename");
// DEFINE_uint64(enable_slow_start, 1,
//     "Send frames slower than the specified frame duration during warmup");

int main(int argc, char* argv[])
{
    int opt;
    std::string conf_file = TOSTRING(PROJECT_DIRECTORY) "/data/tddconfig-sim-ul.json";
    std::string server_mac_addr = "ff:ff:ff:ff:ff:ff";
    uint64_t num_threads = 4;
    uint64_t num_masters = 1;
    uint64_t core_offset = 0;
    uint64_t frame_duration = 5000;
    uint64_t enable_slow_start = 1;
    while ((opt = getopt(argc, argv, "c:s:t:m:o:e")) != -1) {
        switch (opt) {
            case 'c':
                conf_file = optarg;
                break;
            case 's':
                server_mac_addr = optarg;
                break;
            case 't':
                num_threads = std::stoul(optarg);
                break;
            case 'm':
                num_masters = std::stoul(optarg);
                break;
            case 'o':
                core_offset = std::stoul(optarg);
                break;
            case 'e':
                enable_slow_start = 0;
                break;
            default:
                std::cerr << "Usage: " << argv[0] << " [-c conf_file] [-s server_mac_addr] [-t num_threads] [-m num_masters] [-o core_offset] [-e]" << std::endl;
                exit(EXIT_FAILURE);
        }
    }

    // gflags::ParseCommandLineFlags(&argc, &argv, true);
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = conf_file;
    auto* cfg = new Config(filename.c_str());
    cfg->genData();
    frame_duration = cfg->slot_us;

    // auto* sender = new Sender(cfg, FLAGS_num_masters, FLAGS_num_threads, FLAGS_core_offset,
    //     FLAGS_frame_duration, FLAGS_enable_slow_start, FLAGS_server_mac_addr);
    auto* sender = new Sender(cfg, num_masters, num_threads, core_offset,
        frame_duration, enable_slow_start, server_mac_addr);
    double* frame_start = new double[kNumStatsFrames]();
    double* frame_end = new double[kNumStatsFrames]();
    sender->startTXfromMain(frame_start, frame_end);
    sender->join_thread();
    return 0;
}
