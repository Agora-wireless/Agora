#include "sender.hpp"
#include <gflags/gflags.h>

DEFINE_uint64(num_threads, 4, "Number of sender threads");
DEFINE_uint64(core_offset, 0, "Core ID of the first sender thread");
DEFINE_uint64(delay, 5000, "Frame duration in microseconds");
DEFINE_string(server_mac_addr, "ff:ff:ff:ff:ff:ff",
    "MAC address of the remote Agora server to send data to");
DEFINE_string(conf_file,
    TOSTRING(PROJECT_DIRECTORY) "/data/tddconfig-sim-ul.json",
    "Config filename");
DEFINE_uint64(enable_slow_start, 1, "Send frames slowly at first.");

int main(int argc, char* argv[])
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = FLAGS_conf_file;
    auto* cfg = new Config(filename.c_str());
    cfg->genData();

    auto* sender = new Sender(cfg, FLAGS_num_threads, FLAGS_core_offset,
        FLAGS_delay, FLAGS_enable_slow_start, FLAGS_server_mac_addr);
    sender->startTX();
    return 0;
}
