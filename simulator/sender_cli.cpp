/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "sender.hpp"
#include <gflags/gflags.h>

DEFINE_uint64(num_threads, 4, "Number of sender threads");
DEFINE_uint64(core_offset, 0, "Core ID of the first sender thread");
DEFINE_uint64(delay, 5000, "Frame duration in microseconds");
DEFINE_string(conf_file, TOSTRING(PROJECT_DIRECTORY) "/data/tddconfig-sim-dl.json", "Config filename");
DEFINE_bool(enable_slow_start, true, "Send frames slowly at first.");

int main(int argc, char* argv[])
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = FLAGS_conf_file;
    auto* cfg = new Config(filename.c_str());
    cfg->genData();

    printf("Starting sender\n");
    auto* sender = new Sender(cfg, FLAGS_num_threads, FLAGS_core_offset,
        FLAGS_delay, FLAGS_enable_slow_start);
    sender->startTX();
    return 0;
}
