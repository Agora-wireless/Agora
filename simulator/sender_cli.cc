#include <gflags/gflags.h>

#include "sender.h"

DEFINE_uint64(num_threads, 4, "Number of sender threads");
DEFINE_uint64(core_offset, 0, "Core ID of the first sender thread");
DEFINE_uint64(frame_duration, 5000, "Frame duration in microseconds");
DEFINE_string(server_mac_addr, "ff:ff:ff:ff:ff:ff",
              "MAC address of the remote Agora server to send data to");
DEFINE_string(conf_file,
              TOSTRING(PROJECT_DIRECTORY) "/data/tddconfig-sim-ul.json",
              "Config filename");
DEFINE_uint64(
    enable_slow_start, 1,
    "Send frames slower than the specified frame duration during warmup");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  std::string filename = FLAGS_conf_file;
  auto* cfg = new Config(filename.c_str());
  cfg->GenData();

  auto* sender = new Sender(cfg, FLAGS_num_threads, FLAGS_core_offset,
                            FLAGS_frame_duration, FLAGS_enable_slow_start,
                            FLAGS_server_mac_addr);
  sender->StartTx();
  return 0;
}
