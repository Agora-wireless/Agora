/**
 * @file sender_cli.cc
 * @brief Main file for the sender executable
 */
#include <gflags/gflags.h>

#include "ul_mac_sender.h"

DEFINE_uint64(num_threads, 1, "Number of mac client sender threads");
DEFINE_uint64(core_offset, 0, "Core ID of the first sender thread");
DEFINE_uint64(frame_duration, 0, "Frame duration in microseconds");
DEFINE_string(conf_file,
              TOSTRING(PROJECT_DIRECTORY) "/data/tddconfig-sim-ul.json",
              "Config filename");
DEFINE_string(data_file, TOSTRING(PROJECT_DIRECTORY) "/data/test_file.bin",
              "Uplink transmit filename");
DEFINE_uint64(
    enable_slow_start, 1,
    "Send frames slower than the specified frame duration during warmup");

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "num_threads, core_offset, frame_duration, conf_file, "
      "data_file, enable_slow_start");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  std::string filename = FLAGS_conf_file;
  {
    auto cfg = std::make_unique<Config>(filename.c_str());
    cfg->GenData();
    {
      auto sender = std::make_unique<UlMacSender>(
          cfg.get(), FLAGS_data_file, FLAGS_num_threads, FLAGS_core_offset,
          FLAGS_frame_duration, 0, FLAGS_enable_slow_start);
      sender->StartTx();
    }  // end context sender
  }    // end context Config
  gflags::ShutDownCommandLineFlags();
  return 0;
}
