/**
 * @file sender_cli.cc
 * @brief Main file for the sender executable
 */
#include <gflags/gflags.h>

#include "logger.h"
#include "sender.h"
#include "version_config.h"

DEFINE_uint64(num_threads, 4, "Number of sender threads");
DEFINE_uint64(core_offset, 0, "Core ID of the first sender thread");
DEFINE_uint64(frame_duration, 0, "Frame duration in microseconds");
DEFINE_uint64(inter_frame_delay, 0, "Delay between two frames in microseconds");
DEFINE_string(server_mac_addr, "ff:ff:ff:ff:ff:ff",
              "MAC address of the remote Agora server to send data to");
DEFINE_string(
    conf_file,
    TOSTRING(PROJECT_DIRECTORY) "/files/config/ci/tddconfig-sim-ul.json",
    "Config filename");
DEFINE_uint64(
    enable_slow_start, 1,
    "Send frames slower than the specified frame duration during warmup");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  gflags::SetVersionString(GetAgoraProjectVersion());
  std::string filename = FLAGS_conf_file;
  AGORA_LOG_INIT();

  {
    auto cfg = std::make_unique<Config>(filename.c_str());
    cfg->GenData();
    {
      auto sender = std::make_unique<Sender>(
          cfg.get(), FLAGS_num_threads, FLAGS_core_offset, FLAGS_frame_duration,
          FLAGS_inter_frame_delay, FLAGS_enable_slow_start,
          FLAGS_server_mac_addr);
      sender->StartTx();
    }  // end context sender
  }    // end context Config

  PrintCoreAssignmentSummary();
  gflags::ShutDownCommandLineFlags();
  AGORA_LOG_SHUTDOWN();
  return 0;
}
