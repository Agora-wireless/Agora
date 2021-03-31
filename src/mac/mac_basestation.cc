/**
 * @file mac_basestation.cc
 * @brief Main file for the mac bs receiver executable. This will receive data
 * from the ue uplink
 */
#include <gflags/gflags.h>

#include "signal_handler.h"
#include "ul_mac_receiver.h"

DEFINE_uint64(num_threads, 1, "Number of mac receiver threads");
DEFINE_uint64(core_offset, 0, "Core ID of the first sender thread");
DEFINE_string(conf_file, TOSTRING(PROJECT_DIRECTORY) "/data/bs-ul-sim.json",
              "Config filename");

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage("num_threads, core_offset, conf_file");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  std::string filename = FLAGS_conf_file;

  auto cfg = std::make_unique<Config>(filename.c_str());
  cfg->GenData();
  int ret = EXIT_FAILURE;
  try {
    SignalHandler signal_handler;

    // Register signal handler to handle kill signal
    signal_handler.SetupSignalHandlers();
    auto receiver_ = std::make_unique<UlMacReceiver>(
        cfg.get(), FLAGS_num_threads, FLAGS_core_offset);
    std::vector<std::thread> rx_threads = receiver_->StartRecv();
    for (auto& thread : rx_threads) {
      thread.join();
    }
    ret = EXIT_SUCCESS;
  } catch (SignalException& e) {
    std::cerr << "SignalException: " << e.what() << std::endl;
    ret = EXIT_FAILURE;
  }
  std::printf("Shutdown\n");
  gflags::ShutDownCommandLineFlags();
  return ret;
}
