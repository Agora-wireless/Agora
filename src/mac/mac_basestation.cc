/**
 * @file mac_basestation.cc
 * @brief Main file for the mac bs transmitter/receiver executable.
 */
#include <gflags/gflags.h>

#include "dl_mac_sender.h"
#include "signal_handler.h"
#include "ul_mac_receiver.h"

DEFINE_uint64(num_sender_threads, 1,
              "Number of mac basestation sender threads");
DEFINE_uint64(num_receiver_threads, 1,
              "Number of mac basestation receiver threads");
DEFINE_uint64(core_offset, 3, "Core ID of the first sender thread");
DEFINE_uint64(frame_duration, 0, "Frame duration in microseconds");
DEFINE_string(conf_file, TOSTRING(PROJECT_DIRECTORY) "/data/bs-mac-sim.json",
              "Config filename");
DEFINE_string(data_file,
              TOSTRING(PROJECT_DIRECTORY) "/data/dl_increment_file.bin",
              "Downlink transmit filename");
DEFINE_uint64(
    enable_slow_start, 0,
    "Send frames slower than the specified frame duration during warmup");

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "num_sender_threads, num_receiver_threads, core_offset, frame_duration, "
      "conf_file, "
      "data_file, enable_slow_start");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  std::string filename = FLAGS_conf_file;
  std::string data_filename = FLAGS_data_file;
  int ret = EXIT_FAILURE;
  {
    auto cfg = std::make_unique<Config>(filename.c_str());
    cfg->GenData();

    // Generate pattern file for testing
    if (data_filename == "") {
      std::ofstream create_file;
      data_filename = TOSTRING(PROJECT_DIRECTORY) +
                      std::string("/data/dl_increment_file.bin");
      std::printf("Generating test binary file %s\n", data_filename.c_str());

      create_file.open(
          data_filename,
          (std::ofstream::out | std::ofstream::binary | std::ofstream::trunc));
      assert(create_file.is_open() == true);

      std::vector<char> mac_data;
      mac_data.resize(cfg->MacPayloadLength());

      for (size_t i = 0;
           i < (cfg->FramesToTest() * cfg->DlMacPacketsPerframe()); i++) {
        std::fill(mac_data.begin(), mac_data.end(), (char)i);
        create_file.write(mac_data.data(), mac_data.size());
      }
      create_file.close();
    }

    try {
      SignalHandler signal_handler;

      // Register signal handler to handle kill signal
      signal_handler.SetupSignalHandlers();
      if (cfg->Frame().NumDlDataSyms() > 0) {
        auto sender = std::make_unique<DlMacSender>(
            cfg.get(), data_filename, FLAGS_num_sender_threads,
            FLAGS_core_offset, FLAGS_frame_duration, 0,
            FLAGS_enable_slow_start);
        sender->StartTx();
      }
      if (cfg->Frame().NumUlDataSyms() > 0) {
        auto receiver_ = std::make_unique<UlMacReceiver>(
            cfg.get(), FLAGS_num_receiver_threads,
            FLAGS_core_offset + FLAGS_num_sender_threads);
        std::vector<std::thread> rx_threads = receiver_->StartRecv();
        for (auto& thread : rx_threads) {
          thread.join();
        }
      }
      ret = EXIT_SUCCESS;
    } catch (SignalException& e) {
      std::cerr << "SignalException: " << e.what() << std::endl;
      ret = EXIT_FAILURE;
    }
  }  // end context Config
  std::printf("Shutdown Basestation App!\n");
  gflags::ShutDownCommandLineFlags();
  return ret;
}
