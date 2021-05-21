/**
 * @file mac_client.cc
 * @brief Main file for the mac client executable. This will send data to the
 * mac thread of the UE.
 */
#include <gflags/gflags.h>

#include "ul_mac_sender.h"

DEFINE_uint64(num_threads, 1, "Number of mac client sender threads");
DEFINE_uint64(core_offset, 1, "Core ID of the first sender thread");
DEFINE_uint64(frame_duration, 0, "Frame duration in microseconds");
DEFINE_string(conf_file, TOSTRING(PROJECT_DIRECTORY) "/data/ue-mac-sim.json",
              "Config filename");
DEFINE_string(data_file, TOSTRING(PROJECT_DIRECTORY) "/data/increment_file.bin",
              "Uplink transmit filename");
DEFINE_uint64(
    enable_slow_start, 0,
    "Send frames slower than the specified frame duration during warmup");

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "num_threads, core_offset, frame_duration, conf_file, "
      "data_file, enable_slow_start");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  std::string filename = FLAGS_conf_file;
  std::string data_filename = FLAGS_data_file;
  {
    auto cfg = std::make_unique<Config>(filename.c_str());
    cfg->GenData();

    // Generate pattern file for testing
    if (data_filename == "") {
      std::ofstream create_file;
      data_filename =
          TOSTRING(PROJECT_DIRECTORY) + std::string("/data/increment_file.bin");
      std::printf("Generating test binary file %s\n", data_filename.c_str());

      create_file.open(
          data_filename,
          (std::ofstream::out | std::ofstream::binary | std::ofstream::trunc));
      assert(create_file.is_open() == true);

      std::vector<char> mac_data;
      // mac_data.resize(cfg->UlMacDataBytesNumPerframe());
      mac_data.resize(cfg->MacPayloadLength());

      for (size_t i = 0;
           i < (cfg->FramesToTest() * cfg->UlMacPacketsPerframe()); i++) {
        std::fill(mac_data.begin(), mac_data.end(), (char)i);
        create_file.write(mac_data.data(), mac_data.size());
      }
      create_file.close();
    }

    {
      auto sender = std::make_unique<UlMacSender>(
          cfg.get(), data_filename, FLAGS_num_threads, FLAGS_core_offset,
          FLAGS_frame_duration, 0, FLAGS_enable_slow_start);
      sender->StartTx();
    }  // end context sender
  }    // end context Config
  gflags::ShutDownCommandLineFlags();
  return 0;
}
