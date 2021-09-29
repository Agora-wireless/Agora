/**
 * @file mac_client.cc
 * @brief Main file for the mac client executable.
 */
#include <gflags/gflags.h>

#include <chrono>
#include <thread>

#include "mac_receiver.h"
#include "mac_sender.h"
#include "signal_handler.h"
#include "version_config.h"

DEFINE_uint64(num_sender_worker_threads, 1,
              "Number of mac client sender worker threads");
DEFINE_uint64(num_sender_update_threads, 1,
              "Number of mac client sender update threads / streams");
DEFINE_uint64(num_receiver_threads, 1, "Number of mac client receiver threads");
DEFINE_uint64(core_offset, 1, "Core ID of the first sender thread");
DEFINE_uint64(frame_duration, 0, "Frame duration in microseconds");
DEFINE_string(conf_file, TOSTRING(PROJECT_DIRECTORY) "/data/ue-mac-sim.json",
              "Config filename");
DEFINE_string(data_file,
              TOSTRING(PROJECT_DIRECTORY) "/data/ul_increment_file.bin",
              "Uplink transmit filename");
DEFINE_uint64(
    enable_slow_start, 0,
    "Send frames slower than the specified frame duration during warmup");

int main(int argc, char* argv[]) {
  PinToCoreWithOffset(ThreadType::kMaster, FLAGS_core_offset, 0);
  gflags::SetVersionString(GetAgoraProjectVersion());

  gflags::SetUsageMessage(
      "num_sender_threads, num_receiver_threads, core_offset, frame_duration, "
      "conf_file, data_file, enable_slow_start");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  std::string filename = FLAGS_conf_file;
  std::string data_filename = FLAGS_data_file;

  auto frame_start = new double[kNumStatsFrames];
  auto frame_end = new double[kNumStatsFrames];

  int ret = EXIT_FAILURE;
  {
    auto cfg = std::make_unique<Config>(filename.c_str());
    cfg->GenData();

    // Generate pattern file for testing
    if (data_filename == "") {
      std::ofstream create_file;
      data_filename = TOSTRING(PROJECT_DIRECTORY) +
                      std::string("/data/ul_increment_file.bin");
      std::printf("Generating test binary file for user uplink%s\n",
                  data_filename.c_str());

      create_file.open(
          data_filename,
          (std::ofstream::out | std::ofstream::binary | std::ofstream::trunc));
      assert(create_file.is_open() == true);

      std::vector<char> mac_data;
      mac_data.resize(cfg->MacPayloadLength());

      for (size_t i = 0;
           i < (cfg->FramesToTest() * cfg->UlMacPacketsPerframe()); i++) {
        std::fill(mac_data.begin(), mac_data.end(), (char)i);
        create_file.write(mac_data.data(), mac_data.size());
      }
      create_file.close();
    }

    try {
      SignalHandler signal_handler;
      std::unique_ptr<MacSender> sender;
      std::unique_ptr<MacReceiver> receiver;
      std::vector<std::thread> rx_threads;
      //+1 for main thread
      const size_t kNumTotalSenderThreads =
          FLAGS_num_sender_worker_threads + 1 + FLAGS_num_sender_update_threads;

      // Register signal handler to handle kill signal
      signal_handler.SetupSignalHandlers();
      if (cfg->Frame().NumUlDataSyms() > 0) {
        sender = std::make_unique<MacSender>(
            cfg.get(), data_filename, cfg->UlMacPacketsPerframe(),
            cfg->UeServerAddr(), cfg->UeMacRxPort(),
            std::bind(&FrameStats::GetULDataSymbol, cfg->Frame(),
                      std::placeholders::_1),
            FLAGS_core_offset + 1, FLAGS_num_sender_worker_threads,
            FLAGS_num_sender_update_threads, FLAGS_frame_duration, 0,
            FLAGS_enable_slow_start, true);
        sender->StartTXfromMain(frame_start, frame_end);
      }
      if (cfg->Frame().NumDlDataSyms() > 0) {
        receiver = std::make_unique<MacReceiver>(
            cfg.get(), cfg->DlMacDataBytesNumPerframe(), cfg->UeServerAddr(),
            cfg->UeMacTxPort(), FLAGS_num_receiver_threads,
            FLAGS_core_offset + kNumTotalSenderThreads);
        rx_threads = receiver->StartRecv();
      }

      std::printf("Running mac client application\n");
      while ((cfg->Running() == true) &&
             (SignalHandler::GotExitSignal() == false)) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }

      /* May want to move this exit section to after the exception */
      cfg->Running(false);
      std::printf("Terminating mac client application\n");
      sender.reset();
      for (auto& thread : rx_threads) {
        thread.join();
      }
      receiver.reset();
      ret = EXIT_SUCCESS;
    } catch (SignalException& e) {
      std::cerr << "SignalException: " << e.what() << std::endl;
      cfg->Running(false);
      ret = EXIT_FAILURE;
    }
  }  // end context Config
  delete[](frame_start);
  delete[](frame_end);
  std::printf("Mac user application terminated!\n");
  gflags::ShutDownCommandLineFlags();
  return ret;
}
