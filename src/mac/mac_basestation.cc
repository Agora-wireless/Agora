/**
 * @file mac_basestation.cc
 * @brief Main file for the mac bs transmitter/receiver executable.
 */
#include <gflags/gflags.h>

#include <chrono>
#include <thread>

#include "logger.h"
#include "mac_receiver.h"
#include "mac_sender.h"
#include "signal_handler.h"
#include "version_config.h"

DEFINE_uint64(num_sender_worker_threads, 1,
              "Number of mac basestation sender worker threads");
DEFINE_uint64(num_sender_update_threads, 1,
              "Number of mac client sender update threads / streams");
DEFINE_uint64(num_receiver_threads, 1,
              "Number of mac basestation receiver threads");
DEFINE_uint64(core_offset, 6, "Core ID of the first sender thread");
DEFINE_uint64(frame_duration, 0, "Frame duration in microseconds");
DEFINE_string(
    conf_file,
    TOSTRING(PROJECT_DIRECTORY) "/files/config/examples/bs-mac-sim.json",
    "Config filename");
DEFINE_string(
    data_file,
    TOSTRING(PROJECT_DIRECTORY) "/files/experiment/dl_increment_file.bin",
    "Downlink transmit filename");

DEFINE_string(fwd_udp_address, "", "Forward decoded mac data to address");
DEFINE_uint64(fwd_udp_port, 0,
              "Forward decoded mac data port id (Set to 0 to disable)");

DEFINE_uint64(
    enable_slow_start, 0,
    "Send frames slower than the specified frame duration during warmup");

int main(int argc, char* argv[]) {
  gflags::SetVersionString(GetAgoraProjectVersion());
  gflags::SetUsageMessage(
      "num_sender_threads, num_receiver_threads, core_offset, frame_duration, "
      "conf_file, data_file, enable_slow_start");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  std::string filename = FLAGS_conf_file;
  std::string data_filename = FLAGS_data_file;
  AGORA_LOG_INIT();

  auto* frame_start = new double[kNumStatsFrames];
  auto* frame_end = new double[kNumStatsFrames];

  int ret = EXIT_FAILURE;
  {
    auto cfg = std::make_unique<Config>(filename.c_str());
    cfg->GenData();

    // Generate pattern file for testing
    if (data_filename.empty()) {
      std::ofstream create_file;
      data_filename = TOSTRING(PROJECT_DIRECTORY) +
                      std::string("/files/experiment/dl_increment_file.bin");
      std::printf(
          "Generating test binary file for basestation downlink %s.  Frames: "
          "%zu, Packets: %zu, Packet Size: %zu\n",
          data_filename.c_str(), cfg->FramesToTest(),
          cfg->MacPacketsPerframe(Direction::kDownlink),
          cfg->MacPayloadMaxLength(Direction::kDownlink));

      create_file.open(
          data_filename,
          (std::ofstream::out | std::ofstream::binary | std::ofstream::trunc));
      assert(create_file.is_open() == true);

      std::vector<char> mac_data;
      mac_data.resize(cfg->MacPayloadMaxLength(Direction::kDownlink));

      for (size_t i = 0; i < (cfg->FramesToTest() *
                              cfg->MacPacketsPerframe(Direction::kDownlink));
           i++) {
        std::fill(mac_data.begin(), mac_data.end(), (char)i);
        create_file.write(mac_data.data(), mac_data.size());
      }
      create_file.close();
    }

    PinToCoreWithOffset(ThreadType::kMaster, FLAGS_core_offset, 0, true);

    try {
      SignalHandler signal_handler;
      std::unique_ptr<MacSender> sender;
      std::unique_ptr<MacReceiver> receiver;
      std::vector<std::thread> rx_threads;
      //+1 for main thread a
      const size_t num_total_sender_threads =
          FLAGS_num_sender_worker_threads + FLAGS_num_sender_update_threads;
      size_t thread_start = FLAGS_core_offset;

      // Register signal handler to handle kill signal
      signal_handler.SetupSignalHandlers();
      if (cfg->Frame().NumDlDataSyms() > 0) {
        sender = std::make_unique<MacSender>(
            cfg.get(), data_filename,
            cfg->MacPacketLength(Direction::kDownlink),
            cfg->MacPayloadMaxLength(Direction::kDownlink),
            cfg->MacPacketsPerframe(Direction::kDownlink), cfg->BsServerAddr(),
            cfg->BsMacRxPort(),
            std::bind(&FrameStats::GetDLDataSymbol, cfg->Frame(),
                      std::placeholders::_1),
            thread_start, FLAGS_num_sender_worker_threads,
            FLAGS_num_sender_update_threads, FLAGS_frame_duration, 0,
            FLAGS_enable_slow_start, true);
        thread_start += num_total_sender_threads;
        sender->StartTxfromMain(frame_start, frame_end);
      }
      if (cfg->Frame().NumUlDataSyms() > 0) {
        if ((FLAGS_fwd_udp_port != 0) && (!FLAGS_fwd_udp_address.empty())) {
          receiver = std::make_unique<MacReceiver>(
              cfg.get(), cfg->MacDataBytesNumPerframe(Direction::kDownlink),
              cfg->BsServerAddr(), cfg->BsMacTxPort(), FLAGS_fwd_udp_address,
              FLAGS_fwd_udp_port, FLAGS_num_receiver_threads, thread_start);
        } else {
          receiver = std::make_unique<MacReceiver>(
              cfg.get(), cfg->MacDataBytesNumPerframe(Direction::kDownlink),
              cfg->BsServerAddr(), cfg->BsMacTxPort(),
              FLAGS_num_receiver_threads, thread_start);
        }
        rx_threads = receiver->StartRecv();
      }

      std::printf("Running mac basestation application\n");
      while ((cfg->Running() == true) &&
             (SignalHandler::GotExitSignal() == false)) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }

      /* May want to move this exit section to after the exception */
      cfg->Running(false);
      std::printf("Terminating mac basestation application\n");
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
  std::printf("Mac basestation application terminated!\n");
  PrintCoreAssignmentSummary();
  gflags::ShutDownCommandLineFlags();
  AGORA_LOG_SHUTDOWN();
  return ret;
}
