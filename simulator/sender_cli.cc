/**
 * @file sender_cli.cc
 * @brief Main file for the sender executable
 */
#include <gflags/gflags.h>

#include "sender.h"
#include "signal_handler.h"

DEFINE_uint64(num_threads, 4, "Number of sender threads");
DEFINE_uint64(core_offset, 0, "Core ID of the first sender thread");
DEFINE_uint64(frame_duration, 5000, "Frame duration in microseconds");
DEFINE_uint64(inter_frame_delay, 0, "Delay between two frames in microseconds");
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

  int ret = EXIT_SUCCESS;
  {
    auto cfg = std::make_unique<Config>(filename.c_str());

    try {
      cfg->GenData();
      {
        auto sender = std::make_unique<Sender>(
            cfg.get(), FLAGS_num_threads, FLAGS_core_offset, FLAGS_frame_duration,
            FLAGS_inter_frame_delay, FLAGS_enable_slow_start,
            FLAGS_server_mac_addr);
        sender->StartTx();
      }  // end context sender
    } catch (SignalException &e) {
      std::cerr << "SignalException: " << e.what() << std::endl;
      ret = EXIT_FAILURE;
    } catch (std::runtime_error &e) {
      std::cerr << "RuntimeErrorException: " << e.what() << std::endl;
      ret = EXIT_FAILURE;
    } catch (std::invalid_argument &e) {
      std::cerr << "InvalidArgumentException: " << e.what() << std::endl;
      ret = EXIT_FAILURE;
    }
  }    // end context Config

  return ret;
}
