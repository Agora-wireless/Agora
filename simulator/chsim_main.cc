/**
 * @file chsim_main.cc
 * @brief Main file for the chsim executable
 */
#include <gflags/gflags.h>

#include "channel_sim.h"
#include "logger.h"
#include "version_config.h"

DEFINE_uint64(bs_threads, 1,
              "Number of threads for handling reception of BS packets");
DEFINE_uint64(ue_threads, 1,
              "Number of threads for handling reception of UE packets");
DEFINE_uint64(
    worker_threads, 1,
    "Number of worker threads handling packet transmissions to BS and UE "
    "Antennas");
DEFINE_uint64(core_offset, 0, "Core ID of the first channel_sim thread");
DEFINE_string(
    conf_file,
    TOSTRING(PROJECT_DIRECTORY) "/files/config/ci/tddconfig-sim-both.json",
    "Config filename");
DEFINE_string(chan_model, "RAYLEIGH", "Simulator Channel Type: RAYLEIGH/AWGN");
DEFINE_double(chan_snr, 20.0, "Signal-to-Noise Ratio");

int main(int argc, char* argv[]) {
  int ret = EXIT_FAILURE;
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  gflags::SetVersionString(GetAgoraProjectVersion());
  AGORA_LOG_INIT();
  std::printf("Base Station configuration\n");
  auto config = std::make_unique<Config>(FLAGS_conf_file);
  {
    try {
      SignalHandler signal_handler;

      // Register signal handler to handle kill signal
      signal_handler.SetupSignalHandlers();
      auto sim = std::make_unique<ChannelSim>(
          config.get(), FLAGS_bs_threads, FLAGS_ue_threads,
          FLAGS_worker_threads, FLAGS_core_offset, FLAGS_chan_model,
          FLAGS_chan_snr);
      sim->Run();
      ret = EXIT_SUCCESS;
    } catch (SignalException& e) {
      std::cerr << "chsim: SignalException: " << e.what() << std::endl;
      ret = EXIT_FAILURE;
    }
  }
  std::printf("Channel Simulator Exit\n");
  PrintCoreAssignmentSummary();
  gflags::ShutDownCommandLineFlags();
  AGORA_LOG_SHUTDOWN();
  return ret;
}
