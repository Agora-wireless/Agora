/**
 * @file chsim_main.cc
 * @brief Main file for the chsim executable
 */
#include <gflags/gflags.h>

#include "channel_sim.h"

static constexpr double kNoSelectChanSnr = 1000.0f;

DEFINE_uint64(bs_threads, 1,
              "Number of threads for handling reception of BS packets");
DEFINE_uint64(ue_threads, 1,
              "Number of threads for handling reception of UE packets");
DEFINE_uint64(
    worker_threads, 1,
    "Number of worker threads handling packet transmissions to BS and UE "
    "Antennas");
DEFINE_uint64(core_offset, 0, "Core ID of the first channel_sim thread");
DEFINE_string(bs_conf_file,
              TOSTRING(PROJECT_DIRECTORY) "/data/tddconfig-sim-bs.json",
              "BS Config filename");
DEFINE_string(ue_conf_file,
              TOSTRING(PROJECT_DIRECTORY) "/data/tddconfig-sim-ue.json",
              "UE Config filename");
DEFINE_string(chan_model, "RAYLEIGH", "Simulator Channel Type: RAYLEIGH/AWGN");
DEFINE_double(chan_snr, kNoSelectChanSnr, "Signal-to-Noise Ratio dB");

int main(int argc, char* argv[]) {
  int ret = EXIT_FAILURE;
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::printf("Base Station configuration\n");
  auto bs_config = std::make_unique<Config>(FLAGS_bs_conf_file);
  std::printf("User configuration\n");
  auto ue_config = std::make_unique<Config>(FLAGS_ue_conf_file);
  {
    try {
      SignalHandler signal_handler;

      // Register signal handler to handle kill signal
      signal_handler.SetupSignalHandlers();
      double snr_db = FLAGS_chan_snr;
      // TODO, setup channel Noise (to and from)
      if (FLAGS_chan_snr == kNoSelectChanSnr) {
        snr_db = bs_config->NoiseLevel() * 1000.0f;
      }
      std::printf("chsim: noise value: %f\n", snr_db);

      auto sim = std::make_unique<ChannelSim>(
          bs_config.get(), ue_config.get(), FLAGS_bs_threads, FLAGS_ue_threads,
          FLAGS_worker_threads, FLAGS_core_offset, FLAGS_chan_model, snr_db);
      sim->Start();
      ret = EXIT_SUCCESS;
    } catch (SignalException& e) {
      std::cerr << "chsim: SignalException: " << e.what() << std::endl;
      ret = EXIT_FAILURE;
    }
  }
  std::printf("Channel Simulator Exit\n");
  return ret;
}
