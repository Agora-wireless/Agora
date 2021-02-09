#include "channel_sim.h"
#include <gflags/gflags.h>

DEFINE_uint64(
    bs_threads, 1, "Number of threads for handling reception of BS packets");
DEFINE_uint64(
    ue_threads, 1, "Number of threads for handling reception of UE packets");
DEFINE_uint64(worker_threads, 1,
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
DEFINE_double(chan_snr, 20.0, "Signal-to-Noise Ratio");

int main(int argc, char* argv[])
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    auto* bs_config = new Config(FLAGS_bs_conf_file);
    auto* ue_config = new Config(FLAGS_ue_conf_file);
    auto* sim = new ChannelSim(bs_config, ue_config, FLAGS_bs_threads,
        FLAGS_ue_threads, FLAGS_worker_threads, FLAGS_core_offset,
        FLAGS_chan_model, FLAGS_chan_snr);
    sim->Start();
    return 0;
}
