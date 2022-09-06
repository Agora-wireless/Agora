/**
 * @file data_generator_main.cc
 * @brief Data generator to generate binary files as inputs to Agora, sender
 * and correctness tests
 */
#include <gflags/gflags.h>

#include <cstddef>
#include <memory>
#include <string>

#include "config.h"
#include "data_generator.h"
#include "logger.h"
#include "version_config.h"

static constexpr bool kVerbose = false;
static constexpr bool kPrintUplinkInformationBytes = false;
static constexpr bool kPrintDownlinkInformationBytes = false;

DEFINE_string(profile, "random",
              "The profile of the input user bytes (e.g., 'random', '123')");
DEFINE_string(
    conf_file,
    TOSTRING(PROJECT_DIRECTORY) "/files/examples/ci/tddconfig-sim-both.json",
    "Agora config filename");

int main(int argc, char* argv[]) {
  const std::string output_directory =
      TOSTRING(PROJECT_DIRECTORY) "/files/experiment/";
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  gflags::SetVersionString(GetAgoraProjectVersion());
  AGORA_LOG_INIT();
  auto cfg = std::make_unique<Config>(FLAGS_conf_file.c_str());

  const DataGenerator::Profile profile =
      FLAGS_profile == "123" ? DataGenerator::Profile::kProfile123
                             : DataGenerator::Profile::kRandom;
  std::unique_ptr<DataGenerator> data_generator =
      std::make_unique<DataGenerator>(cfg.get(), 0 /* RNG seed */, profile);

  AGORA_LOG_INFO(
      "DataGenerator: Config file: %s, data profile = %s\n",
      FLAGS_conf_file.c_str(),
      profile == DataGenerator::Profile::kProfile123 ? "123" : "random");

  AGORA_LOG_INFO("DataGenerator: Using %s-orthogonal pilots\n",
                 cfg->FreqOrthogonalPilot() ? "frequency" : "time");

  AGORA_LOG_INFO("DataGenerator: Generating encoded and modulated data\n");
  data_generator->DoDataGeneration(output_directory);
  AGORA_LOG_SHUTDOWN();
  return 0;
}
