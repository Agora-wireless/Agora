/**
 * @file user-main.cc
 * @brief Command line executable for the user code
 */
#include <string>

#include "config.h"
#include "gflags/gflags.h"
#include "logger.h"
#include "phy-ue.h"
#include "signal_handler.h"
#include "version_config.h"

DEFINE_string(conf_file,
              TOSTRING(PROJECT_DIRECTORY) "/files/config/ci/chsim.json",
              "Config filename");

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage("conf_file : set the configuration filename");
  gflags::SetVersionString(GetAgoraProjectVersion());
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  AGORA_LOG_INIT();

  std::string filename;
  // For backwards compatibility
  if (argc == 2) {
    filename = argv[1];
    std::printf("User: Setting configuration filename to %s\n",
                filename.c_str());
  } else {
    filename = FLAGS_conf_file;
  }

  auto config = std::make_unique<Config>(filename.c_str());
  config->GenData();
  int ret;
  try {
    SignalHandler signal_handler;

    // Register signal handler to handle kill signal
    signal_handler.SetupSignalHandlers();
    auto phy = std::make_unique<PhyUe>(config.get());
    phy->Start();
    ret = EXIT_SUCCESS;
  } catch (SignalException& e) {
    std::cerr << "SignalException: " << e.what() << std::endl;
    ret = EXIT_FAILURE;
  }

  PrintCoreAssignmentSummary();
  gflags::ShutDownCommandLineFlags();
  AGORA_LOG_SHUTDOWN();

  return ret;
}
