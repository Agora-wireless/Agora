/**
 * @file user-main.cc
 * @brief Command line executable for the user code
 */
#include <string>

#include "config.h"
#include "gflags/gflags.h"
#include "phy-ue.h"
#include "signal_handler.h"

DEFINE_string(conf_file,
              TOSTRING(PROJECT_DIRECTORY) "/data/userconfig_512.json",
              "Config filename");

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage("conf_file : set the configuration filename");
  gflags::ParseCommandLineFlags(&argc, &argv, true);

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
  int ret;
  try {
    config->GenData();

    SignalHandler signal_handler;
    // Register signal handler to handle kill signal
    signal_handler.SetupSignalHandlers();
    auto phy = std::make_unique<PhyUe>(config.get());
    phy->Start();
    ret = EXIT_SUCCESS;
  } catch (SignalException& e) {
    std::cerr << "SignalException: " << e.what() << std::endl;
    ret = EXIT_FAILURE;
  } catch (std::runtime_error &e) {
    std::cerr << "RuntimeErrorException: " << e.what() << std::endl;
    ret = EXIT_FAILURE;
  } catch (std::invalid_argument &e) {
    std::cerr << "InvalidArgumentException: " << e.what() << std::endl;
    ret = EXIT_FAILURE;
  }
  gflags::ShutDownCommandLineFlags();
  return ret;
}
