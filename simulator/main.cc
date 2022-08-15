/**
 * @file main.cc
 * @brief Main file for the simulator executable
 */
#include "simulator.h"

int main(int argc, char const* argv[]) {
  const std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  std::string conf_file;
  int thread_num;
  int core_offset;
  int delay;
  if (argc == 5) {
    thread_num = strtol(argv[1], nullptr, 10);
    core_offset = strtol(argv[2], nullptr, 10);
    delay = strtol(argv[3], nullptr, 10);
    conf_file = std::string(argv[4]);
  } else {
    conf_file = cur_directory + "/files/config/ci/tddconfig-sim-dl.json";
    thread_num = 4;
    core_offset = 22;
    delay = 5000;
    std::printf(
        "Wrong arguments (requires 4 arguments: 1. number of tx "
        "threads, 2. "
        "core offset, 3. frame duration, 4. config file)\n");
    std::printf("Arguments set to default: 4, 22, 5000, %s\n",
                conf_file.c_str());
  }
  auto cfg = std::make_unique<Config>(conf_file.c_str());
  cfg->GenData();
  int ret;
  try {
    SignalHandler signal_handler;

    // Register signal handler to handle kill signal
    signal_handler.SetupSignalHandlers();
    auto simulator =
        std::make_unique<Simulator>(cfg.get(), thread_num, core_offset, delay);
    simulator->Start();
    ret = EXIT_SUCCESS;
  } catch (SignalException& e) {
    std::cerr << "SignalException: " << e.what() << std::endl;
    ret = EXIT_FAILURE;
  }

  return ret;
}
