/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "simulator.hpp"

int main(int argc, char const *argv[]) {
  std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  std::string filename = cur_directory + "/data/tddconfig.json";
  Config *cfg = new Config(filename.c_str());
  Simulator *simulator;
  int ret;
  try {
    SignalHandler signalHandler;

    // Register signal handler to handle kill signal
    signalHandler.setupSignalHandlers();
    if (argc == 5) {
      simulator = new Simulator(
          cfg, strtol(argv[1], NULL, 10), strtol(argv[2], NULL, 10),
          strtol(argv[3], NULL, 10), strtol(argv[4], NULL, 10));
    } else {
      printf(
          "Wrong arguments (requires 3 arguments: 1. number of task threads, "
          "2. number of tx threads, 3. core offset, 4. frame duration)\n");
      printf("Arguments set to default: 10, 4, 21, 5000\n");
      simulator = new Simulator(cfg, 10, 4, 21, 5000);
    }
    simulator->start();
    ret = EXIT_SUCCESS;
  } catch (SignalException &e) {
    std::cerr << "SignalException: " << e.what() << std::endl;
    ret = EXIT_FAILURE;
  }

  return ret;
}
