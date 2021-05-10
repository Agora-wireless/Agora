/**
 * @file main.cc
 * @brief Main file for the agora server
 */
#include "agora.h"
#include "config.h"
#include "recorder.h"
#include "signal_handler.h"

int main(int argc, char* argv[]) {
  std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  std::string conf_file = cur_directory + "/data/tddconfig-sim-ul.json";
  if (argc == 2) {
    conf_file = std::string(argv[1]);
  }

  std::unique_ptr<Config> cfg = std::make_unique<Config>(conf_file.c_str());
  cfg->GenData();

  int ret;
  try {
    SignalHandler signal_handler;

    // Register signal handler to handle kill signal
    signal_handler.SetupSignalHandlers();

    auto data_recoder = std::make_unique<Agora_recorder::Recorder>(cfg.get());

    std::unique_ptr<Agora> agora_cli = std::make_unique<Agora>(cfg.get());
    agora_cli->Start();
    ret = EXIT_SUCCESS;
  } catch (SignalException& e) {
    std::cerr << "SignalException: " << e.what() << std::endl;
    ret = EXIT_FAILURE;
  }
  return ret;
}
