#include "agora.hpp"

int main(int argc, char* argv[]) {
  std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  std::string confFile = cur_directory + "/data/tddconfig-sim-ul.json";
  if (argc == 2) {
    confFile = std::string(argv[1]);
  }

  std::unique_ptr<Config> cfg(new Config(confFile.c_str()));
  cfg->GenData();

  int ret;
  try {
    SignalHandler signalHandler;

    // Register signal handler to handle kill signal
    signalHandler.setupSignalHandlers();
    std::unique_ptr<Agora> agora_cli(new Agora(cfg.get()));
    agora_cli->Start();
    ret = EXIT_SUCCESS;
  } catch (SignalException& e) {
    std::cerr << "SignalException: " << e.what() << std::endl;
    ret = EXIT_FAILURE;
  }
  return ret;
}
