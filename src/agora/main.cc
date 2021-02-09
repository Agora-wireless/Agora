#include "agora.h"

int main(int argc, char* argv[])
{
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string confFile = cur_directory + "/data/tddconfig-sim-ul.json";
    if (argc == 2)
        confFile = std::string(argv[1]);
    auto* cfg = new Config(confFile.c_str());
    cfg->genData();
    Agora* agora_cli;

    int ret;
    try {
        SignalHandler signalHandler;

        // Register signal handler to handle kill signal
        signalHandler.setupSignalHandlers();
        agora_cli = new Agora(cfg);
        agora_cli->start();
        ret = EXIT_SUCCESS;
    } catch (SignalException& e) {
        std::cerr << "SignalException: " << e.what() << std::endl;
        ret = EXIT_FAILURE;
    }
    delete cfg;

    return ret;
}
