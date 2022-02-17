#include "agora.hpp"
// #include <gflags/gflags.h>

// DEFINE_string(conf_file,
//     TOSTRING(PROJECT_DIRECTORY) "/data/tddconfig-sim-ul.json",
//     "Config filename");

int main(int argc, char* argv[])
{
    int opt;
    std::string conf_file = TOSTRING(PROJECT_DIRECTORY) "/data/tddconfig-sim-ul.json";
    while ((opt = getopt(argc, argv, "c:")) != -1) {
        switch (opt) {
            case 'c':
                conf_file = optarg;
                break;
            default:
                std::cerr << "Usage: " << argv[0] << " [-c conf_file]" << std::endl;
                exit(EXIT_FAILURE);
        }
    }

    // gflags::ParseCommandLineFlags(&argc, &argv, true);
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string confFile = conf_file;
    auto* cfg = new Config(confFile.c_str());
    cfg->genData();
    Agora* agora_cli;

    int ret;
    try {
        SignalHandler signalHandler;

        // Register signal handler to handle kill signal
        signalHandler.setupSignalHandlers();
        agora_cli = new Agora(cfg);
        agora_cli->Start();
        ret = EXIT_SUCCESS;
    } catch (SignalException& e) {
        std::cerr << "SignalException: " << e.what() << std::endl;
        ret = EXIT_FAILURE;
    }

    if (cfg->error) {
        exit(1);
    }

    delete cfg;

    return ret;
}
