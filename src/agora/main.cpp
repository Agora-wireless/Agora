#include "agora.hpp"
#include <gflags/gflags.h>

DEFINE_string(conf_file,
    TOSTRING(PROJECT_DIRECTORY) "/data/tddconfig-sim-ul.json",
    "Config filename");

int main(int argc, char* argv[])
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string confFile = FLAGS_conf_file;
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
