#include "simulator.hpp"

int main(int argc, char const* argv[])
{
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string confFile;
    int thread_num, core_offset, delay;
    if (argc == 5) {
        thread_num = strtol(argv[1], NULL, 10);
        core_offset = strtol(argv[2], NULL, 10);
        delay = strtol(argv[3], NULL, 10);
        confFile = std::string(argv[4]);
    } else {
        confFile = cur_directory + "/data/tddconfig-sim-dl.json";
        thread_num = 4;
        core_offset = 22;
        delay = 5000;
        printf("Wrong arguments (requires 4 arguments: 1. number of tx "
               "threads, 2. "
               "core offset, 3. frame duration, 4. config file)\n");
        printf("Arguments set to default: 4, 22, 5000, %s\n", confFile.c_str());
    }
    auto* cfg = new Config(confFile.c_str());
    cfg->genData();
    Simulator* simulator;
    int ret;
    try {
        // SignalHandler signalHandler;

        // Register signal handler to handle kill signal
        // signalHandler.setupSignalHandlers();
        simulator = new Simulator(cfg, thread_num, core_offset, delay);
        simulator->start();
        ret = EXIT_SUCCESS;
    } catch (SignalException& e) {
        std::cerr << "SignalException: " << e.what() << std::endl;
        ret = EXIT_FAILURE;
    }

    return ret;
}
