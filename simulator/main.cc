#include "simulator.h"

int main(int argc, char const* argv[])
{
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string conf_file;
    int thread_num;
    int core_offset;
    int delay;
    if (argc == 5) {
        thread_num = strtol(argv[1], NULL, 10);
        core_offset = strtol(argv[2], NULL, 10);
        delay = strtol(argv[3], NULL, 10);
        conf_file = std::string(argv[4]);
    } else {
        conf_file = cur_directory + "/data/tddconfig-sim-dl.json";
        thread_num = 4;
        core_offset = 22;
        delay = 5000;
        std::printf("Wrong arguments (requires 4 arguments: 1. number of tx "
                    "threads, 2. "
                    "core offset, 3. frame duration, 4. config file)\n");
        std::printf(
            "Arguments set to default: 4, 22, 5000, %s\n", conf_file.c_str());
    }
    auto* cfg = new Config(conf_file.c_str());
    cfg->GenData();
    Simulator* simulator;
    int ret;
    try {
        SignalHandler signal_handler;

        // Register signal handler to handle kill signal
        signal_handler.SetupSignalHandlers();
        simulator = new Simulator(cfg, thread_num, core_offset, delay);
        simulator->Start();
        ret = EXIT_SUCCESS;
    } catch (SignalException& e) {
        std::cerr << "SignalException: " << e.what() << std::endl;
        ret = EXIT_FAILURE;
    }

    return ret;
}
