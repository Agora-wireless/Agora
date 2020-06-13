/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "simple_mac_bs.hpp"

int main(int argc, char* argv[])
{
    std::string confFile;
    int core_offset;
    int thread_num;
    if (argc == 4) {
        thread_num = strtol(argv[1], NULL, 10);
        core_offset = strtol(argv[2], NULL, 10);
        confFile = std::string("/") + std::string(argv[3]);
    } else {
        confFile = "/data/tddconfig-sim-dl.json";
        thread_num = 1;
        core_offset = 22;
        printf(
            "Wrong arguments (requires 2 arguments: 1. number of threads, 2. "
            "core offset, 3. config file)\n");
        printf("Arguments set to default: 22, %s\n", confFile.c_str());
    }

    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + confFile;
    auto* cfg = new Config(filename.c_str());
    cfg->genData();
    int ret;
    try {
        SignalHandler signalHandler;

        // Register signal handler to handle kill signal
        signalHandler.setupSignalHandlers();
        auto* receiver_ = new SimpleBSMac(cfg, thread_num, core_offset);
        receiver_->startRecv();
        ret = EXIT_SUCCESS;
    } catch (SignalException& e) {
        std::cerr << "SignalException: " << e.what() << std::endl;
        ret = EXIT_FAILURE;
    }
    delete cfg;

    return ret;
}
