/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */

#include "millipede.hpp"

int main(int argc, char* argv[])
{
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string confFile = cur_directory + "/data/tddconfig-sim-ul.json";
    if (argc == 2)
        confFile = std::string(argv[1]);
    auto* cfg = new Config(confFile.c_str());
    cfg->genData();
    Millipede* millipede_cli;

    int ret;
    try {
        SignalHandler signalHandler;

        // Register signal handler to handle kill signal
        signalHandler.setupSignalHandlers();
        millipede_cli = new Millipede(cfg);
        millipede_cli->flags.enable_save_decode_data_to_file = true;
        millipede_cli->start();
        ret = EXIT_SUCCESS;
    } catch (SignalException& e) {
        std::cerr << "SignalException: " << e.what() << std::endl;
        ret = EXIT_FAILURE;
    }
    delete cfg;

    return ret;
}
