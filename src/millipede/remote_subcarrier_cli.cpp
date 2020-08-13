/**
 * Author: Kevin Boos
 * Email: kevinaboos@gmail.com
 *
 * This file is the main entry point for a remote subcarrier process
 * and simply parses the command-line arguments to create a new
 * RemoteSubcarrier instance. 
 * 
 * See `RemoteSubcarrier` for all the interesting details. 
 */

#include "remote_subcarrier.hpp"
#include "signalHandler.hpp"
#include <gflags/gflags.h>

DEFINE_uint32(sc_endpoint_index, 0, "The index into the "
    "\"subcarrier_endpoints\" array in the JSON \"conf_file\" "
    "that this remote subcarrier instance represents.");
DEFINE_string(conf_file,
    TOSTRING(PROJECT_DIRECTORY) "/data/config-remote-subcarrier-sim-ul.json",
    "Path to configuration file");


int main(int argc, char* argv[])
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    auto* cfg = new Config(FLAGS_conf_file);
    cfg->genData();

    int ret;
    try {
        SignalHandler signalHandler;
        // Register signal handler to handle kill signal
        signalHandler.setupSignalHandlers();

        // Create the RemoteWorker instance that represents this process
        RemoteSubcarrier rsc(cfg, FLAGS_sc_endpoint_index);

        sleep(5);

        std::cout << "[RemoteSubcarrierCLI]: exiting loop." << std::endl;

        ret = EXIT_SUCCESS;
    } catch (SignalException& e) {
        std::cerr << "SignalException: " << e.what() << std::endl;
        ret = EXIT_FAILURE;
    }

    // Cleanup and exit process.
    std::cout << std::endl;
    std::cout << "[RemoteSubcarrier] main(): broke out of main loop."
              << std::endl;
    if (SignalHandler::gotExitSignal()) {
        std::cout << "[RemoteSubcarrier] main(): received exit signal."
                  << std::endl;
    }
    delete cfg;
    return ret;
}