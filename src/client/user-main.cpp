#include "phy-ue.hpp"

int main(int argc, char const* argv[])
{
    std::string filename;
    if (argc > 1)
        filename = argv[1];
    else {
        std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
        filename = cur_directory + "/data/userconfig_512.json";
    }
    auto* config = new Config(filename.c_str());
    int ret;
    try {
        SignalHandler signalHandler;

        // Register signal handler to handle kill signal
        signalHandler.setupSignalHandlers();
        auto* phy = new Phy_UE(config);
        phy->start();
        ret = EXIT_SUCCESS;
    } catch (SignalException& e) {
        std::cerr << "SignalException: " << e.what() << std::endl;
        ret = EXIT_FAILURE;
    }

    return ret;
}
