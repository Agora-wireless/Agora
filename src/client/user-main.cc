#include "phy-ue.h"

int main(int argc, char const* argv[])
{
    std::string filename;
    if (argc > 1) {
        filename = argv[1];
    } else {
        std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
        filename = cur_directory + "/data/userconfig_512.json";
    }
    auto* config = new Config(filename.c_str());
    config->GenData();
    int ret;
    try {
        SignalHandler signal_handler;

        // Register signal handler to handle kill signal
        signal_handler.SetupSignalHandlers();
        auto* phy = new PhyUe(config);
        phy->Start();
        ret = EXIT_SUCCESS;
    } catch (SignalException& e) {
        std::cerr << "SignalException: " << e.what() << std::endl;
        ret = EXIT_FAILURE;
    }

    return ret;
}
