/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "simple_mac_client.hpp"

int main(int argc, char* argv[])
{
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string confFile;
    int core_offset, delay;
    if (argc == 4) {
        core_offset = strtol(argv[1], NULL, 10);
        delay = strtol(argv[2], NULL, 10);
        confFile = std::string(argv[3]);
    } else {
        confFile = cur_directory + "/data/tddconfig-sim-dl.json";
        core_offset = 22;
        delay = 5000;
        printf("Wrong arguments (requires 3 arguments: 1. number of tx "
               "core offset, 2. frame duration, 3. config file)\n");
        printf("Arguments set to default: 22, 5000, %s\n", confFile.c_str());
    }

    auto* cfg = new Config(confFile.c_str());
    cfg->genData();
    auto* sender = new SimpleClientMac(cfg, core_offset, delay);

    printf("Start MAC sender\n");
    sender->startTX();

    return 0;
}
