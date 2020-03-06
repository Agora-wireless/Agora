/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "sender.hpp"

int main(int argc, char* argv[])
{
    std::string confFile;
    int thread_num, core_offset, delay;
    if (argc == 5) {
        thread_num = strtol(argv[1], NULL, 10);
        core_offset = strtol(argv[2], NULL, 10);
        delay = strtol(argv[3], NULL, 10);
        confFile = std::string("/") + std::string(argv[4]);
    } else {
        confFile = "/data/tddconfig-sim-dl.json";
        thread_num = 4;
        core_offset = 22;
        delay = 5000;
        printf("Wrong arguments (requires 4 arguments: 1. number of tx "
               "threads, 2. "
               "core offset, 3. frame duration, 4. config file)\n");
        printf("Arguments set to default: 4, 22, 5000, %s\n", confFile.c_str());
    }
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + confFile;
    Config* cfg = new Config(filename.c_str());
    Sender* sender;
    sender = new Sender(cfg, thread_num, core_offset, delay);
    printf("start sender\n");
    sender->startTX();
    return 0;
}
