/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */
#include "sender.hpp"

int main(int argc, char const *argv[])
{
	std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + "/data/tddconfig.json";
    Config *cfg = new Config(filename.c_str());
    Sender *sender;
    if(argc == 4) {
        sender = new Sender(cfg, strtol(argv[1], NULL, 10), strtol(argv[2], NULL, 10), strtol(argv[3], NULL, 10));
    }
    else {
        printf("Wrong arguments (requires 3 arguments: 1. number of tx threads, 2. core offset, 3. frame duration)\n");
        printf("Arguments set to default: 4, 22, 5000\n");
        sender = new Sender(cfg, 4, 22, 5000);
    }

    printf("start sender\n");
    sender->startTX();
    return 0;
}
