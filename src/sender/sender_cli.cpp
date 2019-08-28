/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */
#include "packageSender.hpp"

int main(int argc, char const *argv[])
{
	std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + "/data/tddconfig.json";
    Config *cfg = new Config(filename.c_str());
    if(argc > 3)
        PackageSender sender(cfg, strtol(argv[1], NULL, 10), strtol(argv[2], NULL, 10), strtol(argv[3], NULL, 10), strtol(argv[4], NULL, 10));
    else
        PackageSender sender(cfg, strtol(argv[1], NULL, 10), strtol(argv[2], NULL, 10));
    printf("send package\n");
    return 0;
}
