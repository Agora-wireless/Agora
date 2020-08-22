#include "channel_sim.hpp"
#include "config.hpp"

int main(int argc, char const* argv[])
{
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string ue_conf_file;
    std::string bs_conf_file;
    size_t bs_thread_num, ue_thread_num, bs_socket_num, ue_socket_num,
        worker_thread_num, core_offset;
    // argv[1] bs_socket_num
    // argv[2] user_socket_num
    // argv[3] bs_thread_num for rx
    // argv[4] user_thread_num for rx
    // argv[5] core_offset
    if (argc == 9) {
        bs_socket_num = strtoul(argv[1], NULL, 10);
        ue_socket_num = strtoul(argv[2], NULL, 10);
        bs_thread_num = strtoul(argv[3], NULL, 10);
        ue_thread_num = strtoul(argv[4], NULL, 10);
        worker_thread_num = strtoul(argv[5], NULL, 10);
        core_offset = strtoul(argv[6], NULL, 10);
        bs_conf_file = std::string(argv[7]);
        ue_conf_file = std::string(argv[8]);
    } else {
        bs_conf_file = cur_directory + "/data/tddconfig-sim-bs.json";
        ue_conf_file = cur_directory + "/data/tddconfig-sim-ue.json";
        ue_thread_num = 1;
        bs_thread_num = 1;
        worker_thread_num = 1;
        ue_socket_num = 1;
        bs_socket_num = 1;
        core_offset = 0;
        printf("Wrong arguments (requires 7 arguments: 1. number of tx "
               "threads, 2. "
               "core offset, 3. frame duration, 4. config file)\n");
        printf("Arguments set to default: 4, 22, 5000, %s\n",
            bs_conf_file.c_str());
    }
    Config* bs_config = new Config(bs_conf_file);
    Config* ue_config = new Config(ue_conf_file);
    ChannelSim* sim
        = new ChannelSim(bs_config, ue_config, bs_socket_num, ue_socket_num,
            bs_thread_num, ue_thread_num, worker_thread_num, core_offset);
    sim->start();
    return 0;
}
