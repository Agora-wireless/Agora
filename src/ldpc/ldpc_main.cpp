#include "remote_ldpc.hpp"
#include "utils.h"

#include <thread>

void run_remote(Config* cfg, size_t tid, erpc::Nexus* nexus)
{
    pin_to_core_with_offset(
        ThreadType::kWorker, cfg->remote_ldpc_core_offset, tid);

    auto* remote = new RemoteLDPC(cfg->LDPC_config, tid, nexus);
    remote->run_erpc_event_loop_forever();
}

int main(int argc, char** argv)
{
    std::string confFile;
    if (argc == 2)
        confFile = std::string("/") + std::string(argv[1]);
    else
        confFile = "/data/tddconfig-sim-ul.json";
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + confFile;
    auto* cfg = new Config(filename.c_str());
    cfg->genData();

    constexpr int kReqType = 2;
    constexpr int kUDPPort = 31850;

    auto* task_threads = new std::thread*[cfg->remote_ldpc_num];

    std::string uri = cfg->remote_ldpc_addr + ":" + std::to_string(kUDPPort);
    auto* nexus = new erpc::Nexus(uri, 0, 0);
    nexus->register_req_func(kReqType, ldpc_req_handler);

    for (size_t i = 0; i < cfg->remote_ldpc_num; i++) {
        task_threads[i] = new std::thread(run_remote, cfg, i, nexus);
    }

    for (size_t i = 0; i < cfg->remote_ldpc_num; i++) {
        task_threads[i]->join();
    }

    return 0;
}