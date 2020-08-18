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
    if (argc == 2) {
        confFile = std::string(argv[1]);
    } else {
        confFile = TOSTRING(PROJECT_DIRECTORY)
            + std::string("/data/tddconfig-sim-ul.json");
    }
    auto* cfg = new Config(confFile.c_str());
    cfg->genData();

    constexpr int kReqType = 2;
    constexpr int kUDPPort = 31850 + 1;

    auto* task_threads = new std::thread*[cfg->remote_ldpc_num_threads];

    std::string uri = cfg->remote_ldpc_addr + ":" + std::to_string(kUDPPort);
    auto* nexus = new erpc::Nexus(uri, 0, 0);
    nexus->register_req_func(kReqType, ldpc_req_handler);

    for (size_t i = 0; i < cfg->remote_ldpc_num_threads; i++) {
        task_threads[i] = new std::thread(run_remote, cfg, i, nexus);
    }

    for (size_t i = 0; i < cfg->remote_ldpc_num_threads; i++) {
        task_threads[i]->join();
    }

    return 0;
}