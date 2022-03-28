#include "dpdk_transport.hpp"
#include <netinet/ether.h>
#include <arpa/inet.h>
#include <stdexcept>
#include "gettime.h"
#include <sys/time.h>
#include "config.hpp"
#include <iostream>
#include "utils.h"

static inline void rt_assert(bool condition, const char* throw_str) {
  if (!condition) throw std::runtime_error(throw_str);
}

int main(int argc, char **argv) 
{
    int opt;
    std::string conf_file = TOSTRING(PROJECT_DIRECTORY) "/config/run.json";
    size_t remote_server_idx = 0;
    while ((opt = getopt(argc, argv, "c:")) != -1) {
        switch (opt) {
            case 'c':
                conf_file = optarg;
                break;
            default:
                std::cerr << "Usage: " << argv[0] << " [-c conf_file]" << std::endl;
                exit(EXIT_FAILURE);
        }
    }

    Config* cfg = new Config(conf_file.c_str());

    rte_mempool *mbuf_pool;
    DpdkTransport::dpdk_init(0, 1, cfg->pci_addr.c_str());
    mbuf_pool = DpdkTransport::create_mempool();
    if (DpdkTransport::nic_init(0, mbuf_pool, 1, 1) != 0) {
        printf("NIC init error!\n");
        exit(1);
    }

    rte_mbuf *mbuf[32];
    
    while (true) {
        int recv = rte_eth_rx_burst(0, 0, mbuf, 32);
        if (recv > 0) {
            for (int i = 0; i < recv; i ++) {
                auto* eth_hdr = rte_pktmbuf_mtod(mbuf[i], rte_ether_hdr*);
                auto* payload = reinterpret_cast<char*>(eth_hdr) + kPayloadOffset;
                if (*((uint32_t*)payload) == 4244) {
                    rte_ether_addr tmp = eth_hdr->s_addr;
                    eth_hdr->s_addr = eth_hdr->d_addr;
                    eth_hdr->d_addr = tmp;
                    struct timeval current_time;
                    gettimeofday(&current_time, NULL);
                    *((uint64_t*)(payload + 4)) = current_time.tv_sec * 1000000000L + current_time.tv_usec;
                    rte_eth_tx_burst(0, 0, mbuf + i, 1);
                    return 0;
                }
            }
        }
    }

    return 0;
}