#include "dpdk_transport.hpp"
#include <netinet/ether.h>
#include <arpa/inet.h>
#include <stdexcept>
#include "gettime.h"
#include <sys/time.h>
#include "config.hpp"
#include <iostream>
#include "utils.h"

int main(int argc, char **argv) 
{
    int opt;
    std::string conf_file = TOSTRING(PROJECT_DIRECTORY) "/config/run.json";
    size_t remote_server_idx = 0;
    while ((opt = getopt(argc, argv, "c:r:")) != -1) {
        switch (opt) {
            case 'c':
                conf_file = optarg;
                break;
            case 'r':
                remote_server_idx = atoi(optarg);
                break;
            default:
                std::cerr << "Usage: " << argv[0] << " [-c conf_file]" << 
                    " [-r remote_server_idx]" << std::endl;
                exit(EXIT_FAILURE);
        }
    }

    Config* cfg = new Config(conf_file.c_str());

    rte_mempool *mbuf_pool;
    DpdkTransport::dpdk_init(0, 1, cfg->pci_addr);
    mbuf_pool = DpdkTransport::create_mempool();
    if (DpdkTransport::nic_init(0, mbuf_pool, 1, 1) != 0) {
        printf("NIC init error!\n");
        exit(1);
    }

    rte_ether_addr src_mac_addr, dst_mac_addr;
    uint32_t src_ip, dst_ip;
    uint16_t src_port, dst_port;

    int ret = rte_eth_macaddr_get(0, &src_mac_addr);
    rt_assert(ret == 0, "Cannot get MAC address of the port");

    ether_addr* parsed_mac = ether_aton(cfg->bs_server_mac_list[remote_server_idx].c_str());
    rt_assert(parsed_mac != NULL, "Invalid server mac address");
    memcpy(&dst_mac_addr, parsed_mac, sizeof(ether_addr));

    ret = inet_pton(AF_INET, cfg->bs_server_addr_list[cfg->bs_server_addr_idx].c_str(), &src_ip);
    rt_assert(ret == 1, "Invalid src IP address");
    ret = inet_pton(AF_INET, cfg->bs_server_addr_list[remote_server_idx].c_str(), &dst_ip);
    rt_assert(ret == 1, "Invalid dst IP address");

    src_port = dst_port = 12000;

    rte_mbuf *mbuf = DpdkTransport::alloc_udp(mbuf_pool, src_mac_addr,
        dst_mac_addr, src_ip, dst_ip, src_port, dst_port, 16);
    auto* eth_hdr = rte_pktmbuf_mtod(mbuf, rte_ether_hdr*);
    auto* payload = reinterpret_cast<char*>(eth_hdr) + kPayloadOffset;
    *((uint32_t*)payload) = 4244;
    
    struct timeval start_time;
    gettimeofday(&start_time, NULL);
    rte_eth_tx_burst(0, 0, &mbuf, 1);

    rte_mbuf *mbufs[32];
    
    while (true) {
        int recv = rte_eth_rx_burst(0, 0, mbufs, 32);
        if (recv > 0) {
            for (int i = 0; i < recv; i ++) {
                auto* eth_hdr = rte_pktmbuf_mtod(mbufs[i], rte_ether_hdr*);
                auto* payload = reinterpret_cast<char*>(eth_hdr) + kPayloadOffset;
                if (*((uint32_t*)payload) == 4244) {
                    struct timeval end_time;
                    gettimeofday(&end_time, NULL);
                    uint64_t start_tc = start_time.tv_sec * 1000000000L + start_time.tv_usec;
                    uint64_t end_tc = end_time.tv_sec * 1000000000L + end_time.tv_usec;
                    int64_t latency = (int64_t)end_tc - (int64_t)start_tc;
                    int64_t middle = *((int64_t*)(payload + 4)) - (int64_t)start_tc;
                    printf("Received timer %ld %ld %ld\n", latency, middle, middle-latency/2);
                    return 0;
                }
            }
        }
    }

    return 0;
}