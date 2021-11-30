#include "dpdk_transport.hpp"
#include <netinet/ether.h>
#include <arpa/inet.h>
#include <stdexcept>
#include "gettime.h"
#include <sys/time.h>

static inline void rt_assert(bool condition, const char* throw_str) {
  if (!condition) throw std::runtime_error(throw_str);
}

int main() 
{
    rte_mempool *mbuf_pool;
    DpdkTransport::dpdk_init(0, 1, "37:00.1");
    mbuf_pool = DpdkTransport::create_mempool();
    if (DpdkTransport::nic_init(0, mbuf_pool, 1, 1) != 0) {
        printf("NIC init error!\n");
        exit(1);
    }

    rte_ether_addr src_mac_addr, dst_mac_addr;
    uint32_t src_ip, dst_ip;

    int ret = rte_eth_macaddr_get(0, &src_mac_addr);
    rt_assert(ret == 0, "Cannot get MAC address of the port");

    ether_addr* parsed_mac = ether_aton("0c:42:a1:50:c7:ee");
    rt_assert(parsed_mac != NULL, "Invalid server mac address");
    memcpy(&dst_mac_addr, parsed_mac, sizeof(ether_addr));

    ret = inet_pton(AF_INET, "192.168.21.181", &src_ip);
    rt_assert(ret == 1, "Invalid src IP address");
    ret = inet_pton(AF_INET, "192.168.21.182", &dst_ip);
    rt_assert(ret == 1, "Invalid dst IP address");

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