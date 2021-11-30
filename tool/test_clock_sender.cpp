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
    DpdkTransport::dpdk_init(0, 1);
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

    ether_addr* parsed_mac = ether_aton("0c:42:a1:50:c8:06");
    rt_assert(parsed_mac != NULL, "Invalid server mac address");
    memcpy(&dst_mac_addr, parsed_mac, sizeof(ether_addr));

    ret = inet_pton(AF_INET, "192.168.21.181", &src_ip);
    rt_assert(ret == 1, "Invalid src IP address");
    ret = inet_pton(AF_INET, "192.168.21.184", &dst_ip);
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
                    printf("Received timer %ld %ld %ld!\n", latency, middle, middle-latency/2);
                    return 0;
                }
            }
        }
    }

    return 0;
}