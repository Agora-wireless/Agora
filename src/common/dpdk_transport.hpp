/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */

#ifndef DPDK_TRANSPORT
#define DPDK_TRANSPORT

#include <inttypes.h>
#include <rte_byteorder.h>
#include <rte_cycles.h>
#include <rte_debug.h>
#include <rte_distributor.h>
#include <rte_eal.h>
#include <rte_ethdev.h>
#include <rte_ether.h>
#include <rte_flow.h>
#include <rte_ip.h>
#include <rte_malloc.h>
#include <rte_pause.h>
#include <rte_prefetch.h>
#include <rte_udp.h>
#include <unistd.h>

#define RX_RING_SIZE 2048
#define TX_RING_SIZE 2048

#define NUM_MBUFS ((32 * 1024) - 1)
#define MBUF_CACHE_SIZE 128

#define JUMBO_FRAME_MAX_SIZE 0x2600 // allow max jumbo frame 9.5 KB
#define EMPTY_MASK 0x0
#define FULL_MASK 0xffffffff
/// Maximum number of packets received in rx_burst
static constexpr size_t kRxBatchSize = 16;
static constexpr size_t kTxBatchSize = 1;
/// Offset to the payload
static constexpr size_t kPayloadOffset = sizeof(struct rte_ether_hdr)
    + sizeof(struct rte_ipv4_hdr) + sizeof(struct rte_udp_hdr) + 22;
static_assert(kPayloadOffset == 64, "");

class DpdkTransport {
public:
    DpdkTransport();
    ~DpdkTransport();

    static int nic_init(
        uint16_t port, struct rte_mempool* mbuf_pool, int thread_num);
    static struct rte_flow* generate_ipv4_flow(uint16_t port_id, uint16_t rx_q,
        uint32_t src_ip, uint32_t src_mask, uint32_t dest_ip,
        uint32_t dest_mask, uint16_t src_port, uint16_t src_port_mask,
        uint16_t dst_port, uint16_t dst_port_mask,
        struct rte_flow_error* error);
    static void fastMemcpy(void* pvDest, void* pvSrc, size_t nBytes);
    static void print_pkt(int src_ip, int dst_ip, uint16_t src_port,
        uint16_t dst_port, int len, int tid);
    static rte_mbuf* generate_udp_header(rte_mempool* mbuf_pool,
        rte_ether_addr src_mac_addr, rte_ether_addr dst_mac_addr,
        uint32_t src_addr, uint32_t dst_addr, int src_port, int dst_port,
        size_t buffer_length);
};

#endif