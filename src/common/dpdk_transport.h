#pragma once

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

#include <string>

#include "utils.h"

#define RX_RING_SIZE 2048
#define TX_RING_SIZE 2048

#define NUM_MBUFS ((32 * 1024) - 1)
#define MBUF_CACHE_SIZE 128

#define JUMBO_FRAME_MAX_SIZE 0x2600  // allow max jumbo frame 9.5 KB
/// Maximum number of packets received in rx_burst
static constexpr size_t kRxBatchSize = 16;
static constexpr size_t kTxBatchSize = 1;

/// Offset to the payload starting from the beginning of the UDP frame
static constexpr size_t kPayloadOffset = sizeof(struct rte_ether_hdr) +
                                         sizeof(struct rte_ipv4_hdr) +
                                         sizeof(struct rte_udp_hdr) + 22;
static_assert(kPayloadOffset == 64, "");

class DpdkTransport {
 public:
  DpdkTransport();
  ~DpdkTransport();

  static int nic_init(uint16_t port, struct rte_mempool* mbuf_pool,
                      int thread_num, size_t pkt_len = JUMBO_FRAME_MAX_SIZE);

  // Steer the flow [src_ip, dest_ip, src_port, dst_port] arriving on
  // [port_id] to RX queue [rx_q]
  static void install_flow_rule(uint16_t port_id, uint16_t rx_q,
                                uint32_t src_ip, uint32_t dest_ip,
                                uint16_t src_port, uint16_t dst_port);

  static void fastMemcpy(void* pvDest, void* pvSrc, size_t nBytes);
  static void print_pkt(int src_ip, int dst_ip, uint16_t src_port,
                        uint16_t dst_port, int len, int tid);

  /// Return a string representation of this packet
  static std::string pkt_to_string(const rte_mbuf* pkt);

  /// Allocate and return a fresh rte_mbuf with Ethernet, IPv4, and UDP
  /// header filled
  static rte_mbuf* alloc_udp(rte_mempool* mbuf_pool,
                             rte_ether_addr src_mac_addr,
                             rte_ether_addr dst_mac_addr, uint32_t src_ip_addr,
                             uint32_t dst_ip_addr, uint16_t src_udp_port,
                             uint16_t dst_udp_port, size_t buffer_length);

  /// Init dpdk on core [core_offset:core_offset+thread_num]
  static void dpdk_init(uint16_t core_offset, size_t thread_num);
  static rte_mempool* create_mempool(
      size_t packet_length = JUMBO_FRAME_MAX_SIZE);
};