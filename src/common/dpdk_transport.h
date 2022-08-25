/**
 * @file dpdk_transport.h
 * @brief Declaration file for the DpdkTransport class.
 */

#ifndef DPDK_TRANSPORT_H_
#define DPDK_TRANSPORT_H_

#include <netinet/ether.h>
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

#include <cinttypes>
#include <string>

// #include "eth_common.h"
#include "utils.h"

static constexpr size_t kRxRingSize = 2048;
static constexpr size_t kTxRingSize = 2048;
static constexpr size_t kNumMBufs = ((64 * 1024) - 1);
static constexpr size_t kMBufCacheSize = 128;
static constexpr size_t kMacAddrBtyes = 17;

// allow max jumbo frame 9.5 KB
static constexpr size_t kJumboFrameMaxSize = 0x2600;
/// Maximum number of packets received in rx_burst
static constexpr size_t kRxBatchSize = 16;
static constexpr size_t kTxBatchSize = 1;

/// Offset to the payload starting from the beginning of the UDP frame
//static constexpr size_t kPayloadOffset =
//    sizeof(rte_ether_hdr) + sizeof(rte_ipv4_hdr) + sizeof(rte_udp_hdr) + 22;
//static_assert(kPayloadOffset == 64, "kPayloadOffset must equal 64");

static constexpr size_t kPayloadOffset =
    sizeof(rte_ether_hdr) + sizeof(rte_ipv4_hdr) + sizeof(rte_udp_hdr);

class DpdkTransport {
 public:
  DpdkTransport();
  ~DpdkTransport();

  static std::vector<uint16_t> GetPortIDFromMacAddr(
      size_t port_num, const std::string& mac_addrs);

  static int NicInit(uint16_t port, struct rte_mempool* mbuf_pool,
                     int thread_num, size_t pkt_len = kJumboFrameMaxSize);

  // Steer the flow [src_ip, dest_ip, src_port, dst_port] arriving on
  // [port_id] to RX queue [rx_q]
  static void InstallFlowRule(uint16_t port_id, uint16_t rx_q, uint32_t src_ip,
                              uint32_t dest_ip, uint16_t src_port,
                              uint16_t dst_port);

  static void InstallFlowRuleDropAll(uint16_t port_id);

  static void FastMemcpy(void* pvDest, void* pvSrc, size_t nBytes);
  static void PrintPkt(rte_be32_t src_ip, rte_be32_t dst_ip,
                       rte_be16_t src_port, rte_be16_t dst_port, size_t len,
                       size_t tid);

  /// Return a string representation of this packet
  static std::string PktToString(const rte_mbuf* pkt);

  /// Allocate and return a fresh rte_mbuf with Ethernet, IPv4, and UDP
  /// header filled
  static rte_mbuf* AllocUdp(rte_mempool* mbuf_pool, rte_ether_addr src_mac_addr,
                            rte_ether_addr dst_mac_addr, uint32_t src_ip_addr,
                            uint32_t dst_ip_addr, uint16_t src_udp_port,
                            uint16_t dst_udp_port, size_t buffer_length,
                            uint16_t pkt_id);

  /// Init dpdk on core [core_offset:core_offset+thread_num]
  static void DpdkInit(uint16_t core_offset, size_t thread_num);
  static rte_mempool* CreateMempool(size_t num_ports,
                                    size_t packet_length = kJumboFrameMaxSize);
};

#endif  // DPDK_TRANSPORT_H_