#if defined(USE_DPDK)

#include "dpdk_transport.h"

#include <immintrin.h>

#include <chrono>
#include <string>

#include "eth_common.h"
#include "logger.h"
#include "message.h"
#include "rte_version.h"
#include "utils.h"

static constexpr size_t kJumboFrameSize = 9000;
///#define ETH_IN_PROMISCUOUS_MODE

std::vector<uint16_t> DpdkTransport::GetPortIDFromMacAddr(
    size_t port_num, const std::string& mac_addrs) {
  RtAssert(mac_addrs.length() == (port_num * (kMacAddrBtyes + 1) - 1),
           "Invalid length of MAC address in config");
  std::vector<uint16_t> port_ids;
  for (size_t i = 0; i < port_num; i++) {
    // Parse MAC addresses from string
    ether_addr* parsed_mac = ether_aton(
        mac_addrs.substr(i * (kMacAddrBtyes + 1), kMacAddrBtyes).c_str());
    rte_ether_addr rte_mac_addr;
    RtAssert(parsed_mac != nullptr, "Invalid server mac address");
    std::memcpy(&rte_mac_addr, parsed_mac, sizeof(ether_addr));
    // Find the port id with the given MAC address
    uint16_t port_id;
    RTE_ETH_FOREACH_DEV(port_id) {
      rte_ether_addr addr;
      rte_eth_macaddr_get(port_id, &addr);
      if (std::equal(std::begin(addr.addr_bytes), std::end(addr.addr_bytes),
                     std::begin(rte_mac_addr.addr_bytes))) {
        port_ids.push_back(port_id);
        break;
      }
    }
  }
  return port_ids;
}

int DpdkTransport::NicInit(uint16_t port, rte_mempool* mbuf_pool,
                           int thread_num, size_t pkt_len) {
  rte_eth_conf port_conf = rte_eth_conf();
  const uint16_t rx_rings = thread_num;
  const uint16_t tx_rings = thread_num;
  int retval;
  uint16_t q;
  uint16_t nb_rxd = kRxRingSize;
  uint16_t nb_txd = kTxRingSize;

  rte_eth_dev_info dev_info;
  rte_eth_rxconf rxconf;
  rte_eth_txconf txconf;

  if (rte_eth_dev_count_avail() < port) {
    rte_exit(EXIT_FAILURE, "Not Enough NICs\n");
  }

  if (rte_eth_dev_is_valid_port(port) == 0) {
    rte_exit(EXIT_FAILURE, "NIC ID is invalid\n");
  }
#if defined(ETH_IN_PROMISCUOUS_MODE)
  // All the rx examples have this enabled, but it doesn't seem like this is necessary
  // for our use case where we know the sender and dest addresses
  int promiscuous_en = rte_eth_promiscuous_get(port);
  rte_eth_promiscuous_enable(port);
  promiscuous_en = rte_eth_promiscuous_get(port);
  RtAssert(promiscuous_en == 1, "Unable to set promiscuous mode");
#endif

  // 1 == enable
  // This will make dpdk follow the flow rules to deliver packets to the application
  // Allows the application to not have to filter unnecessary traffic
  rte_flow_error flow_error;
  int en_isolate = rte_flow_isolate(port, 1, &flow_error);
  if (en_isolate != 0) {
    AGORA_LOG_WARN(
        "Flow cannot be isolated %d message: %s\n", flow_error.type,
        flow_error.message ? flow_error.message : "(no stated reason)");
    //RtAssert(en_isolate == 0, "Unable to set flow isolate mode");
  }

  retval = rte_eth_dev_info_get(port, &dev_info);
  if (retval < 0) {
    RtAssert(retval == 0, "Unable to obtain dev info");
    return retval;
  }

  int overhead;
  AGORA_LOG_INFO("Device max mtu %d rx max pktlen %d pkt size %zu\n",
                 dev_info.max_mtu, dev_info.max_rx_pktlen, pkt_len);
  if ((dev_info.max_mtu != UINT16_MAX) &&
      (dev_info.max_rx_pktlen > dev_info.max_mtu)) {
    overhead = dev_info.max_rx_pktlen - dev_info.max_mtu;
  } else {
    overhead = RTE_ETHER_HDR_LEN + RTE_ETHER_CRC_LEN;
  }
  AGORA_LOG_TRACE("Device overhead %d offset %zu\n", overhead, kPayloadOffset);

  //set the max rx packet size
  const uint16_t desired_max_size = pkt_len + kPayloadOffset - overhead;
  if (desired_max_size < dev_info.max_rx_pktlen) {
    AGORA_LOG_INFO("Setting max rx pkt size to %d\n", desired_max_size);
    dev_info.max_rx_pktlen = desired_max_size;
  }

  if ((dev_info.rx_offload_capa & DEV_RX_OFFLOAD_IPV4_CKSUM) ==
      DEV_TX_OFFLOAD_IPV4_CKSUM) {
    std::printf("DEV_RX_OFFLOAD_IPV4_CKSUM  enabled\n");
    port_conf.rxmode.offloads |= DEV_RX_OFFLOAD_IPV4_CKSUM;
  }
  if ((dev_info.rx_offload_capa & DEV_RX_OFFLOAD_UDP_CKSUM) ==
      DEV_RX_OFFLOAD_UDP_CKSUM) {
    std::printf("DEV_RX_OFFLOAD_UDP_CKSUM enabled\n");
    port_conf.rxmode.offloads |= DEV_RX_OFFLOAD_UDP_CKSUM;
  }

  //port_conf.rx_adv_conf.rss_conf.rss_hf &= dev_info.flow_type_rss_offloads;
  if ((dev_info.tx_offload_capa & DEV_TX_OFFLOAD_MBUF_FAST_FREE) ==
      DEV_TX_OFFLOAD_MBUF_FAST_FREE) {
    std::printf("DEV_TX_OFFLOAD_MBUF_FAST_FREE enabled\n");
    port_conf.txmode.offloads |= DEV_TX_OFFLOAD_MBUF_FAST_FREE;
  }
  if ((dev_info.tx_offload_capa & DEV_TX_OFFLOAD_IPV4_CKSUM) ==
      DEV_TX_OFFLOAD_IPV4_CKSUM) {
    std::printf("DEV_TX_OFFLOAD_IPV4_CKSUM  enabled\n");
    port_conf.txmode.offloads |= DEV_TX_OFFLOAD_IPV4_CKSUM;
  }
  if ((dev_info.tx_offload_capa & DEV_TX_OFFLOAD_UDP_CKSUM) ==
      DEV_TX_OFFLOAD_UDP_CKSUM) {
    std::printf("DEV_TX_OFFLOAD_UDP_CKSUM enabled\n");
    port_conf.txmode.offloads |= DEV_TX_OFFLOAD_UDP_CKSUM;
  }

  retval = rte_eth_dev_configure(port, rx_rings, tx_rings, &port_conf);
  if (retval != 0) {
    std::printf("Error in rte_eth_dev_configure\n");
    return retval;
  }
  retval = rte_eth_dev_adjust_nb_rx_tx_desc(port, &nb_rxd, &nb_txd);
  if (retval != 0) {
    std::printf("Error in rte_eth_dev_adjust_nb_rx_tx_desc\n");
    return retval;
  }

  //kPayloadOffset  overhead
  const uint16_t desired_mtu = desired_max_size;
  uint16_t mtu_size = desired_mtu;
  retval = rte_eth_dev_set_mtu(port, mtu_size);
  RtAssert(retval == 0, "Cannot set the MTU size for the given dev");

  retval = rte_eth_dev_get_mtu(port, &mtu_size);
  RtAssert(retval == 0, "Cannot get the MTU size for the given dev");
  RtAssert(mtu_size == desired_mtu, "Invalid MTU");

  //Setup RX
  rxconf = dev_info.default_rxconf;
  rxconf.offloads = port_conf.rxmode.offloads;
  for (q = 0; q < rx_rings; q++) {
    retval = rte_eth_rx_queue_setup(
        port, q, nb_rxd, rte_eth_dev_socket_id(port), &rxconf, mbuf_pool);
    if (retval < 0) {
      return retval;
    }
  }

  //Setup TX
  txconf = dev_info.default_txconf;
  txconf.offloads = port_conf.txmode.offloads;
  for (q = 0; q < tx_rings; q++) {
    retval = rte_eth_tx_queue_setup(port, q, nb_txd,
                                    rte_eth_dev_socket_id(port), &txconf);
    if (retval < 0) {
      return retval;
    }
  }

  retval = rte_eth_dev_start(port);
  if (retval < 0) {
    return retval;
  }

  rte_ether_addr addr;
  retval = rte_eth_macaddr_get(port, &addr);
  std::printf("NIC %u Socket: %d, MAC: %02" PRIx8 " %02" PRIx8 " %02" PRIx8
              " %02" PRIx8 " %02" PRIx8 " %02" PRIx8 " \n",
              port, rte_eth_dev_socket_id(port), addr.addr_bytes[0],
              addr.addr_bytes[1], addr.addr_bytes[2], addr.addr_bytes[3],
              addr.addr_bytes[4], addr.addr_bytes[5]);

  rte_eth_link link;
  rte_eth_link_get_nowait(port, &link);
  while (link.link_status == 0) {
    std::printf("Waiting for link up on NIC %" PRIu16 "\n", port);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    rte_eth_link_get_nowait(port, &link);
  }
  if (link.link_status == 0) {
    std::printf("Link down on NIC %" PRIx16 "\n", port);
    return 0;
  }
  return 0;
}

// Reference: https://stackoverflow.com/a/44948720
void DpdkTransport::FastMemcpy(void* pvDest, void* pvSrc, size_t nBytes) {
  // std::printf("pvDest: 0x%lx, pvSrc: 0x%lx, Dest: %lx, Src,
  // %lx\n",intptr_t(pvDest), intptr_t(pvSrc), (intptr_t(pvDest) & 31),
  // (intptr_t(pvSrc) & 31) ); assert(nBytes % 32 == 0);

#if defined(__AVX512F__) and (__GNUC__ >= 9)
  // Requires 64 Byte alignment of src and dest
  assert((reinterpret_cast<intptr_t>(pvDest) & 0x3F) == 0);
  assert((reinterpret_cast<intptr_t>(pvSrc) & 0x3F) == 0);

  __m512i* pSrc = reinterpret_cast<__m512i*>(pvSrc);
  __m512i* pDest = reinterpret_cast<__m512i*>(pvDest);
  int64_t nVects = nBytes / sizeof(*pSrc);
  for (; nVects > 0; nVects--, pSrc++, pDest++) {
    const __m512i loaded = _mm512_stream_load_si512(pSrc);
    _mm512_stream_si512(pDest, loaded);
  }
#else
  // Requires 32 Byte alignment of src and dest
  assert((reinterpret_cast<intptr_t>(pvDest) & 0x1F) == 0);
  assert((reinterpret_cast<intptr_t>(pvSrc) & 0x1F) == 0);
  const __m256i* p_src = reinterpret_cast<const __m256i*>(pvSrc);
  __m256i* p_dest = reinterpret_cast<__m256i*>(pvDest);
  int64_t n_vects = nBytes / sizeof(*p_src);
  for (; n_vects > 0; n_vects--, p_src++, p_dest++) {
    const __m256i loaded = _mm256_stream_load_si256(p_src);
    _mm256_stream_si256(p_dest, loaded);
  }
#endif
  _mm_sfence();
}

std::string DpdkTransport::PktToString(const rte_mbuf* pkt) {
  const uint8_t* buf = rte_pktmbuf_mtod(pkt, uint8_t*);

  std::ostringstream ret;
  ret << frame_header_to_string(buf) << " [ "
      << std::to_string(kPayloadOffset - kInetHdrsTotSize)
      << " unused bytes ] ";

  const auto* packet = reinterpret_cast<const Packet*>(buf + kPayloadOffset);
  ret << packet->ToString();
  return ret.str();
}

// Expecting Network Byte orders
void DpdkTransport::PrintPkt(rte_be32_t src_ip, rte_be32_t dst_ip,
                             rte_be16_t src_port, rte_be16_t dst_port,
                             size_t len, size_t tid) {
  uint8_t b[8u];
  uint16_t sp = rte_be_to_cpu_16(src_port);
  uint16_t dp = rte_be_to_cpu_16(dst_port);

  b[0] = src_ip & 0xFF;
  b[1] = (src_ip >> 8) & 0xFF;
  b[2] = (src_ip >> 16) & 0xFF;
  b[3] = (src_ip >> 24) & 0xFF;

  b[4] = dst_ip & 0xFF;
  b[5] = (dst_ip >> 8) & 0xFF;
  b[6] = (dst_ip >> 16) & 0xFF;
  b[7] = (dst_ip >> 24) & 0xFF;

  std::printf(
      "DpdkTransport[%zu]: received %u.%u.%u.%u:%u -> %u.%u.%u.%u:%u (%zu "
      "bytes)\n",
      tid, b[0u], b[1u], b[2u], b[3u], sp, b[4u], b[5u], b[6u], b[7u], dp, len);
}

// Matches source / dest udp ipv4 packets and rountes them to specific rx queues
void DpdkTransport::InstallFlowRule(uint16_t port_id, uint16_t rx_q,
                                    uint32_t src_ip, uint32_t dest_ip,
                                    uint16_t src_port, uint16_t dst_port) {
  //constexpr enum rte_flow_item_type kPattherEthIpv4Udp[] = {
  //    RTE_FLOW_ITEM_TYPE_ETH,
  //    RTE_FLOW_ITEM_TYPE_IPV4,
  //    RTE_FLOW_ITEM_TYPE_UDP,
  //    RTE_FLOW_ITEM_TYPE_END,
  //};
  rte_flow_attr attr;
  rte_flow_item pattern[4u];
  rte_flow_action action[2u];
  rte_flow_action_queue queue = {.index = rx_q};
  std::memset(pattern, 0u, sizeof(pattern));
  std::memset(action, 0u, sizeof(action));

  // Set the rule attribute. Only ingress packets will be checked.
  std::memset(&attr, 0u, sizeof(rte_flow_attr));
  attr.ingress = 1;
  attr.priority = 0;

  // Create the action sequence. One action only: move packet to queue
  action[0u].type = RTE_FLOW_ACTION_TYPE_QUEUE;
  action[0u].conf = &queue;
  action[1u].type = RTE_FLOW_ACTION_TYPE_END;

  // Set the first level of the pattern (ETH), allow all
  rte_flow_item& eth_pattern = pattern[0u];
  rte_flow_item_eth eth_spec;
  rte_flow_item_eth eth_mask;
  std::memset(&eth_spec, 0u, sizeof(rte_flow_item_eth));
  std::memset(&eth_mask, 0u, sizeof(rte_flow_item_eth));

  eth_spec.type = 0;
  eth_mask.type = 0;

  eth_pattern.type = RTE_FLOW_ITEM_TYPE_ETH;
  eth_pattern.spec = &eth_spec;
  eth_pattern.mask = &eth_mask;
  eth_pattern.last = nullptr;

  // Set the second level of the pattern (IP)
  rte_flow_item_ipv4 ip_spec;
  rte_flow_item_ipv4 ip_mask;
  std::memset(&ip_spec, 0u, sizeof(rte_flow_item_ipv4));
  std::memset(&ip_mask, 0u, sizeof(rte_flow_item_ipv4));
  //ip_spec.hdr.dst_addr = rte_cpu_to_be_32(dest_ip);
  ip_spec.hdr.dst_addr = dest_ip;
  ip_mask.hdr.dst_addr = 0xffffffff;
  //ip_spec.hdr.src_addr = rte_cpu_to_be_32(src_ip);
  ip_spec.hdr.src_addr = src_ip;
  ip_mask.hdr.src_addr = 0xffffffff;

  rte_flow_item& ipv4_pattern = pattern[1u];
  ipv4_pattern.type = RTE_FLOW_ITEM_TYPE_IPV4;
  ipv4_pattern.spec = &ip_spec;
  ipv4_pattern.mask = &ip_mask;
  ipv4_pattern.last = nullptr;

  // Set the third level of the pattern (UDP)
  rte_flow_item_udp udp_spec;
  rte_flow_item_udp udp_mask;

  rte_flow_item& udp_item = pattern[2u];
  udp_spec.hdr.src_port = rte_cpu_to_be_16(src_port);
  udp_spec.hdr.dst_port = rte_cpu_to_be_16(dst_port);
  udp_spec.hdr.dgram_len = 0;
  udp_spec.hdr.dgram_cksum = 0;

  udp_mask.hdr.src_port = 0xffff;
  udp_mask.hdr.dst_port = 0xffff;
  udp_mask.hdr.dgram_len = 0;
  udp_mask.hdr.dgram_cksum = 0;

  udp_item.type = RTE_FLOW_ITEM_TYPE_UDP;
  udp_item.spec = &udp_spec;
  udp_item.mask = &udp_mask;
  udp_item.last = nullptr;
  //End the filter
  pattern[3u].type = RTE_FLOW_ITEM_TYPE_END;

  rte_flow_error flow_error;
  int res = rte_flow_validate(port_id, &attr, pattern, action, &flow_error);
  if (res != 0) {
    std::printf("UDP flow rule cannot be validated %d message: %s\n",
                flow_error.type,
                flow_error.message ? flow_error.message : "(no stated reason)");
  }
  RtAssert(res == 0, "DPDK: Failed to validate flow rule");

  rte_flow* flow =
      rte_flow_create(port_id, &attr, pattern, action, &flow_error);
  RtAssert(flow != nullptr, "DPDK: Failed to install flow rule");
}

// Matches source / dest udp ipv4 packets and rountes them to specific rx queues
void DpdkTransport::InstallFlowRuleDropAll(uint16_t port_id) {
  rte_flow_attr attr;
  rte_flow_item pattern[2u];
  rte_flow_action action[2u];
  std::memset(pattern, 0u, sizeof(pattern));
  std::memset(action, 0u, sizeof(action));

  // Set the rule attribute. Only ingress packets will be checked.
  std::memset(&attr, 0u, sizeof(rte_flow_attr));
  attr.ingress = 1;
  attr.priority = 1;

  // Drop everything
  action[0u].type = RTE_FLOW_ACTION_TYPE_DROP;
  action[1u].type = RTE_FLOW_ACTION_TYPE_END;

  rte_flow_item& eth_pattern = pattern[0u];
  rte_flow_item_eth eth_spec;
  rte_flow_item_eth eth_mask;
  std::memset(&eth_spec, 0u, sizeof(rte_flow_item_eth));
  std::memset(&eth_mask, 0u, sizeof(rte_flow_item_eth));
  eth_spec.type = 0;
  eth_mask.type = 0;

  eth_pattern.type = RTE_FLOW_ITEM_TYPE_ETH;
  eth_pattern.spec = &eth_spec;
  eth_pattern.mask = &eth_mask;
  eth_pattern.last = nullptr;

  pattern[1u].type = RTE_FLOW_ITEM_TYPE_END;

  rte_flow_error error;
  int res = rte_flow_validate(port_id, &attr, pattern, action, &error);
  RtAssert(res == 0, "DPDK: Failed to validate drop all flow rule");

  rte_flow* flow = rte_flow_create(port_id, &attr, pattern, action, &error);
  RtAssert(flow != nullptr, "DPDK: Failed to install drop all flow rule");
}

rte_mbuf* DpdkTransport::AllocUdp(rte_mempool* mbuf_pool,
                                  rte_ether_addr src_mac_addr,
                                  rte_ether_addr dst_mac_addr,
                                  uint32_t src_ip_addr, uint32_t dst_ip_addr,
                                  uint16_t src_udp_port, uint16_t dst_udp_port,
                                  size_t buffer_length, uint16_t pkt_id) {
  rte_mbuf* tx_buf __attribute__((aligned(64)));
  tx_buf = rte_pktmbuf_alloc(mbuf_pool);

  rte_ether_hdr* eth_hdr = rte_pktmbuf_mtod(tx_buf, rte_ether_hdr*);
  eth_hdr->ether_type = rte_be_to_cpu_16(RTE_ETHER_TYPE_IPV4);
  std::memcpy(eth_hdr->src_addr.addr_bytes, src_mac_addr.addr_bytes,
              RTE_ETHER_ADDR_LEN);
  std::memcpy(eth_hdr->dst_addr.addr_bytes, dst_mac_addr.addr_bytes,
              RTE_ETHER_ADDR_LEN);

  auto* ip_h = (rte_ipv4_hdr*)((char*)eth_hdr + sizeof(rte_ether_hdr));
  ip_h->src_addr = src_ip_addr;
  ip_h->dst_addr = dst_ip_addr;
  ip_h->next_proto_id = IPPROTO_UDP;
  ip_h->version_ihl = 0x45;
  ip_h->type_of_service = 0;
  ip_h->total_length =
      rte_cpu_to_be_16(buffer_length + kPayloadOffset - sizeof(rte_ether_hdr));
  ip_h->packet_id = rte_cpu_to_be_16(pkt_id);
  //Do not fragment flag?
  ip_h->fragment_offset = rte_cpu_to_be_16(1 << 14);
  ip_h->time_to_live = 64;
  ip_h->hdr_checksum = 0;

  auto* udp_h = (rte_udp_hdr*)((char*)ip_h + sizeof(rte_ipv4_hdr));
  udp_h->src_port = rte_cpu_to_be_16(src_udp_port);
  udp_h->dst_port = rte_cpu_to_be_16(dst_udp_port);
  udp_h->dgram_len =
      rte_cpu_to_be_16(buffer_length + kPayloadOffset - sizeof(rte_ether_hdr) -
                       sizeof(rte_ipv4_hdr));
  udp_h->dgram_cksum = 0;

  tx_buf->pkt_len = buffer_length + kPayloadOffset;
  tx_buf->data_len = buffer_length + kPayloadOffset;
  //Should very that these offloads were enabled
  tx_buf->ol_flags =
      RTE_MBUF_F_TX_IPV4 | RTE_MBUF_F_TX_IP_CKSUM | RTE_MBUF_F_TX_UDP_CKSUM;
  //Not sure if this is needed?
  tx_buf->l2_len = sizeof(*eth_hdr);
  tx_buf->l3_len = sizeof(rte_ipv4_hdr);
  return tx_buf;
}

void DpdkTransport::DpdkInit(uint16_t core_offset, size_t thread_num) {
  // DPDK setup
  std::string core_list = std::to_string(GetPhysicalCoreId(core_offset));
  for (size_t i = 1; i < thread_num + 1; i++) {
    core_list =
        core_list + "," + std::to_string(GetPhysicalCoreId(core_offset + i));
  }
  // n: channels, m: maximum memory in megabytes
  const char* rte_argv[] = {"txrx",        "-l",           core_list.c_str(),
                            "--log-level", "lib.eal:info", nullptr};
  int rte_argc = static_cast<int>(sizeof(rte_argv) / sizeof(rte_argv[0])) - 1;

  // Initialize DPDK environment
  std::printf("Dpdk init on core start %d, num threads %zu\n", core_offset,
              thread_num);
  int ret = rte_eal_init(rte_argc, const_cast<char**>(rte_argv));
  RtAssert(
      ret >= 0,
      "Failed to initialize DPDK.  Are you running with root permissions?");
  std::printf("%s initialized\n", rte_version());
}

rte_mempool* DpdkTransport::CreateMempool(size_t num_ports,
                                          size_t packet_length) {
  const size_t mbuf_size = packet_length + kPayloadOffset + kMBufCacheSize;
  rte_mempool* mbuf_pool =
      rte_pktmbuf_pool_create("MBUF_POOL", kNumMBufs * num_ports,
                              kMBufCacheSize, 0, mbuf_size, rte_socket_id());

  RtAssert(mbuf_pool != nullptr, "Cannot create mbuf pool");
  return mbuf_pool;
}

#endif  // USE_DPDK