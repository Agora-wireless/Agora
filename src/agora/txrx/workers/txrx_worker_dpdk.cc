/**
 * @file txrx_worker_dpdk.cc
 * @brief Implementation of PacketTxRx initialization functions, and datapath
 * functions for communicating with simulators over dpdk.
 */

#include "txrx_worker_dpdk.h"

#include <arpa/inet.h>

#include <cassert>
#include <utility>

#include "dpdk_transport.h"
#include "gettime.h"
#include "logger.h"
#include "message.h"

static constexpr bool kDebugDPDK = false;

TxRxWorkerDpdk::TxRxWorkerDpdk(
    size_t core_offset, size_t tid, size_t interface_count,
    size_t interface_offset, Config* const config, size_t* rx_frame_start,
    moodycamel::ConcurrentQueue<EventData>* event_notify_q,
    moodycamel::ConcurrentQueue<EventData>* tx_pending_q,
    moodycamel::ProducerToken& tx_producer,
    moodycamel::ProducerToken& notify_producer,
    std::vector<RxPacket>& rx_memory, std::byte* const tx_memory,
    std::mutex& sync_mutex, std::condition_variable& sync_cond,
    std::atomic<bool>& can_proceed,
    std::vector<std::pair<uint16_t, uint16_t>> dpdk_phy, rte_mempool* mbuf_pool)
    : TxRxWorker(core_offset, tid, interface_count, interface_offset,
                 config->NumChannels(), config, rx_frame_start, event_notify_q,
                 tx_pending_q, tx_producer, notify_producer, rx_memory,
                 tx_memory, sync_mutex, sync_cond, can_proceed),
      dpdk_phy_port_queues_(std::move(dpdk_phy)),
      mbuf_pool_(mbuf_pool) {
  int ret = inet_pton(AF_INET, config->BsRruAddr().c_str(), &bs_rru_addr_);
  RtAssert(ret == 1, "Invalid sender IP address");
  ret = inet_pton(AF_INET, config->BsServerAddr().c_str(), &bs_server_addr_);
  RtAssert(ret == 1, "Invalid server IP address");

  RtAssert(dpdk_phy_port_queues_.size() == num_interfaces_,
           "The dev / queue id's list is not long enough to support the number "
           "of requested interfaces");

  //A worker should support multiple dpdk eth devices (ports)
  // and multi radios (interfaces, local ports) per device
  //Direct the traffic flow to this thread and its interfaces
  src_mac_.reserve(num_interfaces_);
  src_mac_.resize(num_interfaces_);
  dest_mac_.reserve(num_interfaces_);
  dest_mac_.resize(num_interfaces_);
  for (size_t interface = 0; interface < num_interfaces_; interface++) {
    const uint16_t dest_port =
        config->BsServerPort() + (interface + interface_offset_);
    const uint16_t src_port =
        config->BsRruPort() + (interface + interface_offset_);

    const auto& port_queue_id = dpdk_phy_port_queues_.at(interface);
    const auto& port_id = port_queue_id.first;
    const auto& queue_id = port_queue_id.second;

    auto status = rte_eth_macaddr_get(port_id, &src_mac_.at(interface));
    //addr.addr_bytes
    dest_mac_.at(interface) = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    RtAssert(status == 0, "Could not retreive mac address");

    AGORA_LOG_INFO(
        "Adding steering rule for src IP %s, dest IP %s, src port: "
        "%d, dst port: %d, DPDK dev %d, queue: %d\n",
        config->BsRruAddr().c_str(), config->BsServerAddr().c_str(), src_port,
        dest_port, port_id, queue_id);
    DpdkTransport::InstallFlowRule(port_id, queue_id, bs_rru_addr_,
                                   bs_server_addr_, src_port, dest_port);
  }
}

TxRxWorkerDpdk::~TxRxWorkerDpdk() { Stop(); };

//Todo: remove this
template <void (TxRxWorkerDpdk::*run_thread)()>
static void ClassFunctioWrapper(TxRxWorkerDpdk* context) {
  return (context->*run_thread)();
}

// DPDK doesn't use c++ std threads but the class still has a 1:1 mapping
// worker:lcore
void TxRxWorkerDpdk::Start() {
  rte_eal_wait_lcore(tid_);
  AGORA_LOG_TRACE("TxRxWorkerDpdk[%zu]: starting\n", tid_);
  const int status = rte_eal_remote_launch(
      (lcore_function_t*)(ClassFunctioWrapper<&TxRxWorkerDpdk::DoTxRx>), this,
      tid_);
  AGORA_LOG_INFO("TxRxWorkerDpdk[%zu]: started on dpdk managed l_core\n", tid_);
  RtAssert(status == 0, "Lcore cannot launch TxRx function");
}

void TxRxWorkerDpdk::Stop() {
  Configuration()->Running(false);
  //Wait until the lcore finishes its job (join)
  rte_eal_wait_lcore(tid_);
}

void TxRxWorkerDpdk::DoTxRx() {
  size_t prev_frame_id = SIZE_MAX;
  size_t rx_index = 0;
  const unsigned int thread_socket = rte_socket_id();

  AGORA_LOG_INFO("TxRxWorkerDpdk[%zu]: running on socket %u\n", tid_,
                 thread_socket);
  uint16_t dev_id = UINT16_MAX;
  for (auto& device_queue : dpdk_phy_port_queues_) {
    const uint16_t current_dev_id = device_queue.first;
    const unsigned int dev_socket = rte_eth_dev_socket_id(current_dev_id);
    if ((dev_id != current_dev_id) && (thread_socket != dev_socket)) {
      AGORA_LOG_WARN(
          "TxRxWorkerDpdk[%zu]: running on socket %u but the ethernet device "
          "is on socket %u\n",
          tid_, thread_socket, dev_socket);
    }
    dev_id = current_dev_id;
  }
  running_ = true;
  WaitSync();
  AGORA_LOG_TRACE("TxRxWorkerDpdk[%zu]: synced\n", tid_);

  while (Configuration()->Running()) {
    const size_t send_result = DequeueSend();
    if (0 == send_result) {
      const auto& port_queue_id = dpdk_phy_port_queues_.at(rx_index);
      auto rx_result = RecvEnqueue(port_queue_id.first, port_queue_id.second);
      for (auto& rx_packet : rx_result) {
        //Could move this to the Recv function
        if (kIsWorkerTimingEnabled) {
          const size_t& rx_frame_id = rx_packet->frame_id_;
          if ((prev_frame_id == SIZE_MAX) || (rx_frame_id > prev_frame_id)) {
            rx_frame_start_[rx_frame_id % kNumStatsFrames] = GetTime::Rdtsc();
            prev_frame_id = rx_frame_id;
          }
        }  // end kIsWorkerTimingEnabled
      }
      //Cycle through all ports / queues.  Don't wait for successful rx on any given config
      rx_index++;
      if (rx_index == dpdk_phy_port_queues_.size()) {
        rx_index = 0;
      }
    }  // send_result == 0
  }    // running
  running_ = false;
}

std::vector<Packet*> TxRxWorkerDpdk::RecvEnqueue(uint16_t port_id,
                                                 uint16_t queue_id) {
  std::vector<Packet*> rx_packets;
  std::array<rte_mbuf*, kRxBatchSize> rx_bufs;
  uint16_t nb_rx =
      rte_eth_rx_burst(port_id, queue_id, rx_bufs.data(), kRxBatchSize);

  for (size_t i = 0; i < nb_rx; i++) {
    rte_mbuf* dpdk_pkt = rx_bufs.at(i);

    auto* eth_hdr = rte_pktmbuf_mtod(dpdk_pkt, rte_ether_hdr*);
    auto* ip_hdr = reinterpret_cast<rte_ipv4_hdr*>(
        reinterpret_cast<uint8_t*>(eth_hdr) + sizeof(rte_ether_hdr));

    const uint16_t eth_type = rte_be_to_cpu_16(eth_hdr->ether_type);
    if (eth_type == RTE_ETHER_TYPE_VLAN) {
      AGORA_LOG_WARN("VLAN taged frame, larger than normal offset!");
      throw std::runtime_error("VLAN Tagging not supported!");
    }

    /// \todo Add support / detection of fragmented packets

    // This function will free the rx packet if returning true
    bool discard_rx = Filter(dpdk_pkt, port_id, queue_id);
    if (discard_rx == false) {
      if (kDebugDPDK) {
        auto* udp_h = reinterpret_cast<rte_udp_hdr*>(
            reinterpret_cast<uint8_t*>(ip_hdr) + sizeof(rte_ipv4_hdr));

        DpdkTransport::PrintPkt(ip_hdr->src_addr, ip_hdr->dst_addr,
                                udp_h->src_port, udp_h->dst_port,
                                dpdk_pkt->data_len, tid_);
        std::printf(
            "pkt_len: %d, datagram len %d, data len %d, nb_segs: %d, data "
            "offset %d, Header type: %d, IPv4: %d on dpdk dev %u and queue id "
            "%u\n",
            dpdk_pkt->pkt_len, rte_be_to_cpu_16(udp_h->dgram_len),
            dpdk_pkt->data_len, dpdk_pkt->nb_segs, dpdk_pkt->data_off,
            rte_be_to_cpu_16(eth_hdr->ether_type), RTE_ETHER_TYPE_IPV4, port_id,
            queue_id);
      }

      auto* payload = reinterpret_cast<uint8_t*>(eth_hdr) + kPayloadOffset;
      auto& rx = GetRxPacket();
      Packet* pkt = rx.RawPacket();

#if defined(USE_DPDK_MEMORY)
      rx.Set(dpdk_pkt, reinterpret_cast<Packet*>(payload));
#else
      rte_memcpy(reinterpret_cast<uint8_t*>(pkt), payload,
                 Configuration()->PacketLength());
      rte_pktmbuf_free(dpdk_pkt);
#endif

      AGORA_LOG_FRAME(
          "TxRxWorkerDpdk[%zu]::RecvEnqueue received pkt (frame %d, symbol "
          "%d, ant %d) on port %u queue %u\n",
          tid_, pkt->frame_id_, pkt->symbol_id_, pkt->ant_id_, port_id,
          queue_id);

      // Push kPacketRX event into the queue.
      const EventData rx_message(EventType::kPacketRX, rx_tag_t(rx).tag_);
      NotifyComplete(rx_message);
      rx_packets.push_back(pkt);
    }
  }
  return rx_packets;
}

size_t TxRxWorkerDpdk::DequeueSend() {
  auto tx_events = GetPendingTxEvents();

  //Process each pending tx event
  for (const EventData& current_event : tx_events) {
    assert(current_event.event_type_ == EventType::kPacketTX);

    const size_t ant_id = gen_tag_t(current_event.tags_[0]).ant_id_;
    const size_t frame_id = gen_tag_t(current_event.tags_[0]).frame_id_;
    const size_t symbol_id = gen_tag_t(current_event.tags_[0]).symbol_id_;
    // Make sure this was sent to the correct thread...
    const size_t interface_id = ant_id / channels_per_interface_;

    assert((interface_id >= interface_offset_) &&
           (interface_id <= (num_interfaces_ + interface_offset_)));

    auto* pkt = GetTxPacket(frame_id, symbol_id, ant_id);
    new (pkt)
        Packet(frame_id, symbol_id, Configuration()->CellId().at(0), ant_id);

    if (kDebugPrintInTask) {
      std::printf(
          "TxRxWorkerDpdk[%zu]::DequeueSend() Transmitted frame %zu, symbol "
          "%zu, ant %zu, tag %zu\n",
          tid_, frame_id, symbol_id, ant_id,
          gen_tag_t(current_event.tags_[0]).tag_);
    }

    const size_t local_interface_idx = interface_id - interface_offset_;

    rte_mbuf* tx_bufs __attribute__((aligned(64)));
    tx_bufs = DpdkTransport::AllocUdp(
        mbuf_pool_, src_mac_.at(local_interface_idx),
        dest_mac_.at(local_interface_idx), bs_server_addr_, bs_rru_addr_,
        Configuration()->BsServerPort() + interface_id,
        Configuration()->BsRruPort() + interface_id,
        Configuration()->DlPacketLength(), 1);

    static_assert(
        kTxBatchSize == 1,
        "kTxBatchSize must equal 1 - correct logic or set the value to 1");
    rte_ether_hdr* eth_hdr = rte_pktmbuf_mtod(tx_bufs, rte_ether_hdr*);
    auto* payload = reinterpret_cast<char*>(eth_hdr) + kPayloadOffset;

    rte_memcpy(payload, pkt, Configuration()->DlPacketLength());

    // Send data (one OFDM symbol)
    // Must send this out the correct port (dev) + queue that is assigned to this interface (convert global to local index)
    const auto& tx_info = dpdk_phy_port_queues_.at(local_interface_idx);
    size_t nb_tx_new =
        rte_eth_tx_burst(tx_info.first, tx_info.second, &tx_bufs, kTxBatchSize);
    if (unlikely(nb_tx_new != kTxBatchSize)) {
      AGORA_LOG_ERROR("TxRxWorkerDpdk[%zu]: rte_eth_tx_burst() failed\n", tid_);
      throw std::runtime_error("TxRxWorkerDpdk: rte_eth_tx_burst() failed");
    }
    const auto complete_event =
        EventData(EventType::kPacketTX, current_event.tags_[0]);
    NotifyComplete(complete_event);
  }
  return tx_events.size();
}

// return true if the packet is not useful
bool TxRxWorkerDpdk::Filter(rte_mbuf* packet, uint16_t port_id,
                            uint16_t queue_id) {
  auto* eth_hdr = rte_pktmbuf_mtod(packet, rte_ether_hdr*);
  const uint16_t eth_type = rte_be_to_cpu_16(eth_hdr->ether_type);

  bool discard = true;
  // By default, free and discard the message
  bool free_message = true;

  if (eth_type == RTE_ETHER_TYPE_IPV4) {
    auto* ip_hdr = reinterpret_cast<const rte_ipv4_hdr*>(
        reinterpret_cast<const uint8_t*>(eth_hdr) + sizeof(rte_ether_hdr));

    if ((ip_hdr->next_proto_id == IPPROTO_UDP) &&
        (ip_hdr->src_addr == bs_rru_addr_) &&
        (ip_hdr->dst_addr == bs_server_addr_)) {
      // Do not filter this out
      discard = false;
      free_message = false;
    } else {
      // IPV4 packet
      char pkt_dest_buf[INET_ADDRSTRLEN];
      char pkt_src_buf[INET_ADDRSTRLEN];
      ::in_addr pkt_dest;
      ::in_addr pkt_src;
      pkt_dest.s_addr = ip_hdr->dst_addr;
      pkt_src.s_addr = ip_hdr->src_addr;
      ::inet_ntop(AF_INET, &pkt_dest, pkt_dest_buf, sizeof(pkt_dest_buf));
      ::inet_ntop(AF_INET, &pkt_src, pkt_src_buf, sizeof(pkt_src_buf));

      std::printf(
          "TxRxWorkerDpdk[%zu]: Ignoring pkt rx on dev %d queue %d. "
          "Pkt dest addr %s : Pkt source addr %s : proto %u:%u\n",
          tid_, port_id, queue_id, pkt_dest_buf, pkt_src_buf,
          ip_hdr->next_proto_id, IPPROTO_UDP);
    }
  } else if (eth_type == RTE_ETHER_TYPE_ARP) {
    rte_ether_addr dst_addr;
    rte_ether_addr bond_mac_addr;

    throw std::runtime_error("Got an arp request?");

    //Handle ARP
    //auto* arp_hdr = (rte_arp_hdr*)((char*)(eth_hdr + 1) + offset);
    auto* arp_hdr = reinterpret_cast<rte_arp_hdr*>(
        reinterpret_cast<uint8_t*>(eth_hdr) + sizeof(rte_ether_hdr));
    if (arp_hdr->arp_opcode == rte_cpu_to_be_16(RTE_ARP_OP_REQUEST)) {
      AGORA_LOG_INFO("TxRxWorkerDpdk[%zu]: Arp request\n", tid_);
      rte_eth_macaddr_get(port_id, &bond_mac_addr);
      arp_hdr->arp_opcode = rte_cpu_to_be_16(RTE_ARP_OP_REPLY);
      // Switch src and dst data and set bonding MAC
      rte_ether_addr_copy(&eth_hdr->src_addr, &eth_hdr->dst_addr);
      rte_ether_addr_copy(&bond_mac_addr, &eth_hdr->src_addr);
      rte_ether_addr_copy(&arp_hdr->arp_data.arp_sha,
                          &arp_hdr->arp_data.arp_tha);
      arp_hdr->arp_data.arp_tip = arp_hdr->arp_data.arp_sip;
      rte_ether_addr_copy(&bond_mac_addr, &dst_addr);
      rte_ether_addr_copy(&dst_addr, &arp_hdr->arp_data.arp_sha);
      arp_hdr->arp_data.arp_sip = bs_server_addr_;
      //Message is reused and will be freed by the tx
      free_message = false;
      rte_eth_tx_burst(port_id, queue_id, &packet, 1);
    } else {
      AGORA_LOG_INFO("TxRxWorkerDpdk[%zu]: Arp - odcode %u\n", tid_,
                     arp_hdr->arp_opcode);
      rte_eth_tx_burst(port_id, queue_id, NULL, 0);
    }
  } else {
    AGORA_LOG_WARN("TxRxWorkerDpdk[%zu]: Rx pkt unhandled - type %u\n", tid_,
                   eth_type);
  }

  if (free_message == true) {
    rte_pktmbuf_free(packet);
  }
  return discard;
}