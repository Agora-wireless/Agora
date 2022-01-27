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
#include "logger.h"

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
    : TxRxWorker(core_offset, tid, interface_count, interface_offset, config,
                 rx_frame_start, event_notify_q, tx_pending_q, tx_producer,
                 notify_producer, rx_memory, tx_memory, sync_mutex, sync_cond,
                 can_proceed),
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
  for (size_t interface = 0; interface < num_interfaces_; interface++) {
    const uint16_t dest_port = rte_cpu_to_be_16(
        config->BsServerPort() + (interface + interface_offset_));
    const uint16_t src_port =
        rte_cpu_to_be_16(config->BsRruPort() + (interface + interface_offset_));

    const auto& port_queue_id = dpdk_phy_port_queues_.at(interface);
    const auto& port_id = port_queue_id.first;
    const auto& queue_id = port_queue_id.second;

    MLPD_INFO(
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
  MLPD_INFO("TxRxWorkerDpdk[%zu] starting\n", tid_);
  int status = rte_eal_remote_launch(
      (lcore_function_t*)(ClassFunctioWrapper<&TxRxWorkerDpdk::DoTxRx>), this,
      tid_);
  MLPD_INFO("TxRxWorkerDpdk: started on lcore %zu\n", tid_);
  RtAssert(status == 0, "Lcore cannot launch TxRx function");
}

void TxRxWorkerDpdk::Stop() {
  Configuration()->Running(false);
  MLPD_INFO("TxRxWorker[%zu] stopping\n", tid_);
  rte_eal_wait_lcore(tid_);
  MLPD_INFO("TxRxWorker[%zu] stopped\n", tid_);
}

void TxRxWorkerDpdk::DoTxRx() {
  size_t prev_frame_id = SIZE_MAX;
  size_t rx_index = 0;

  running_ = true;
  WaitSync();

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
    uint16_t eth_type = rte_be_to_cpu_16(eth_hdr->ether_type);
    if (kDebugDPDK) {
      auto* udp_h = reinterpret_cast<rte_udp_hdr*>(
          reinterpret_cast<uint8_t*>(ip_hdr) + sizeof(rte_ipv4_hdr));
      DpdkTransport::PrintPkt(ip_hdr->src_addr, ip_hdr->dst_addr,
                              udp_h->src_port, udp_h->dst_port,
                              dpdk_pkt->data_len, H5T_UNIX_D32BE_g);
      std::printf("pkt_len: %d, nb_segs: %d, Header type: %d, IPV4: %d\n",
                  dpdk_pkt->pkt_len, dpdk_pkt->nb_segs, eth_type,
                  RTE_ETHER_TYPE_IPV4);
      std::printf("UDP: %d, %d\n", ip_hdr->next_proto_id, IPPROTO_UDP);
    }

    bool discard_rx = false;

    //Is this the data we care about
    if ((eth_type != RTE_ETHER_TYPE_IPV4) ||
        (ip_hdr->next_proto_id != IPPROTO_UDP)) {
      discard_rx = true;
    }

    if (ip_hdr->src_addr != bs_rru_addr_) {
      std::fprintf(stderr, "DPDK: Source addr does not match\n");
      discard_rx = true;
    }
    if (ip_hdr->dst_addr != bs_server_addr_) {
      std::fprintf(stderr, "DPDK: Destination addr does not match\n");
      discard_rx = true;
    }

    if (discard_rx == false) {
      auto* payload = reinterpret_cast<uint8_t*>(eth_hdr) + kPayloadOffset;
      auto& rx = GetRxPacket();
      Packet* pkt = rx.RawPacket();

#if defined(USE_DPDK_MEMORY)
      rx.Set(dpdk_pkt, reinterpret_cast<Packet*>(payload));
#else
      DpdkTransport::FastMemcpy(reinterpret_cast<uint8_t*>(pkt), payload,
                                Configuration()->PacketLength());
      rte_pktmbuf_free(dpdk_pkt);
#endif

      // Push kPacketRX event into the queue.
      EventData rx_message(EventType::kPacketRX, rx_tag_t(rx).tag_);
      NotifyComplete(rx_message);
      rx_packets.push_back(pkt);
    } else {
      // Not using the rx data
      rte_pktmbuf_free(dpdk_pkt);
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

    rte_mbuf* tx_bufs[kTxBatchSize] __attribute__((aligned(64)));
    static_assert(
        kTxBatchSize == 1,
        "kTxBatchSize must equal 1 - correct logic or set the value to 1");
    tx_bufs[0] = rte_pktmbuf_alloc(mbuf_pool_);
    rte_ether_hdr* eth_hdr = rte_pktmbuf_mtod(tx_bufs[0], rte_ether_hdr*);
    eth_hdr->ether_type = rte_be_to_cpu_16(RTE_ETHER_TYPE_IPV4);

    rte_ipv4_hdr* ip_h =
        (rte_ipv4_hdr*)((char*)eth_hdr + sizeof(rte_ether_hdr));
    ip_h->src_addr = bs_server_addr_;
    ip_h->dst_addr = bs_rru_addr_;
    ip_h->next_proto_id = IPPROTO_UDP;

    rte_udp_hdr* udp_h = (rte_udp_hdr*)((char*)ip_h + sizeof(rte_ipv4_hdr));
    udp_h->src_port =
        rte_cpu_to_be_16(Configuration()->BsServerPort() + interface_id);
    udp_h->dst_port =
        rte_cpu_to_be_16(Configuration()->BsRruPort() + interface_id);

    tx_bufs[0]->pkt_len = Configuration()->DlPacketLength() + kPayloadOffset;
    tx_bufs[0]->data_len = Configuration()->DlPacketLength() + kPayloadOffset;
    char* payload = (char*)eth_hdr + kPayloadOffset;
    DpdkTransport::FastMemcpy(payload, (char*)pkt,
                              Configuration()->DlPacketLength());

    // Send data (one OFDM symbol)
    // Must send this out the correct port (dev) + queue that is assigned to this interface (convert gloabl to local index)
    const auto& tx_info =
        dpdk_phy_port_queues_.at(interface_id - interface_offset_);
    size_t nb_tx_new =
        rte_eth_tx_burst(tx_info.first, tx_info.second, tx_bufs, kTxBatchSize);
    if (unlikely(nb_tx_new != kTxBatchSize)) {
      std::printf("TxRxWorkerDpdk[%zu]: rte_eth_tx_burst() failed\n", tid_);
      throw std::runtime_error("TxRxWorkerDpdk: rte_eth_tx_burst() failed");
    }
    auto complete_event =
        EventData(EventType::kPacketTX, current_event.tags_[0]);
    NotifyComplete(complete_event);
  }
  return tx_events.size();
}