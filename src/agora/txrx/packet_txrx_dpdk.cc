/**
 * @file packet_txrx_dpdk.cc
 * @brief Implementation of PacketTxRxDpdk datapath functions for communicating
 * with DPDK
 */

#include "packet_txrx_dpdk.h"

static constexpr bool kDebugDPDK = false;

PacketTxRxDpdk::PacketTxRxDpdk(Config* cfg, size_t core_offset)
    : cfg_(cfg),
      core_offset_(core_offset),
      ant_per_cell_(cfg->BsAntNum() / cfg->NumCells()),
      socket_thread_num_(cfg->SocketThreadNum()) {
  DpdkTransport::DpdkInit(core_offset_ - 1, socket_thread_num_);
  std::printf(
      "Number of ports: %d used (offset: %d), %d available, socket: %d\n",
      cfg_->DpdkNumPorts(), cfg_->DpdkPortOffset(), rte_eth_dev_count_avail(),
      rte_socket_id());
  RtAssert(cfg_->DpdkNumPorts() <= rte_eth_dev_count_avail(),
           "Invalid number of DPDK ports");
  mbuf_pool_ = DpdkTransport::CreateMempool(cfg->DpdkNumPorts());

  int ret = inet_pton(AF_INET, cfg_->BsRruAddr().c_str(), &bs_rru_addr_);
  RtAssert(ret == 1, "Invalid sender IP address");
  ret = inet_pton(AF_INET, cfg_->BsServerAddr().c_str(), &bs_server_addr_);
  RtAssert(ret == 1, "Invalid server IP address");

  for (uint16_t port_id = 0; port_id < cfg_->DpdkNumPorts(); port_id++) {
    if (DpdkTransport::NicInit(port_id + cfg->DpdkPortOffset(), mbuf_pool_,
                               socket_thread_num_) != 0) {
      rte_exit(EXIT_FAILURE, "Cannot init port %u\n",
               port_id + cfg->DpdkPortOffset());
    }
  }

  for (size_t i = 0; i < socket_thread_num_; i++) {
    uint16_t src_port = rte_cpu_to_be_16(cfg_->BsRruPort() + i);
    uint16_t dst_port = rte_cpu_to_be_16(cfg_->BsServerPort() + i);

    std::printf(
        "Adding steering rule for src IP %s, dest IP %s, src port: %zu, "
        "dst port: %zu, DPDK port %zu, queue: %zu\n",
        this->cfg_->BsRruAddr().c_str(), this->cfg_->BsServerAddr().c_str(),
        this->cfg_->BsRruPort() + i, this->cfg_->BsServerPort() + i,
        i % this->cfg_->DpdkNumPorts() + cfg->DpdkPortOffset(),
        i / this->cfg_->DpdkNumPorts());
    DpdkTransport::InstallFlowRule(
        i % this->cfg_->DpdkNumPorts() + cfg->DpdkPortOffset(),
        i / this->cfg_->DpdkNumPorts(), bs_rru_addr_, bs_server_addr_, src_port,
        dst_port);
  }

  std::printf("Number of DPDK cores: %d\n", rte_lcore_count());
}

PacketTxRxDpdk::PacketTxRxDpdk(
    Config* cfg, size_t core_offset,
    moodycamel::ConcurrentQueue<EventData>* queue_message,
    moodycamel::ConcurrentQueue<EventData>* queue_task,
    moodycamel::ProducerToken** rx_ptoks, moodycamel::ProducerToken** tx_ptoks)
    : PacketTxRxDpdk(cfg, core_offset) {
  message_queue_ = queue_message;
  task_queue_ = queue_task;
  rx_ptoks_ = rx_ptoks;
  tx_ptoks_ = tx_ptoks;
}

PacketTxRxDpdk::~PacketTxRxDpdk() { rte_mempool_free(mbuf_pool_); }

bool PacketTxRxDpdk::StartTxRx(Table<complex_float>& calib_dl_buffer,
                               Table<complex_float>& calib_ul_buffer) {
  unused(calib_dl_buffer);
  unused(calib_ul_buffer);

  frame_start_ = &frame_start;
  buffers_per_socket_ = packet_num_in_buffer / socket_thread_num_;
  tx_buffer_ = tx_buffer;

  rx_packets_.resize(socket_thread_num_);
  for (size_t i = 0; i < socket_thread_num_; i++) {
    rx_packets_.at(i).reserve(buffers_per_socket_);
#if defined(USE_DPDK_MEMORY)
    unused(buffer);
    rx_packets_dpdk_.at(i).resize(buffers_per_socket_);
#else
    for (size_t number_packets = 0; number_packets < buffers_per_socket_;
         number_packets++) {
      auto* pkt_loc = reinterpret_cast<Packet*>(
          buffer[i] + (number_packets * cfg_->PacketLength()));
      rx_packets_.at(i).emplace_back(pkt_loc);
    }
#endif  // defined(USE_DPDK_MEMORY)
  }

  unsigned int lcore_id;
  size_t worker_id = 0;
  // Launch specific task to cores
  // For dpdk version >= 20.11.1 use RTE_LCORE_FOREACH_WORKER
  RTE_LCORE_FOREACH_SLAVE(lcore_id) {
    // launch communication and task thread onto specific core
    if (worker_id < socket_thread_num_) {
      auto context = new EventHandlerContext<PacketTxRxDpdk>;
      context->obj_ptr_ = this;
      context->id_ = worker_id;
      rte_eal_remote_launch(
          (lcore_function_t*)
              PthreadFunWrapper<PacketTxRxDpdk, &PacketTxRxDpdk::LoopTxRx>,
          context, lcore_id);
      std::printf("DPDK TXRX thread %zu: pinned to core %d\n", worker_id,
                  lcore_id);
    }
    worker_id++;
  }
  return true;
}

void PacketTxRxDpdk::DoTxRx(size_t tid) {
  size_t rx_slot = 0;
  size_t prev_frame_id = SIZE_MAX;
  const uint16_t port_id = tid % cfg_->DpdkNumPorts() + cfg_->DpdkPortOffset();
  const uint16_t queue_id = tid / cfg_->DpdkNumPorts();

  while (cfg_->Running()) {
    if (0 == DequeueSend(tid)) {
      DpdkRecv((int)tid, port_id, queue_id, prev_frame_id, rx_slot);
    }
  }
}

uint16_t PacketTxRxDpdk::DpdkRecv(size_t tid, uint16_t port_id,
                                  uint16_t queue_id, size_t& prev_frame_id,
                                  size_t& rx_slot) {
  rte_mbuf* rx_bufs[kRxBatchSize];
  uint16_t nb_rx = rte_eth_rx_burst(port_id, queue_id, rx_bufs, kRxBatchSize);
  if (unlikely(nb_rx == 0)) return 0;

  for (size_t i = 0; i < nb_rx; i++) {
    rte_mbuf* dpdk_pkt = rx_bufs[i];

    // If the RX buffer is full, it means that the base station processing
    // hasn't kept up, so exit.
    auto& rx = rx_packets_.at(tid).at(rx_slot);

    if (rx.Empty() == false) {
      std::printf("TXRX thread [%d] DpdkRecv rx_buffer full, slot: %zu\n", tid,
                  rx_slot);
      cfg_->Running(false);
      return 0;
    }

    auto* eth_hdr = rte_pktmbuf_mtod(dpdk_pkt, rte_ether_hdr*);
    auto* ip_hdr = reinterpret_cast<rte_ipv4_hdr*>(
        reinterpret_cast<uint8_t*>(eth_hdr) + sizeof(rte_ether_hdr));
    uint16_t eth_type = rte_be_to_cpu_16(eth_hdr->ether_type);
    if (kDebugDPDK) {
      auto* udp_h = reinterpret_cast<rte_udp_hdr*>(
          reinterpret_cast<uint8_t*>(ip_hdr) + sizeof(rte_ipv4_hdr));
      DpdkTransport::PrintPkt(ip_hdr->src_addr, ip_hdr->dst_addr,
                              udp_h->src_port, udp_h->dst_port,
                              dpdk_pkt->data_len, tid);
      std::printf("pkt_len: %d, nb_segs: %d, Header type: %d, IPV4: %d\n",
                  dpdk_pkt->pkt_len, dpdk_pkt->nb_segs, eth_type,
                  RTE_ETHER_TYPE_IPV4);
      std::printf("UDP: %d, %d\n", ip_hdr->next_proto_id, IPPROTO_UDP);
    }

    if (eth_type != RTE_ETHER_TYPE_IPV4 or
        ip_hdr->next_proto_id != IPPROTO_UDP) {
      rte_pktmbuf_free(dpdk_pkt);
      continue;
    }

    if (ip_hdr->src_addr != bs_rru_addr_) {
      std::fprintf(stderr, "DPDK: Source addr does not match\n");
      rte_pktmbuf_free(dpdk_pkt);
      continue;
    }
    if (ip_hdr->dst_addr != bs_server_addr_) {
      std::fprintf(stderr, "DPDK: Destination addr does not match\n");
      rte_pktmbuf_free(dpdk_pkt);
      continue;
    }

    auto* payload = reinterpret_cast<uint8_t*>(eth_hdr) + kPayloadOffset;
#if defined(USE_DPDK_MEMORY)
    rx.Set(dpdk_pkt, reinterpret_cast<Packet*>(payload));
#else
    DpdkTransport::FastMemcpy(reinterpret_cast<uint8_t*>(rx.RawPacket()),
                              payload, cfg_->PacketLength());
    rte_pktmbuf_free(dpdk_pkt);
#endif

    if (kIsWorkerTimingEnabled) {
      if (prev_frame_id == SIZE_MAX or
          rx.RawPacket()->frame_id_ > prev_frame_id) {
        (*frame_start_)[tid][rx.RawPacket()->frame_id_ % kNumStatsFrames] =
            GetTime::Rdtsc();
        prev_frame_id = rx.RawPacket()->frame_id_;
      }
    }

    rx.Use();
    if (message_queue_->enqueue(
            *rx_ptoks_[tid],
            EventData(EventType::kPacketRX, rx_tag_t(rx).tag_)) == false) {
      std::printf("Failed to enqueue socket message\n");
      throw std::runtime_error(
          "PacketTxRxDpdk: Failed to enqueue socket message");
    }
    rx_slot = (rx_slot + 1) % buffers_per_socket_;
  }
  return nb_rx;
}

// TODO: check correctness of this funcion
size_t PacketTxRxDpdk::DequeueSend(size_t tid) {
  EventData event;
  size_t dequeue_events = 0;

  bool status = task_queue_->try_dequeue_from_producer(*tx_ptoks_[tid], event);
  if (status == true) {
    dequeue_events = 1;
    // std::printf("tx queue length: %d\n", task_queue_->size_approx());
    assert(event.event_type_ == EventType::kPacketTX);

    size_t ant_id = gen_tag_t(event.tags_[0]).ant_id_;
    size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
    size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;

    size_t data_symbol_idx_dl = this->cfg_->Frame().GetDLSymbolIdx(symbol_id);
    size_t offset =
        (this->cfg_->GetTotalDataSymbolIdxDl(frame_id, data_symbol_idx_dl) *
         this->cfg_->BsAntNum()) +
        ant_id;

    if (kDebugPrintInTask) {
      std::printf(
          "In TX thread %d: Transmitted frame %zu, symbol %zu, "
          "ant %zu, tag %zu, offset: %zu, msg_queue_length: %zu\n",
          tid, frame_id, symbol_id, ant_id, gen_tag_t(event.tags_[0]).tag_,
          offset, message_queue_->size_approx());
    }

    char* cur_buffer_ptr = tx_buffer_ + offset * this->cfg_->DlPacketLength();
    auto* pkt = (Packet*)cur_buffer_ptr;
    new (pkt) Packet(frame_id, symbol_id, 0 /* cell_id */, ant_id);

    struct rte_mbuf* tx_bufs[kTxBatchSize] __attribute__((aligned(64)));
    tx_bufs[0] = rte_pktmbuf_alloc(mbuf_pool_);
    struct rte_ether_hdr* eth_hdr =
        rte_pktmbuf_mtod(tx_bufs[0], struct rte_ether_hdr*);
    eth_hdr->ether_type = rte_be_to_cpu_16(RTE_ETHER_TYPE_IPV4);

    struct rte_ipv4_hdr* ip_h =
        (struct rte_ipv4_hdr*)((char*)eth_hdr + sizeof(struct rte_ether_hdr));
    ip_h->src_addr = bs_server_addr_;
    ip_h->dst_addr = bs_rru_addr_;
    ip_h->next_proto_id = IPPROTO_UDP;

    struct rte_udp_hdr* udp_h =
        (struct rte_udp_hdr*)((char*)ip_h + sizeof(struct rte_ipv4_hdr));
    udp_h->src_port = rte_cpu_to_be_16(this->cfg_->BsServerPort() + tid);
    udp_h->dst_port = rte_cpu_to_be_16(this->cfg_->BsRruPort() + tid);

    tx_bufs[0]->pkt_len = this->cfg_->DlPacketLength() + kPayloadOffset;
    tx_bufs[0]->data_len = this->cfg_->DlPacketLength() + kPayloadOffset;
    char* payload = (char*)eth_hdr + kPayloadOffset;
    DpdkTransport::FastMemcpy(payload, (char*)pkt,
                              this->cfg_->DlPacketLength());

    // Send data (one OFDM symbol)
    size_t nb_tx_new = rte_eth_tx_burst(0, tid, tx_bufs, 1);
    if (unlikely(nb_tx_new != 1)) {
      std::printf("rte_eth_tx_burst() failed\n");
      throw std::runtime_error("PacketTxRxDpdk: rte_eth_tx_burst() failed");
    }
    RtAssert(
        message_queue_->enqueue(
            *rx_ptoks_[tid], EventData(EventType::kPacketTX, event.tags_[0])),
        "Socket message enqueue failed\n");
  }
  return dequeue_events;
}

bool PacketTxRxDpdk::CreateWorker(size_t tid, size_t interface_count,
                                  size_t interface_offset,
                                  size_t* rx_frame_start,
                                  std::vector<RxPacket>& rx_memory,
                                  std::byte* const tx_memory) {
  RtAssert(false, "This function is not supported in DPDK mode\n") l
}
