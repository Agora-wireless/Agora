/**
 * @file sender.cc
 * @brief Implementation file for the sender class
 */
#include "sender.h"

#include <algorithm>
#include <csignal>
#include <thread>

#include "datatype_conversion.h"
#include "gettime.h"
#include "logger.h"
#include "message.h"
#include "udp_client.h"

#if defined(USE_DPDK)
#define DPDK_BURST_BULK
#include <arpa/inet.h>
#endif

static constexpr bool kDebugPrintSender = false;

static std::atomic<bool> keep_running = true;
// A spinning barrier to synchronize the start of worker threads
static std::atomic<size_t> num_workers_ready_atomic = 0;

void InterruptHandler(int /*unused*/) {
  std::cout << "Will exit..." << std::endl;
  keep_running.store(false);
}

void DelayTicks(uint64_t start, uint64_t ticks) {
  while ((GetTime::Rdtsc() - start) < ticks) {
    _mm_pause();
  }
}

Sender::Sender(Config* cfg, size_t socket_thread_num, size_t core_offset,
               size_t frame_duration, size_t inter_frame_delay,
               size_t enable_slow_start, const std::string& server_mac_addr_str,
               bool create_thread_for_master)
    : cfg_(cfg),
      freq_ghz_(GetTime::MeasureRdtscFreq()),
      ticks_per_usec_(freq_ghz_ * 1e3),
      socket_thread_num_(socket_thread_num),
      enable_slow_start_(enable_slow_start),
      core_offset_(core_offset),
      inter_frame_delay_(inter_frame_delay),
      ticks_inter_frame_(inter_frame_delay_ * ticks_per_usec_) {
  if (frame_duration == 0) {
    frame_duration_ =
        (cfg->Frame().NumTotalSyms() * cfg->SampsPerSymbol() * 1000000ul) /
        cfg->Rate();
  } else {
    frame_duration_ = frame_duration;
  }

  ticks_all_ =
      ((frame_duration_ * ticks_per_usec_) / cfg->Frame().NumTotalSyms());
  ticks_wnd1_ = std::max(ticks_all_ * 40ul,
                         static_cast<uint64_t>((200000.0 * ticks_per_usec_) /
                                               cfg->Frame().NumTotalSyms()));
  ticks_wnd2_ = ticks_all_ * 15;

  AGORA_LOG_INFO(
      "Initializing sender, sending to base station server at %s, frame "
      "duration = %.2f ms, slow start = %s\n",
      cfg->BsServerAddr().c_str(), frame_duration_ / 1000.0,
      enable_slow_start == 1 ? "yes" : "no");

  unused(server_mac_addr_str);
  for (auto& i : packet_count_per_symbol_) {
    i = new size_t[cfg->Frame().NumTotalSyms()]();
  }

  InitIqFromFile(std::string(TOSTRING(PROJECT_DIRECTORY)) +
                 "/files/experiment/LDPC_rx_data_" +
                 std::to_string(cfg->OfdmCaNum()) + "_ant" +
                 std::to_string(cfg->BsAntNum()) + ".bin");

  task_ptok_ =
      static_cast<moodycamel::ProducerToken**>(Agora_memory::PaddedAlignedAlloc(
          Agora_memory::Alignment_t::kAlign64,
          (socket_thread_num * sizeof(moodycamel::ProducerToken*))));
  for (size_t i = 0; i < socket_thread_num; i++) {
    task_ptok_[i] = new moodycamel::ProducerToken(send_queue_);
  }

  // Create a master thread when started from simulator
  if (create_thread_for_master == true) {
    threads_.emplace_back(&Sender::MasterThread, this, socket_thread_num_);
  }

#if defined(USE_DPDK)
  DpdkTransport::DpdkInit(core_offset, socket_thread_num_);
  std::printf(
      "Number of ports: %d used (offset: %d), %d available, socket: %d\n",
      cfg->DpdkNumPorts(), cfg->DpdkPortOffset(), rte_eth_dev_count_avail(),
      rte_socket_id());
  RtAssert(cfg->DpdkNumPorts() <= rte_eth_dev_count_avail(),
           "Invalid number of DPDK ports");
  mbuf_pool_ =
      DpdkTransport::CreateMempool(cfg->DpdkNumPorts(), cfg->PacketLength());

  // Parse IP addresses
  int status = inet_pton(AF_INET, cfg->BsRruAddr().c_str(), &bs_rru_addr_);
  RtAssert(status == 1, "Invalid sender IP address");
  status = inet_pton(AF_INET, cfg->BsServerAddr().c_str(), &bs_server_addr_);
  RtAssert(status == 1, "Invalid server IP address");

  RtAssert(server_mac_addr_str.length() ==
               (cfg->DpdkNumPorts() * (kMacAddrBtyes + 1) - 1),
           "Invalid length of server MAC address");
  sender_mac_addr_.resize(cfg->DpdkNumPorts());
  server_mac_addr_.resize(cfg->DpdkNumPorts());

  if (cfg->DpdkMacAddrs().length() > 0) {
    port_ids_ = DpdkTransport::GetPortIDFromMacAddr(cfg->DpdkNumPorts(),
                                                    cfg->DpdkMacAddrs());
  } else {
    for (uint16_t i = 0; i < cfg->DpdkNumPorts(); i++) {
      port_ids_.push_back(i + cfg->DpdkPortOffset());
    }
  }
  for (size_t i = 0; i < cfg->DpdkNumPorts(); i++) {
    // Parse MAC addresses
    ether_addr* parsed_mac = ether_aton(
        server_mac_addr_str.substr(i * (kMacAddrBtyes + 1), kMacAddrBtyes)
            .c_str());
    RtAssert(parsed_mac != nullptr, "Invalid server mac address");

    const int num_queues = cfg_->NumRadios() / cfg->DpdkNumPorts();
    const auto nic_status = DpdkTransport::NicInit(
        port_ids_.at(i), mbuf_pool_, num_queues, cfg->PacketLength());
    if (nic_status != 0) {
      rte_exit(EXIT_FAILURE, "Cannot init port %u\n", port_ids_.at(i));
    }
    std::memcpy(&server_mac_addr_[i], parsed_mac, sizeof(ether_addr));

    status = rte_eth_macaddr_get(port_ids_.at(i), &sender_mac_addr_[i]);
    RtAssert(status == 0, "Cannot get MAC address of the port");
    AGORA_LOG_INFO("Number of DPDK cores: %d\n", rte_lcore_count());

    std::printf("Sending IP(MAC): From %s To %s(%s)\n",
                cfg->BsRruAddr().c_str(), cfg->BsServerAddr().c_str(),
                ether_ntoa(parsed_mac));
  }

#endif
  num_workers_ready_atomic.store(0);
}

Sender::~Sender() {
  keep_running.store(false);

  for (auto& thread : threads_) {
    AGORA_LOG_INFO("Sender: Joining threads\n");
    thread.join();
  }

  iq_data_short_.Free();
  for (auto& i : packet_count_per_symbol_) {
    delete[] i;
    i = nullptr;
  }

  for (size_t i = 0; i < socket_thread_num_; i++) {
    delete (task_ptok_[i]);
    task_ptok_[i] = nullptr;
  }
  std::free(task_ptok_);
  AGORA_LOG_INFO("Sender: Complete\n");
}

void Sender::StartTx() {
  frame_start_ = new double[kNumStatsFrames]();
  frame_end_ = new double[kNumStatsFrames]();

  CreateWorkerThreads(socket_thread_num_);
  std::signal(SIGINT, InterruptHandler);
  MasterThread(0);  // Start the master thread

  delete[](frame_start_);
  delete[](frame_end_);
}

void Sender::StartTxfromMain(double* in_frame_start, double* in_frame_end) {
  frame_start_ = in_frame_start;
  frame_end_ = in_frame_end;

  CreateWorkerThreads(socket_thread_num_);
}

size_t Sender::FindNextSymbol(size_t start_symbol) {
  size_t next_symbol_id;
  for (next_symbol_id = start_symbol;
       (next_symbol_id < cfg_->Frame().NumTotalSyms()); next_symbol_id++) {
    SymbolType symbol_type = cfg_->GetSymbolType(next_symbol_id);
    if ((symbol_type == SymbolType::kPilot) ||
        (symbol_type == SymbolType::kUL)) {
      break;
    }
  }
  return next_symbol_id;
}

void Sender::ScheduleSymbol(size_t frame, size_t symbol_id) {
  size_t radios_per_worker = (cfg_->NumRadios() / socket_thread_num_);
  if ((cfg_->NumRadios() % socket_thread_num_) != 0) {
    radios_per_worker++;
  }
  const size_t ant_per_worker = radios_per_worker * cfg_->NumChannels();
  //Change to bulk enqueue
  for (size_t ant_num = 0; ant_num < cfg_->BsAntNum(); ant_num++) {
    const size_t worker_id = ant_num / ant_per_worker;
    auto req_tag = gen_tag_t::FrmSymAnt(frame, symbol_id, ant_num);
    // Split up the antennas amoung the worker threads
    RtAssert(send_queue_.enqueue(*task_ptok_[worker_id], req_tag.tag_),
             "Send task enqueue failed");
  }
}

void* Sender::MasterThread(int /*unused*/) {
  constexpr size_t kSenderMasterTid = 0;
  PinToCoreWithOffset(ThreadType::kMasterTX, core_offset_, kSenderMasterTid);

  // Wait for all worker threads to be ready (+1 for Master)
  num_workers_ready_atomic.fetch_add(1);
  while (num_workers_ready_atomic.load() < (socket_thread_num_ + 1)) {
    // Wait
  }

  uint64_t tick_start = GetTime::Rdtsc();
  double frame_start_us = GetTime::GetTimeUs();
  double frame_end_us = 0;
  frame_start_[0] = frame_start_us;

  size_t start_symbol = FindNextSymbol(0);
  // Delay until the start of the first symbol
  if (start_symbol > 0) {
    AGORA_LOG_INFO("Sender: Starting symbol %zu delaying\n", start_symbol);
    DelayTicks(tick_start, GetTicksForFrame(0) * start_symbol);
    tick_start = tick_start + (GetTicksForFrame(0) * start_symbol);
  }
  RtAssert(start_symbol != cfg_->Frame().NumTotalSyms(),
           "Sender: No valid symbols to transmit");
  ScheduleSymbol(0, start_symbol);

  while (keep_running.load() == true) {
    gen_tag_t ctag(0);  // The completion tag
    int ret = static_cast<int>(completion_queue_.try_dequeue(ctag.tag_));
    if (ret > 0) {
      const size_t comp_frame_slot = (ctag.frame_id_ % kFrameWnd);
      packet_count_per_symbol_[comp_frame_slot][ctag.symbol_id_]++;

      if (kDebugPrintSender == true) {
        AGORA_LOG_INFO(
            "Sender: Checking symbol %d : %zu : %zu\n", ctag.symbol_id_,
            comp_frame_slot,
            packet_count_per_symbol_[comp_frame_slot][ctag.symbol_id_]);
      }
      // Check to see if the current symbol is finished
      if (packet_count_per_symbol_[comp_frame_slot][ctag.symbol_id_] ==
          cfg_->BsAntNum()) {
        // Finished with the current symbol
        packet_count_per_symbol_[comp_frame_slot][ctag.symbol_id_] = 0;

        size_t next_symbol_id = FindNextSymbol((ctag.symbol_id_ + 1));
        unsigned symbol_delay = next_symbol_id - ctag.symbol_id_;
        if (kDebugPrintSender) {
          AGORA_LOG_INFO(
              "Sender: Finishing symbol %d, Next Symbol: %zu, Total Symbols: "
              "%zu, delaying %d\n",
              ctag.symbol_id_, next_symbol_id, cfg_->Frame().NumTotalSyms(),
              symbol_delay);
        }
        // Add inter-symbol delay
        DelayTicks(tick_start, GetTicksForFrame(ctag.frame_id_) * symbol_delay);
        tick_start += (GetTicksForFrame(ctag.frame_id_) * symbol_delay);

        size_t next_frame_id = ctag.frame_id_;
        // Check to see if the current frame is finished
        assert(next_symbol_id <= cfg_->Frame().NumTotalSyms());
        if (next_symbol_id == cfg_->Frame().NumTotalSyms()) {
          // Set the end time to time to when the last symbol was completed by
          // the workers (now)
          frame_end_us = GetTime::GetTimeUs();
          next_frame_id++;

          // Find start symbol of next frame and add proper delay
          next_symbol_id = FindNextSymbol(0);
          if ((kDebugSenderReceiver == true) ||
              (kDebugPrintPerFrameDone == true)) {
            AGORA_LOG_INFO(
                "Sender: Tx frame %d in %.2f ms, next frame %zu, start symbol "
                "%zu\n",
                ctag.frame_id_, (frame_end_us - frame_start_us) / 1000.0,
                next_frame_id, next_symbol_id);
          }
          // Set end of frame time to the time after the last symbol
          frame_end_[(ctag.frame_id_ % kNumStatsFrames)] = frame_end_us;

          if (next_frame_id == cfg_->FramesToTest()) {
            keep_running.store(false);
            break; /* Finished */
          } else {
            // Wait for the inter-frame delay
            DelayTicks(tick_start, ticks_inter_frame_);
            tick_start += ticks_inter_frame_;
            frame_start_us = GetTime::GetTimeUs();

            // Set the frame start time to the start time of the frame
            // (independant of symbol type)
            frame_start_[(next_frame_id % kNumStatsFrames)] = frame_start_us;

            // Wait until the first tx symbol
            DelayTicks(tick_start,
                       (GetTicksForFrame(ctag.frame_id_) * next_symbol_id));
            tick_start += (GetTicksForFrame(ctag.frame_id_) * next_symbol_id);
          }
        }  // if (next_symbol_id == cfg_->Frame().NumTotalSyms()) {
        ScheduleSymbol(next_frame_id, next_symbol_id);
      }
    }  // end (ret > 0)
  }
  AGORA_LOG_INFO("Sender main thread exit\n");
  WriteStatsToFile(cfg_->FramesToTest());
  return nullptr;
}

/* Worker expects only valid transmit symbol_ids 'U' 'P' */
void* Sender::WorkerThread(int tid) {
  PinToCoreWithOffset(ThreadType::kWorkerTX, (core_offset_ + 1), tid);

  // Wait for all Sender threads (including master) to start runnung
  num_workers_ready_atomic.fetch_add(1);
  while (num_workers_ready_atomic.load() < (socket_thread_num_ + 1)) {
    // Wait
  }

  const size_t max_symbol_id =
      cfg_->Frame().NumPilotSyms() +
      cfg_->Frame().NumULSyms();  // TEMP not sure if this is ok

  size_t radios_per_worker = (cfg_->NumRadios() / socket_thread_num_);
  if ((cfg_->NumRadios() % socket_thread_num_) != 0) {
    radios_per_worker++;
  }

  const size_t radio_lo = tid * radios_per_worker;
  //This thread has nothing to do
  if (radio_lo >= cfg_->NumRadios()) {
    return nullptr;
  }
  const size_t radio_hi =
      std::min(radio_lo + radios_per_worker, cfg_->NumRadios()) - 1;
  const size_t radios_this_worker = ((radio_hi - radio_lo) + 1);
  const size_t ant_num_this_thread = radios_this_worker * cfg_->NumChannels();

  AGORA_LOG_INFO(
      "Sender worker[%d]: emulating radios %zu:%zu total radios handled by "
      "this worker %zu\n",
      tid, radio_lo, radio_hi, radios_this_worker);

  DFTI_DESCRIPTOR_HANDLE mkl_handle;
  DftiCreateDescriptor(&mkl_handle, DFTI_SINGLE, DFTI_COMPLEX, 1,
                       cfg_->OfdmCaNum());
  DftiCommitDescriptor(mkl_handle);

#if defined(USE_DPDK)
  uint16_t port_id = port_ids_.at(tid % cfg_->DpdkNumPorts());
  AGORA_LOG_INFO("Sender worker[%d]: using port %u\n", tid, port_id);
  std::array<rte_mbuf*, kDequeueBulkSize> tx_mbufs;
#else
  // Make a client / socket for each interface (simular to radio behavior)
  std::vector<std::unique_ptr<UDPClient> > udp_clients;
  //Setting up the source port.  Each radio has a unique source port id
  for (size_t radio_number = radio_lo; radio_number <= radio_hi;
       radio_number++) {
    udp_clients.emplace_back(std::make_unique<UDPClient>(
        cfg_->BsRruAddr(), cfg_->BsRruPort() + radio_number));
  }
#endif

  auto* fft_inout =
      static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
          Agora_memory::Alignment_t::kAlign64,
          cfg_->OfdmCaNum() * sizeof(complex_float)));
  auto* socks_pkt_buf = static_cast<Packet*>(PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign32, cfg_->PacketLength()));

  double begin = GetTime::GetTimeUs();
  size_t total_tx_packets = 0;
  size_t total_tx_packets_rolling = 0;
  size_t cur_radio = radio_lo;

  AGORA_LOG_INFO("Sender worker[%d]: %zu antennas, total bs antennas: %zu\n",
                 tid, ant_num_this_thread, cfg_->BsAntNum());

  // We currently don't support zero-padding OFDM prefix and postfix
  RtAssert(cfg_->PacketLength() ==
           Packet::kOffsetOfData +
               (kUse12BitIQ ? 3 : 4) * (cfg_->SampsPerSymbol()));
  const size_t ant_num_per_cell = cfg_->BsAntNum() / cfg_->NumCells();

  size_t tags[kDequeueBulkSize];
  while (keep_running.load() == true) {
    size_t num_tags = send_queue_.try_dequeue_bulk_from_producer(
        *(task_ptok_[tid]), tags, kDequeueBulkSize);
    if (num_tags > 0) {
      for (size_t tag_id = 0; (tag_id < num_tags); tag_id++) {
        const size_t start_tsc_send = GetTime::Rdtsc();
        const auto tag = gen_tag_t(tags[tag_id]);
        assert((cfg_->GetSymbolType(tag.symbol_id_) == SymbolType::kPilot) ||
               (cfg_->GetSymbolType(tag.symbol_id_) == SymbolType::kUL));

        // Send a message to the server. We assume that the server is running.
        Packet* pkt = nullptr;
#if defined(USE_DPDK)
        tx_mbufs.at(tag_id) = DpdkTransport::AllocUdp(
            mbuf_pool_, sender_mac_addr_[port_id], server_mac_addr_[port_id],
            bs_rru_addr_, bs_server_addr_, cfg_->BsRruPort() + cur_radio,
            cfg_->BsServerPort() + cur_radio, cfg_->PacketLength(),
            (uint16_t(tag.frame_id_ & 0xffff) << 8) |
                uint16_t(tag.symbol_id_ & 0xffff));
        pkt = reinterpret_cast<Packet*>(
            rte_pktmbuf_mtod(tx_mbufs.at(tag_id), uint8_t*) + kPayloadOffset);
#else
        pkt = socks_pkt_buf;
#endif

        if (kDebugPrintSender) {
          AGORA_LOG_INFO(
              "Sender worker [%d]: processing frame %d symbol %d, type %d\n",
              tid, tag.frame_id_, tag.symbol_id_,
              static_cast<int>(cfg_->GetSymbolType(tag.symbol_id_)));
        }

        // Update the TX buffer
        pkt->frame_id_ = tag.frame_id_;
        pkt->symbol_id_ = tag.symbol_id_;
        pkt->cell_id_ = tag.ant_id_ / ant_num_per_cell;
        pkt->ant_id_ = tag.ant_id_ - ant_num_per_cell * (pkt->cell_id_);
        std::memcpy(
            pkt->data_,
            iq_data_short_[(pkt->symbol_id_ * cfg_->BsAntNum()) + tag.ant_id_],
            (cfg_->SampsPerSymbol()) * (kUse12BitIQ ? 3 : 4));
        if (cfg_->FftInRru() == true) {
          RunFft(pkt, fft_inout, mkl_handle);
        }

        const size_t dest_port = cfg_->BsServerPort() + cur_radio;

#if (defined(USE_DPDK) && !defined(DPDK_BURST_BULK))
        const size_t queue_id =
            cur_radio % (cfg_->NumRadios() / cfg_->DpdkNumPorts());
        const size_t nb_tx_new =
            rte_eth_tx_burst(port_id, queue_id, &tx_mbufs.at(tag_id), 1);
        if (unlikely(nb_tx_new != 1)) {
          AGORA_LOG_ERROR(
              "Thread %d rte_eth_tx_burst() failed, nb_tx_new: %zu, \n", tid,
              nb_tx_new, 1);
          keep_running.store(false);
          break;
        }
#elif (!defined(USE_DPDK))
        const size_t interface_idx = cur_radio - radio_lo;
        udp_clients.at(interface_idx)
            ->Send(cfg_->BsServerAddr(), dest_port,
                   reinterpret_cast<std::byte*>(socks_pkt_buf),
                   cfg_->PacketLength());
#endif

        if (kDebugSenderReceiver) {
          AGORA_LOG_INFO(
              "Thread %d (tag = %s) transmit frame %d, symbol %d, ant %d, size "
              "%zu, dest port %zu, TX time: %.3f us\n",
              tid, gen_tag_t(tag).ToString().c_str(), pkt->frame_id_,
              pkt->symbol_id_, pkt->ant_id_, cfg_->PacketLength(), dest_port,
              GetTime::CyclesToUs(GetTime::Rdtsc() - start_tsc_send,
                                  freq_ghz_));
        }

        total_tx_packets_rolling++;
        total_tx_packets++;
        if (total_tx_packets_rolling ==
            ant_num_this_thread * max_symbol_id * 1000) {
          const double end = GetTime::GetTimeUs();
          const double byte_len = cfg_->PacketLength() * ant_num_this_thread *
                                  max_symbol_id * 1000.f;
          const double diff = end - begin;
          AGORA_LOG_INFO(
              "Thread %zu send %zu frames in %f secs, tput %f Mbps\n",
              (size_t)tid,
              total_tx_packets / (ant_num_this_thread * max_symbol_id),
              diff / 1e6, byte_len * 8 * 1e6 / diff / 1024 / 1024);
          begin = GetTime::GetTimeUs();
          total_tx_packets_rolling = 0;
        }

        if (cur_radio == radio_hi) {
          cur_radio = radio_lo;
        } else {
          cur_radio++;
        }
      }

#if (defined(USE_DPDK) && defined(DPDK_BURST_BULK))
      const size_t queue_id =
          cur_radio % (cfg_->NumRadios() / cfg_->DpdkNumPorts());
      //queue id might be a little abused here
      AGORA_LOG_TRACE("Thread %d rte_eth_tx_burst(), queue %zu num_tags: %zu\n",
                      tid, queue_id, num_tags);
      const size_t nb_tx_new =
          rte_eth_tx_burst(port_id, queue_id, tx_mbufs.data(), num_tags);
      if (unlikely(nb_tx_new != num_tags)) {
        AGORA_LOG_ERROR(
            "Thread %d rte_eth_tx_burst() failed, nb_tx_new: %zu, num_tags: "
            "%zu\n",
            tid, nb_tx_new, num_tags);
        keep_running.store(false);
        break;
      }
#endif
      RtAssert(completion_queue_.enqueue_bulk(tags, num_tags),
               "Completion enqueue failed");
    }  // if (num_tags > 0)
  }    // while (keep_running.load() == true)

  DftiFreeDescriptor(&mkl_handle);

  std::free(static_cast<void*>(socks_pkt_buf));
  std::free(static_cast<void*>(fft_inout));
  AGORA_LOG_FRAME("Sender: worker thread %d exit\n", tid);
  return nullptr;
}

uint64_t Sender::GetTicksForFrame(size_t frame_id) const {
  if (enable_slow_start_ == 0) {
    return ticks_all_;
  } else if (frame_id < kFrameWnd) {
    return ticks_wnd1_;
  } else if (frame_id < (kFrameWnd * 4)) {
    return ticks_wnd2_;
  } else {
    return ticks_all_;
  }
}

void Sender::InitIqFromFile(const std::string& filename) {
  const size_t packets_per_frame =
      cfg_->Frame().NumTotalSyms() * cfg_->BsAntNum();
  iq_data_short_.Calloc(packets_per_frame, (cfg_->SampsPerSymbol()) * 2,
                        Agora_memory::Alignment_t::kAlign64);

  Table<float> iq_data_float;
  iq_data_float.Calloc(packets_per_frame, (cfg_->SampsPerSymbol()) * 2,
                       Agora_memory::Alignment_t::kAlign64);

  FILE* fp = std::fopen(filename.c_str(), "rb");
  RtAssert(fp != nullptr, "Failed to open IQ data file");

  for (size_t i = 0; i < packets_per_frame; i++) {
    const size_t expected_count = (cfg_->SampsPerSymbol()) * 2;
    const size_t actual_count =
        std::fread(iq_data_float[i], sizeof(float), expected_count, fp);
    if (expected_count != actual_count) {
      std::fprintf(
          stderr,
          "Sender: Failed to read IQ data file %s. Packet %zu: expected "
          "%zu I/Q samples but read %zu. Errno %s\n",
          filename.c_str(), i, expected_count, actual_count, strerror(errno));
      throw std::runtime_error("Sender: Failed to read IQ data file");
    }
    if (kUse12BitIQ) {
      // Adapt 32-bit IQ samples to 24-bit to reduce network throughput
      ConvertFloatTo12bitIq(iq_data_float[i],
                            reinterpret_cast<uint8_t*>(iq_data_short_[i]),
                            expected_count);
    } else {
      SimdConvertFloatToShort(iq_data_float[i], iq_data_short_[i],
                              expected_count);
    }
  }
  std::fclose(fp);
  iq_data_float.Free();
}

void Sender::CreateWorkerThreads(size_t num_workers) {
  for (size_t i = 0u; i < num_workers; i++) {
    threads_.emplace_back(&Sender::WorkerThread, this, i);
  }
}

void Sender::WriteStatsToFile(size_t tx_frame_count) const {
  const std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  const std::string filename =
      cur_directory + "/files/experiment/tx_result.txt";
  AGORA_LOG_INFO("Printing sender results to file \"%s\"...\n",
                 filename.c_str());
  FILE* fp_debug = std::fopen(filename.c_str(), "w");
  RtAssert(fp_debug != nullptr, "Failed to open stats file");
  for (size_t i = 0; i < tx_frame_count; i++) {
    std::fprintf(fp_debug, "%.5f\n", frame_end_[i % kNumStatsFrames]);
  }
}

void Sender::RunFft(Packet* pkt, complex_float* fft_inout,
                    DFTI_DESCRIPTOR_HANDLE mkl_handle) const {
  // pkt->data has (cp_len + ofdm_ca_num) unsigned short samples. After FFT,
  // we'll remove the cyclic prefix and have ofdm_ca_num() short samples left.
  SimdConvertShortToFloat(&pkt->data_[2 * cfg_->CpLen()],
                          reinterpret_cast<float*>(fft_inout),
                          cfg_->OfdmCaNum() * 2);

  DftiComputeForward(mkl_handle, reinterpret_cast<float*>(fft_inout));

  SimdConvertFloat32ToFloat16(reinterpret_cast<float*>(pkt->data_),
                              reinterpret_cast<float*>(fft_inout),
                              cfg_->OfdmCaNum() * 2);
}
