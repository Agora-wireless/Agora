#include "sender.h"

#include <thread>

#include "datatype_conversion.inc"
#include "udp_client.h"

bool keep_running = true;

static constexpr size_t kMacAddrBtyes = 17;

// A spinning barrier to synchronize the start of worker threads
std::atomic<size_t> num_workers_ready_atomic;

void InterruptHandler(int /*unused*/) {
  std::cout << "Will exit..." << std::endl;
  keep_running = false;
}

void DelayTicks(uint64_t start, uint64_t ticks) {
  while ((Rdtsc() - start) < ticks) {
    _mm_pause();
  }
}

Sender::Sender(Config* cfg, size_t socket_thread_num, size_t core_offset,
               size_t frame_duration, size_t enable_slow_start,
               std::string server_mac_addr_str, bool create_thread_for_master)
    : cfg_(cfg),
      freq_ghz_(MeasureRdtscFreq()),
      ticks_per_usec_(freq_ghz_ * 1e3),
      socket_thread_num_(socket_thread_num),
      enable_slow_start_(enable_slow_start),
      core_offset_(core_offset),
      frame_duration_(frame_duration),
      ticks_all_(frame_duration_ * ticks_per_usec_ /
                 cfg->Frame().NumTotalSyms()),
      ticks_wnd_1_(200000 /* 200 ms */ * ticks_per_usec_ /
                   cfg->Frame().NumTotalSyms()),
      ticks_wnd_2_(15 * frame_duration_ * ticks_per_usec_ /
                   cfg->Frame().NumTotalSyms()) {
  std::printf(
      "Initializing sender, sending to base station server at %s, frame "
      "duration = %.2f ms, slow start = %s\n",
      cfg->BsServerAddr().c_str(), frame_duration / 1000.0,
      enable_slow_start == 1 ? "yes" : "no");

  unused(server_mac_addr_str);
  for (size_t i = 0; i < kFrameWnd; i++) {
    packet_count_per_symbol_[i] = new size_t[cfg->Frame().NumTotalSyms()]();
  }

  InitIqFromFile(std::string(TOSTRING(PROJECT_DIRECTORY)) +
                 "/data/LDPC_rx_data_" + std::to_string(cfg->OfdmCaNum()) +
                 "_ant" + std::to_string(cfg->BsAntNum()) + ".bin");

  task_ptok_ =
      static_cast<moodycamel::ProducerToken**>(Agora_memory::PaddedAlignedAlloc(
          Agora_memory::Alignment_t::kAlign64,
          (socket_thread_num * sizeof(moodycamel::ProducerToken*))));
  for (size_t i = 0; i < socket_thread_num; i++) {
    task_ptok_[i] = new moodycamel::ProducerToken(send_queue_);
  }

  // Create a master thread when started from simulator
  if (create_thread_for_master) {
    CreateThreads(PthreadFunWrapper<Sender, &Sender::MasterThread>,
                  socket_thread_num, socket_thread_num + 1);
  }

#ifdef USE_DPDK
  DpdkTransport::dpdk_init(core_offset, socket_thread_num);
  mbuf_pool = DpdkTransport::create_mempool(cfg->packet_length);

  // Parse IP addresses
  int ret = inet_pton(AF_INET, cfg->BsRruAddr().c_str(), &bs_rru_addr);
  rt_assert(ret == 1, "Invalid sender IP address");
  ret = inet_pton(AF_INET, cfg->BsServerAddr().c_str(), &bs_server_addr);
  rt_assert(ret == 1, "Invalid server IP address");

  rt_assert(cfg->dpdk_num_ports <= rte_eth_dev_count_avail(),
            "Invalid number of DPDK ports");

  rt_assert(server_mac_addr_str.length() ==
                (cfg->dpdk_num_ports * (kMacAddrBtyes + 1) - 1),
            "Invalid length of server MAC address");
  sender_mac_addr.resize(cfg->dpdk_num_ports);
  server_mac_addr.resize(cfg->dpdk_num_ports);

  for (uint16_t port_id = 0; port_id < cfg->dpdk_num_ports; port_id++) {
    if (DpdkTransport::nic_init(port_id, mbuf_pool, socket_thread_num,
                                cfg->packet_length) != 0)
      rte_exit(EXIT_FAILURE, "Cannot init port %u\n", port_id);
    // Parse MAC addresses
    ether_addr* parsed_mac = ether_aton(
        server_mac_addr_str.substr(port_id * (kMacAddrBtyes + 1), kMacAddrBtyes)
            .c_str());
    rt_assert(parsed_mac != NULL, "Invalid server mac address");
    std::memcpy(&server_mac_addr[port_id], parsed_mac, sizeof(ether_addr));

    ret = rte_eth_macaddr_get(port_id, &sender_mac_addr[port_id]);
    rt_assert(ret == 0, "Cannot get MAC address of the port");
    std::printf("Number of DPDK cores: %d\n", rte_lcore_count());
  }

#endif
  num_workers_ready_atomic = 0;
}

Sender::~Sender() {
  iq_data_short_.Free();
  for (size_t i = 0; i < kFrameWnd; i++) {
    std::free(packet_count_per_symbol_[i]);
  }
}

void Sender::StartTx() {
  frame_start_ = new double[kNumStatsFrames]();
  frame_end_ = new double[kNumStatsFrames]();

  CreateThreads(PthreadFunWrapper<Sender, &Sender::WorkerThread>, 0,
                socket_thread_num_);
  MasterThread(0);  // Start the master thread
}

void Sender::StartTXfromMain(double* in_frame_start, double* in_frame_end) {
  frame_start_ = in_frame_start;
  frame_end_ = in_frame_end;

  CreateThreads(PthreadFunWrapper<Sender, &Sender::WorkerThread>, 0,
                socket_thread_num_);
}

size_t Sender::FindNextSymbol(size_t  /*frame*/, size_t start_symbol) {
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
  for (size_t i = 0; i < cfg_->BsAntNum(); i++) {
    auto req_tag = gen_tag_t::FrmSymAnt(frame, symbol_id, i);
    RtAssert(
        send_queue_.enqueue(*task_ptok_[i % socket_thread_num_], req_tag.tag_),
        "Send task enqueue failed");
  }
}

void* Sender::MasterThread(int /*unused*/) {
  signal(SIGINT, InterruptHandler);
  PinToCoreWithOffset(ThreadType::kMasterTX, core_offset_, 0);

  // Wait for all worker threads to be ready
  while (num_workers_ready_atomic != socket_thread_num_) {
    // Wait
  }

  frame_start_[0] = GetTime();
  double start_time = GetTime();
  uint64_t tick_start = Rdtsc();

  size_t start_symbol = FindNextSymbol(0, 0);
  // Delay until the start of the first symbol (pilot)
  if (start_symbol > 0) {
    std::printf("Sender: Starting symbol %zu delaying\n", start_symbol);
    DelayTicks(tick_start, GetTicksForFrame(0) * start_symbol);
    tick_start = Rdtsc();
  }
  RtAssert(start_symbol != cfg_->Frame().NumTotalSyms(),
           "Sender: No valid symbols to transmit");
  ScheduleSymbol(0, start_symbol);

  while (keep_running) {
    gen_tag_t ctag(0);  // The completion tag
    int ret = static_cast<int>(completion_queue_.try_dequeue(ctag.tag_));
    if (ret == 0) {
      continue;
    }

    const size_t comp_frame_slot = (ctag.frame_id_ % kFrameWnd);
    packet_count_per_symbol_[comp_frame_slot][ctag.symbol_id_]++;

    // std::printf("Sender -- checking symbol %d : %zu : %zu\n", ctag.symbol_id,
    // comp_frame_slot,
    // packet_count_per_symbol[comp_frame_slot][ctag.symbol_id]); Check to see
    // if the current symbol is finished
    if (packet_count_per_symbol_[comp_frame_slot][ctag.symbol_id_] ==
        cfg_->BsAntNum()) {
      // Finished with the current symbol
      packet_count_per_symbol_[comp_frame_slot][ctag.symbol_id_] = 0;

      size_t next_symbol_id =
          FindNextSymbol(ctag.frame_id_, (ctag.symbol_id_ + 1));
      unsigned symbol_delay = next_symbol_id - ctag.symbol_id_;
      // std::printf("Sender -- finishing symbol %d : %zu : %zu delayed %d\n",
      // ctag.symbol_id, cfg->symbol_num_perframe, next_symbol_id,
      // symbol_delay); Add inter-symbol delay
      DelayTicks(tick_start, GetTicksForFrame(ctag.frame_id_) * symbol_delay);
      tick_start = Rdtsc();

      size_t next_frame_id = ctag.frame_id_;
      // Check to see if the current frame is finished
      assert(next_symbol_id <= cfg_->Frame().NumTotalSyms());
      if (next_symbol_id == cfg_->Frame().NumTotalSyms()) {
        if ((kDebugSenderReceiver == true) ||
            (kDebugPrintPerFrameDone == true)) {
          std::printf("Sender: Transmitted frame %u in %.1f ms\n",
                      ctag.frame_id_, (GetTime() - start_time) / 1000.0);
          start_time = GetTime();
        }
        next_frame_id++;
        if (next_frame_id == cfg_->FramesToTest()) {
          break;  // Finished
        }
        frame_end_[(ctag.frame_id_ % kNumStatsFrames)] = GetTime();
        frame_start_[(next_frame_id % kNumStatsFrames)] = GetTime();
        tick_start = Rdtsc();

        // Find start symbol of next frame and add proper delay
        next_symbol_id = FindNextSymbol(next_frame_id, 0);
        DelayTicks(tick_start,
                   GetTicksForFrame(ctag.frame_id_) * next_symbol_id);
        tick_start = Rdtsc();
        // std::printf("Sender -- finished frame %d, next frame %zu, start
        // symbol %zu, delaying\n", ctag.frame_id, next_frame_id,
        // next_symbol_id,);
      }
      ScheduleSymbol(next_frame_id, next_symbol_id);
    }
  }
  WriteStatsToFile(cfg_->FramesToTest());
  std::exit(0);
  return nullptr;
}

void* Sender::WorkerThread(int tid) {
  PinToCoreWithOffset(ThreadType::kWorkerTX, core_offset_ + 1, tid);

  // Wait for all Sender threads (including master) to start runnung
  num_workers_ready_atomic++;
  while (num_workers_ready_atomic != socket_thread_num_) {
    // Wait
  }

  DFTI_DESCRIPTOR_HANDLE mkl_handle;
  DftiCreateDescriptor(&mkl_handle, DFTI_SINGLE, DFTI_COMPLEX, 1,
                       cfg_->OfdmCaNum());
  DftiCommitDescriptor(mkl_handle);

  const size_t max_symbol_id =
      cfg_->Frame().NumPilotSyms() + cfg_->Frame().NumULSyms();
  const size_t radio_lo = tid * cfg_->NumRadios() / socket_thread_num_;
  const size_t radio_hi = (tid + 1) * cfg_->NumRadios() / socket_thread_num_;
  const size_t ant_num_this_thread =
      cfg_->BsAntNum() / socket_thread_num_ +
      ((size_t)tid < cfg_->BsAntNum() % socket_thread_num_ ? 1 : 0);
#ifdef USE_DPDK
  const size_t port_id = tid % cfg->dpdk_num_ports;
  const size_t queue_id = tid / cfg->dpdk_num_ports;
  rte_mbuf* tx_mbufs[kDequeueBulkSize];
#endif

  UDPClient udp_client;
  auto* fft_inout =
      static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
          Agora_memory::Alignment_t::kAlign64,
          cfg_->OfdmCaNum() * sizeof(complex_float)));
  auto* socks_pkt_buf = static_cast<Packet*>(PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign32, cfg_->PacketLength()));

  double begin = GetTime();
  size_t total_tx_packets = 0;
  size_t total_tx_packets_rolling = 0;
  size_t cur_radio = radio_lo;

  std::printf("In thread %zu, %zu antennas, BS_ANT_NUM: %zu\n", (size_t)tid,
              ant_num_this_thread, cfg_->BsAntNum());

  // We currently don't support zero-padding OFDM prefix and postfix
  RtAssert(cfg_->PacketLength() ==
           Packet::kOffsetOfData +
               (kUse12BitIQ ? 3 : 4) * (cfg_->CpLen() + cfg_->OfdmCaNum()));
  size_t ant_num_per_cell = cfg_->BsAntNum() / cfg_->NumCells();

  size_t tags[kDequeueBulkSize];
  while (true) {
    size_t num_tags = send_queue_.try_dequeue_bulk_from_producer(
        *(task_ptok_[tid]), tags, kDequeueBulkSize);
    if (num_tags == 0) {
      continue;
    }

    for (size_t tag_id = 0; tag_id < num_tags; tag_id++) {
      size_t start_tsc_send = Rdtsc();

      auto tag = gen_tag_t(tags[tag_id]);
      // Worker expects only valid transmit symbol_ids 'U' 'P'
      assert((cfg_->GetSymbolType(tag.symbol_id_) == SymbolType::kPilot) ||
             (cfg_->GetSymbolType(tag.symbol_id_) == SymbolType::kUL));

      // Send a message to the server. We assume that the server is running.
      Packet* pkt = socks_pkt_buf;
#ifdef USE_DPDK
      tx_mbufs[tag_id] = DpdkTransport::alloc_udp(
          mbuf_pool, sender_mac_addr[port_id], server_mac_addr[port_id],
          bs_rru_addr, bs_server_addr, cfg->bs_rru_port + tid,
          cfg->bs_server_port + tid, cfg->packet_length);
      pkt = (Packet*)(rte_pktmbuf_mtod(tx_mbufs[tag_id], uint8_t*) +
                      kPayloadOffset);
#endif

      // Update the TX buffer
      // std::printf("Sender : worker processing symbol %d, %d\n",
      // tag.symbol_id, (int)symbol_type);
      pkt->frame_id_ = tag.frame_id_;
      pkt->symbol_id_ = tag.symbol_id_;
      pkt->cell_id_ = tag.ant_id_ / ant_num_per_cell;
      pkt->ant_id_ = tag.ant_id_ - ant_num_per_cell * (pkt->cell_id_);
      std::memcpy(
          pkt->data_,
          iq_data_short_[(pkt->symbol_id_ * cfg_->BsAntNum()) + tag.ant_id_],
          (cfg_->CpLen() + cfg_->OfdmCaNum()) * (kUse12BitIQ ? 3 : 4));
      if (cfg_->FftInRru()) {
        RunFft(pkt, fft_inout, mkl_handle);
      }

#ifndef USE_DPDK
      udp_client.Send(cfg_->BsServerAddr(), cfg_->BsServerPort() + cur_radio,
                      reinterpret_cast<uint8_t*>(socks_pkt_buf),
                      cfg_->PacketLength());
#endif

      if (kDebugSenderReceiver == true) {
        std::printf(
            "Thread %d (tag = %s) transmit frame %d, symbol %d, ant "
            "%d, "
            "TX time: %.3f us\n",
            tid, gen_tag_t(tag).ToString().c_str(), pkt->frame_id_,
            pkt->symbol_id_, pkt->ant_id_,
            CyclesToUs(Rdtsc() - start_tsc_send, freq_ghz_));
      }

      total_tx_packets_rolling++;
      total_tx_packets++;
      if (total_tx_packets_rolling ==
          ant_num_this_thread * max_symbol_id * 1000) {
        double end = GetTime();
        double byte_len =
            cfg_->PacketLength() * ant_num_this_thread * max_symbol_id * 1000.f;
        double diff = end - begin;
        std::printf("Thread %zu send %zu frames in %f secs, tput %f Mbps\n",
                    (size_t)tid,
                    total_tx_packets / (ant_num_this_thread * max_symbol_id),
                    diff / 1e6, byte_len * 8 * 1e6 / diff / 1024 / 1024);
        begin = GetTime();
        total_tx_packets_rolling = 0;
      }

      if (++cur_radio == radio_hi) {
        cur_radio = radio_lo;
      }
    }

#ifdef USE_DPDK
    size_t nb_tx_new = rte_eth_tx_burst(port_id, queue_id, tx_mbufs, num_tags);
    if (unlikely(nb_tx_new != num_tags)) {
      std::printf(
          "Thread %d rte_eth_tx_burst() failed, nb_tx_new: %zu, "
          "num_tags: %zu\n",
          tid, nb_tx_new, num_tags);
      keep_running = 0;
      break;
    }
#endif
    RtAssert(completion_queue_.enqueue_bulk(tags, num_tags),
             "Completion enqueue failed");
  }
  return nullptr;
}

uint64_t Sender::GetTicksForFrame(size_t frame_id) const {
  if (enable_slow_start_ == 0) {
    return ticks_all_;
  } else if (frame_id < kFrameWnd) {
    return ticks_wnd_1_;
  } else if (frame_id < kFrameWnd * 4) {
    return ticks_wnd_2_;
  } else {
    return ticks_all_;
  }
}

void Sender::InitIqFromFile(std::string filename) {
  const size_t packets_per_frame =
      cfg_->Frame().NumTotalSyms() * cfg_->BsAntNum();
  iq_data_short_.Calloc(packets_per_frame,
                        (cfg_->CpLen() + cfg_->OfdmCaNum()) * 2,
                        Agora_memory::Alignment_t::kAlign64);

  Table<float> iq_data_float;
  iq_data_float.Calloc(packets_per_frame,
                       (cfg_->CpLen() + cfg_->OfdmCaNum()) * 2,
                       Agora_memory::Alignment_t::kAlign64);

  FILE* fp = fopen(filename.c_str(), "rb");
  RtAssert(fp != nullptr, "Failed to open IQ data file");

  for (size_t i = 0; i < packets_per_frame; i++) {
    const size_t expected_count = (cfg_->CpLen() + cfg_->OfdmCaNum()) * 2;
    const size_t actual_count =
        fread(iq_data_float[i], sizeof(float), expected_count, fp);
    if (expected_count != actual_count) {
      std::fprintf(
          stderr,
          "Sender: Failed to read IQ data file %s. Packet %zu: expected "
          "%zu I/Q samples but read %zu. Errno %s\n",
          filename.c_str(), i, expected_count, actual_count, strerror(errno));
      std::exit(-1);
    }
    if (kUse12BitIQ) {
      // Adapt 32-bit IQ samples to 24-bit to reduce network throughput
      ConvertFloatTo12bitIq(iq_data_float[i],
                            reinterpret_cast<uint8_t*>(iq_data_short_[i]),
                            expected_count);
    } else {
      for (size_t j = 0; j < expected_count; j++) {
        iq_data_short_[i][j] =
            static_cast<unsigned short>(iq_data_float[i][j] * 32768);
      }
    }
  }
  fclose(fp);
  iq_data_float.Free();
}

void Sender::CreateThreads(void* (*worker)(void*), int tid_start, int tid_end) {
  int ret;
  for (int i = tid_start; i < tid_end; i++) {
    pthread_t thread;
    auto* context = new EventHandlerContext<Sender>;
    context->obj_ptr_ = this;
    context->id_ = i;
    ret = pthread_create(&thread, NULL, worker, context);
    RtAssert(ret == 0, "pthread_create() failed");
  }
}

void Sender::WriteStatsToFile(size_t tx_frame_count) const {
  std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  std::string filename = cur_directory + "/data/tx_result.txt";
  std::printf("Printing sender results to file \"%s\"...\n", filename.c_str());
  FILE* fp_debug = fopen(filename.c_str(), "w");
  RtAssert(fp_debug != nullptr, "Failed to open stats file");
  for (size_t i = 0; i < tx_frame_count; i++) {
    std::fprintf(fp_debug, "%.5f\n", frame_end_[i % kNumStatsFrames]);
  }
}

void Sender::RunFft(Packet* pkt, complex_float* fft_inout,
                    DFTI_DESCRIPTOR_HANDLE mkl_handle) const {
  // pkt->data has (CP_LEN + OFDM_CA_NUM) unsigned short samples. After FFT,
  // we'll remove the cyclic prefix and have OFDM_CA_NUM short samples left.
  SimdConvertShortToFloat(&pkt->data_[2 * cfg_->CpLen()],
                          reinterpret_cast<float*>(fft_inout),
                          cfg_->OfdmCaNum() * 2);

  DftiComputeForward(mkl_handle, reinterpret_cast<float*>(fft_inout));

  SimdConvertFloat32ToFloat16(reinterpret_cast<float*>(pkt->data_),
                              reinterpret_cast<float*>(fft_inout),
                              cfg_->OfdmCaNum() * 2);
}
