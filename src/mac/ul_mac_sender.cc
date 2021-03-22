/**
 * @file ul_mac_sender.cc
 * @brief Implementation file for the simple uplink mac sender class
 */
#include "ul_mac_sender.h"

#include <thread>

#include "datatype_conversion.h"
#include "logger.h"
#include "udp_client.h"

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

inline size_t UlMacSender::TagToTxBuffersIndex(gen_tag_t tag) const {
  const size_t frame_slot = tag.frame_id_ % kFrameWnd;
  return (frame_slot * cfg_->UeAntNum()) + tag.ue_id_;
}

// Only send Uplink (non-pilot data)
// size_t tx_port = cfg_->MacRxPort();

UlMacSender::UlMacSender(Config* cfg, size_t socket_thread_num,
                         size_t core_offset, size_t frame_duration_us,
                         size_t inter_frame_delay, size_t enable_slow_start,
                         bool create_thread_for_master)
    : cfg_(cfg),
      freq_ghz_(GetTime::MeasureRdtscFreq()),
      ticks_per_usec_(freq_ghz_ * 1e3),
      socket_thread_num_(socket_thread_num),
      enable_slow_start_(enable_slow_start),
      core_offset_(core_offset),
      inter_frame_delay_(inter_frame_delay),
      ticks_inter_frame_(inter_frame_delay_ * ticks_per_usec_) {
  if (frame_duration_us == 0) {
    frame_duration_us_ =
        (cfg->Frame().NumTotalSyms() * cfg->SampsPerSymbol() * 1000000ul) /
        cfg->Rate();
  } else {
    frame_duration_us_ = frame_duration_us;
  }

  ticks_all_ =
      ((frame_duration_us * ticks_per_usec_) / cfg->Frame().NumTotalSyms());
  ticks_wnd1_ = ticks_all_ * 40;
  ticks_wnd2_ = ticks_all_ * 15;

  tx_buffers_.Malloc(kFrameWnd * cfg_->UeAntNum(), cfg_->MacPacketLength(),
                     Agora_memory::Alignment_t::kAlign64);

  MLPD_INFO(
      "Initializing sender, sending to mac thread at %s, frame "
      "duration = %.2f ms, slow start = %s\n",
      cfg->BsServerAddr().c_str(), frame_duration_us / 1000.0,
      enable_slow_start == 1 ? "yes" : "no");

  for (auto& i : packet_count_per_symbol_) {
    i = new size_t[cfg->Frame().NumTotalSyms()]();
  }

  // InitIqFromFile(std::string(TOSTRING(PROJECT_DIRECTORY)) +
  //               "/data/LDPC_rx_data_" + std::to_string(cfg->OfdmCaNum()) +
  //               "_ant" + std::to_string(cfg->BsAntNum()) + ".bin");

  task_ptok_ =
      static_cast<moodycamel::ProducerToken**>(Agora_memory::PaddedAlignedAlloc(
          Agora_memory::Alignment_t::kAlign64,
          (socket_thread_num * sizeof(moodycamel::ProducerToken*))));
  for (size_t i = 0; i < socket_thread_num; i++) {
    task_ptok_[i] = new moodycamel::ProducerToken(send_queue_);
  }

  // Create a master thread when started from simulator
  if (create_thread_for_master == true) {
    this->threads_.emplace_back(&UlMacSender::MasterThread, this,
                                socket_thread_num_);
  }
  num_workers_ready_atomic.store(0);
}

UlMacSender::~UlMacSender() {
  keep_running.store(false);

  for (auto& thread : this->threads_) {
    MLPD_INFO("Sender: Joining threads\n");
    thread.join();
  }

  for (auto& i : packet_count_per_symbol_) {
    delete[] i;
  }

  for (size_t i = 0; i < socket_thread_num_; i++) {
    delete (task_ptok_[i]);
  }
  std::free(task_ptok_);
  MLPD_INFO("Sender: Complete\n");
}

void UlMacSender::StartTx() {
  this->frame_start_ = new double[kNumStatsFrames]();
  this->frame_end_ = new double[kNumStatsFrames]();

  CreateWorkerThreads(socket_thread_num_);
  signal(SIGINT, InterruptHandler);
  MasterThread(0);  // Start the master thread

  delete[](this->frame_start_);
  delete[](this->frame_end_);
}

void UlMacSender::StartTXfromMain(double* in_frame_start,
                                  double* in_frame_end) {
  frame_start_ = in_frame_start;
  frame_end_ = in_frame_end;

  CreateWorkerThreads(socket_thread_num_);
}

size_t UlMacSender::FindNextSymbol(size_t start_symbol) {
  size_t next_symbol_id;
  for (next_symbol_id = start_symbol;
       (next_symbol_id < cfg_->Frame().NumTotalSyms()); next_symbol_id++) {
    SymbolType symbol_type = cfg_->GetSymbolType(next_symbol_id);
    // Must be uplink data symbol
    if ((symbol_type == SymbolType::kUL) &&
        (this->cfg_->Frame().GetULSymbolIdx(next_symbol_id) >=
         this->cfg_->Frame().ClientUlPilotSymbols())) {
      break;
    }
  }
  return next_symbol_id;
}

void UlMacSender::ScheduleSymbol(size_t frame, size_t symbol_id) {
  for (size_t i = 0; i < cfg_->UeAntNum(); i++) {
    auto req_tag = gen_tag_t::FrmSymAnt(frame, symbol_id, i);
    // Split up the antennas amoung the worker threads
    RtAssert(
        send_queue_.enqueue(*task_ptok_[i % socket_thread_num_], req_tag.tag_),
        "Send task enqueue failed");
  }
}

void* UlMacSender::MasterThread(size_t /*unused*/) {
  PinToCoreWithOffset(ThreadType::kMasterTX, core_offset_, 0);

  // Wait for all worker threads to be ready (+1 for Master)
  num_workers_ready_atomic.fetch_add(1);
  while (num_workers_ready_atomic.load() < (socket_thread_num_ + 1)) {
    // Wait
  }

  double start_time = GetTime::GetTimeUs();
  this->frame_start_[0] = start_time;
  uint64_t tick_start = GetTime::Rdtsc();

  size_t start_symbol = FindNextSymbol(0);
  // Delay until the start of the first symbol
  if (start_symbol > 0) {
    MLPD_INFO("Sender: Starting symbol %zu delaying\n", start_symbol);
    DelayTicks(tick_start, GetTicksForFrame(0) * start_symbol);
  }
  tick_start = GetTime::Rdtsc();
  RtAssert(start_symbol != cfg_->Frame().NumTotalSyms(),
           "Sender: No valid symbols to transmit");
  ScheduleSymbol(0, start_symbol);

  while (keep_running.load() == true) {
    gen_tag_t ctag(0);  // The completion tag
    int ret = static_cast<int>(completion_queue_.try_dequeue(ctag.tag_));
    if (ret > 0) {
      const size_t comp_frame_slot = (ctag.frame_id_ % kFrameWnd);
      packet_count_per_symbol_.at(comp_frame_slot)[ctag.symbol_id_]++;

      if (kDebugPrintSender == true) {
        std::printf(
            "Sender: Checking symbol %d : %zu : %zu\n", ctag.symbol_id_,
            comp_frame_slot,
            packet_count_per_symbol_.at(comp_frame_slot)[ctag.symbol_id_]);
      }
      // Check to see if the current symbol is finished (UeNum / UeAntNum)
      if (packet_count_per_symbol_.at(comp_frame_slot)[ctag.symbol_id_] ==
          cfg_->UeAntNum()) {
        // Finished with the current symbol
        packet_count_per_symbol_.at(comp_frame_slot)[ctag.symbol_id_] = 0;

        size_t next_symbol_id = FindNextSymbol((ctag.symbol_id_ + 1));
        // Set end of frame time to the time the last symbol was transmitted
        // (now) do this before the delay until the end of the frame
        if (next_symbol_id == cfg_->Frame().NumTotalSyms()) {
          double timeus_now = GetTime::GetTimeUs();
          this->frame_end_[(ctag.frame_id_ % kNumStatsFrames)] = timeus_now;
        }

        unsigned symbol_delay = next_symbol_id - ctag.symbol_id_;
        if (kDebugPrintSender) {
          std::printf(
              "Sender: Finishing symbol %d, Next Symbol: %zu, Total Symbols: "
              "%zu, delaying %d\n",
              ctag.symbol_id_, next_symbol_id, cfg_->Frame().NumTotalSyms(),
              symbol_delay);
        }
        // Add inter-symbol delay
        DelayTicks(tick_start, GetTicksForFrame(ctag.frame_id_) * symbol_delay);
        tick_start = GetTime::Rdtsc();

        size_t next_frame_id = ctag.frame_id_;
        // Check to see if the current frame is finished
        assert(next_symbol_id <= cfg_->Frame().NumTotalSyms());
        if (next_symbol_id == cfg_->Frame().NumTotalSyms()) {
          next_frame_id++;

          // Find start symbol of next frame and add proper delay
          next_symbol_id = FindNextSymbol(0);
          if ((kDebugSenderReceiver == true) ||
              (kDebugPrintPerFrameDone == true)) {
            double timeus_now = GetTime::GetTimeUs();
            std::printf(
                "Sender: Tx frame %d in %.2f ms, next frame %zu, start symbol "
                "%zu\n",
                ctag.frame_id_, (timeus_now - start_time) / 1000.0,
                next_frame_id, next_symbol_id);
            start_time = timeus_now;
          }

          if (next_frame_id == cfg_->FramesToTest()) {
            keep_running.store(false);
            break; /* Finished */
          } else {
            // Wait until the next symbol time, then next frame time
            DelayTicks(tick_start,
                       (GetTicksForFrame(ctag.frame_id_) * next_symbol_id) +
                           ticks_inter_frame_);
            // Set the frame start time to the time of the first tx symbol
            // schedule (now)
            this->frame_start_[(next_frame_id % kNumStatsFrames)] =
                GetTime::GetTimeUs();
          }
        }  // if (next_symbol_id == cfg_->Frame().NumTotalSyms()) {
        tick_start = GetTime::Rdtsc();
        ScheduleSymbol(next_frame_id, next_symbol_id);
      }
    }  // end (ret > 0)
  }
  std::printf("Sender main thread exit\n");
  WriteStatsToFile(cfg_->FramesToTest());
  return nullptr;
}

/* Worker expects only valid transmit symbol_ids 'U' 'P' */
void* UlMacSender::WorkerThread(size_t tid) {
  PinToCoreWithOffset(ThreadType::kWorkerTX, (core_offset_ + 1), tid);

  // Wait for all Sender threads (including master) to start runnung
  num_workers_ready_atomic.fetch_add(1);
  while (num_workers_ready_atomic.load() < (socket_thread_num_ + 1)) {
    // Wait
  }
  MLPD_FRAME("Sender: worker thread %d running\n", tid);

  const size_t max_symbol_id =
      cfg_->Frame().NumULSyms() - cfg_->Frame().ClientUlPilotSymbols();
  const size_t radio_lo = (tid * cfg_->NumRadios()) / socket_thread_num_;
  const size_t radio_hi = ((tid + 1) * cfg_->NumRadios()) / socket_thread_num_;
  const size_t ant_num_this_thread =
      cfg_->NumRadios() / socket_thread_num_ +
      (static_cast<size_t>(tid) < cfg_->NumRadios() % socket_thread_num_ ? 1
                                                                         : 0);
  UDPClient udp_client;

  auto* socks_pkt_buf = static_cast<Packet*>(PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign32, cfg_->PacketLength()));

  double begin = GetTime::GetTimeUs();
  size_t total_tx_packets = 0;
  size_t total_tx_packets_rolling = 0;
  size_t cur_radio = radio_lo;

  MLPD_INFO("Sender: In thread %d, %zu antennas, total antennas: %zu\n", tid,
            ant_num_this_thread, cfg_->NumRadios());

  // We currently don't support zero-padding OFDM prefix and postfix
  RtAssert(cfg_->PacketLength() ==
           Packet::kOffsetOfData +
               (kUse12BitIQ ? 3 : 4) * (cfg_->CpLen() + cfg_->OfdmCaNum()));
  size_t ant_num_per_cell = cfg_->BsAntNum() / cfg_->NumCells();

  std::array<size_t, kDequeueBulkSize> tags;
  while (keep_running.load() == true) {
    size_t num_tags = this->send_queue_.try_dequeue_bulk_from_producer(
        *(this->task_ptok_[tid]), tags.data(), kDequeueBulkSize);
    if (num_tags > 0) {
      for (size_t tag_id = 0; (tag_id < num_tags); tag_id++) {
        size_t start_tsc_send = GetTime::Rdtsc();

        auto tag = gen_tag_t(tags.at(tag_id));
        assert(cfg_->GetSymbolType(tag.symbol_id_) == SymbolType::kUL);

        if ((kDebugPrintSender == true)) {
          std::printf(
              "Sender : worker %zu processing frame %d symbol %d, type %d\n",
              tid, tag.frame_id_, tag.symbol_id_,
              static_cast<int>(cfg_->GetSymbolType(tag.symbol_id_)));
        }
        const size_t tx_bufs_idx = TagToTxBuffersIndex(tag);

        udp_client.Send(cfg_->UeServerAddr(), cfg_->MacRxPort() + cur_radio,
                        tx_buffers_[tx_bufs_idx], cfg_->PacketLength());

        if (kDebugSenderReceiver == true) {
          std::printf(
              "Thread %zu (tag = %s) transmit frame %d, radio "
              "%d, TX time: %.3f us\n",
              tid, gen_tag_t(tag).ToString().c_str(), tag.frame_id_, cur_radio,
              GetTime::CyclesToUs(GetTime::Rdtsc() - start_tsc_send,
                                  freq_ghz_));
        }

        total_tx_packets_rolling++;
        total_tx_packets++;
        if (total_tx_packets_rolling ==
            ant_num_this_thread * max_symbol_id * 1000) {
          double end = GetTime::GetTimeUs();
          double byte_len = cfg_->PacketLength() * ant_num_this_thread *
                            max_symbol_id * 1000.f;
          double diff = end - begin;
          std::printf("Thread %zu send %zu frames in %f secs, tput %f Mbps\n",
                      (size_t)tid,
                      total_tx_packets / (ant_num_this_thread * max_symbol_id),
                      diff / 1e6, byte_len * 8 * 1e6 / diff / 1024 / 1024);
          begin = GetTime::GetTimeUs();
          total_tx_packets_rolling = 0;
        }

        if (++cur_radio == radio_hi) {
          cur_radio = radio_lo;
        }
      }
      RtAssert(completion_queue_.enqueue_bulk(tags.data(), num_tags),
               "Completion enqueue failed");
    }  // if (num_tags > 0)
  }    // while (keep_running.load() == true)

  std::free(static_cast<void*>(socks_pkt_buf));
  MLPD_FRAME("Sender: worker thread %zu exit\n", tid);
  return nullptr;
}

uint64_t UlMacSender::GetTicksForFrame(size_t frame_id) const {
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

/*
void UlMacSender::InitIqFromFile(const std::string& filename) {
  const size_t packets_per_frame =
      cfg_->Frame().NumTotalSyms() * cfg_->BsAntNum();
  iq_data_short_.Calloc(packets_per_frame,
                        (cfg_->CpLen() + cfg_->OfdmCaNum()) * 2,
                        Agora_memory::Alignment_t::kAlign64);

  Table<float> iq_data_float;
  iq_data_float.Calloc(packets_per_frame,
                       (cfg_->CpLen() + cfg_->OfdmCaNum()) * 2,
                       Agora_memory::Alignment_t::kAlign64);

  FILE* fp = std::fopen(filename.c_str(), "rb");
  RtAssert(fp != nullptr, "Failed to open IQ data file");

  for (size_t i = 0; i < packets_per_frame; i++) {
    const size_t expected_count = (cfg_->CpLen() + cfg_->OfdmCaNum()) * 2;
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
      for (size_t j = 0; j < expected_count; j++) {
        iq_data_short_[i][j] =
            static_cast<unsigned short>(iq_data_float[i][j] * 32768);
      }
    }
  }
  std::fclose(fp);
  iq_data_float.Free();
} */

void UlMacSender::CreateWorkerThreads(size_t num_workers) {
  for (size_t i = 0u; i < num_workers; i++) {
    this->threads_.emplace_back(&UlMacSender::WorkerThread, this, i);
  }
}

void* UlMacSender::DataUpdateThread(size_t tid) {
  // Sender get better performance when this thread is not pinned to core
  // PinToCoreWithOffset(ThreadType::kWorker, 13, 0);
  printf("Data update thread running on core %d\n", sched_getcpu());

  while (true) {
    size_t tag = 0;
    if (data_update_queue_.try_dequeue(tag) == true) {
      for (size_t i = 0; i < cfg_->UeAntNum(); i++) {
        auto tag_for_ue = gen_tag_t::FrmSymUe(((gen_tag_t)tag).frame_id_,
                                              ((gen_tag_t)tag).symbol_id_, i);
        UpdateTxBuffer(tag_for_ue);
      }
    }
  }
}

void UlMacSender::UpdateTxBuffer(gen_tag_t tag) {
  auto* pkt = (MacPacket*)(tx_buffers_[TagToTxBuffersIndex(tag)]);
  pkt->frame_id_ = tag.frame_id_;
  pkt->symbol_id_ = tag.symbol_id_;
  pkt->ue_id_ = tag.ue_id_;

  // Read data from file.
  socklen_t addrlen = sizeof(data_addr_[tag.ue_id_]);
  int ret = recvfrom(data_sockets_[tag.ue_id_], (char*)pkt->data_,
                     cfg_->MacDataBytesNumPerframe(), 0,
                     (struct sockaddr*)&data_addr_[tag.ue_id_], &addrlen);
  if (ret == -1) {
    if (errno != EAGAIN) {
      perror("video recv failed");
      exit(0);
    }
  }

  ////
  /// https://stackoverflow.com/questions/12149593/how-can-i-create-an-array-of-random-numbers-in-c
  // std::random_device r;
  ////std::seed_seq seed{ r(), r(), r(), r(), r(), r(), r(), r() };
  // std::seed_seq seed{ 11, 12, 13, 14, 15, 16, 17, 18 };
  // std::mt19937 eng(seed); // a source of random data

  // std::uniform_int_distribution<char> dist;
  // std::vector<char> v(cfg_->mac_data_bytes_num_perframe);

  // generate(begin(v), end(v), bind(dist, eng));
  // memcpy(pkt->data, (char*)v.data(), cfg_->mac_data_bytes_num_perframe);

  // Print MAC packet summary
  printf("sending packet for frame %d, symbol %d, ue %d, bytes %d\n",
         pkt->frame_id_, pkt->symbol_id_, pkt->ue_id_,
         cfg_->mac_data_bytes_num_perframe);
  for (size_t i = 0; i < cfg_->mac_data_bytes_num_perframe; i++)
    printf("%i ", *((uint8_t*)pkt->data_ + i));
  printf("\n");
}

void UlMacSender::WriteStatsToFile(size_t tx_frame_count) const {
  std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  std::string filename = cur_directory + "/data/tx_result.txt";
  std::printf("Printing sender results to file \"%s\"...\n", filename.c_str());
  FILE* fp_debug = std::fopen(filename.c_str(), "w");
  RtAssert(fp_debug != nullptr, "Failed to open stats file");
  for (size_t i = 0; i < tx_frame_count; i++) {
    std::fprintf(fp_debug, "%.5f\n", frame_end_[i % kNumStatsFrames]);
  }
}
