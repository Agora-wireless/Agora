/**
 * @file mac_sender.cc
 * @brief Implementation file for the simple mac sender class
 */
#include "mac_sender.h"

#include <fstream>
#include <ios>
#include <iostream>
#include <thread>

#include "datatype_conversion.h"
#include "logger.h"
#include "udp_client.h"

static constexpr bool kDebugPrintSender = false;
static constexpr size_t kFrameLoadAdvance = 10;
static constexpr size_t kBufferInit = 10;

static_assert(kFrameLoadAdvance >= kBufferInit);
static std::atomic<bool> keep_running(true);
// A spinning barrier to synchronize the start of worker threads
static std::atomic<size_t> num_workers_ready_atomic(0);

void InterruptHandler(int /*unused*/) {
  std::cout << "Will exit..." << std::endl;
  keep_running.store(false);
}

void DelayTicks(uint64_t start, uint64_t ticks) {
  while ((GetTime::Rdtsc() - start) < ticks) {
    _mm_pause();
  }
}

inline size_t MacSender::TagToTxBuffersIndex(gen_tag_t tag) const {
  const size_t frame_slot = (tag.frame_id_ % kFrameWnd);

  return (frame_slot * cfg_->UeAntNum()) + tag.ue_id_;
}

// Only send Uplink (non-pilot data)
// size_t tx_port = server_rx_port_;

MacSender::MacSender(Config* cfg, std::string& data_filename,
                     size_t packets_per_frame, std::string server_address,
                     size_t server_rx_port,
                     std::function<size_t(size_t)> get_data_symbol_id,
                     size_t socket_thread_num, size_t core_offset,
                     size_t frame_duration_us, size_t inter_frame_delay,
                     size_t enable_slow_start, bool create_thread_for_master)
    : cfg_(cfg),
      freq_ghz_(GetTime::MeasureRdtscFreq()),
      ticks_per_usec_(freq_ghz_ * 1e3f),
      socket_thread_num_(socket_thread_num),
      enable_slow_start_(enable_slow_start),
      core_offset_(core_offset),
      inter_frame_delay_(inter_frame_delay),
      ticks_inter_frame_(inter_frame_delay_ * ticks_per_usec_),
      data_filename_(data_filename),
      // end -- Ul / Dl     UE / BS
      packets_per_frame_(packets_per_frame),
      server_address_(server_address),
      server_rx_port_(server_rx_port),
      get_data_symbol_id_(get_data_symbol_id)
// end -- Ul / Dl     UE / BS
{
  if (frame_duration_us == 0) {
    frame_duration_us_ =
        (cfg->Frame().NumTotalSyms() * cfg->SampsPerSymbol() * 1000000ul) /
        cfg->Rate();
  } else {
    frame_duration_us_ = frame_duration_us;
  }

  ticks_all_ =
      ((frame_duration_us_ * ticks_per_usec_) / cfg->Frame().NumTotalSyms());
  ticks_wnd1_ = ticks_all_ * 40;
  ticks_wnd2_ = ticks_all_ * 15;

  // tx buffers will be an array of MacPackets
  tx_buffers_.Malloc(kFrameWnd * cfg_->UeAntNum(),
                     (packets_per_frame_ *
                      (cfg_->MacPacketLength() + MacPacket::kOffsetOfData)),
                     Agora_memory::Alignment_t::kAlign64);
  MLPD_TRACE(
      "Tx buffer size: dim1 %zu, dim2 %zu, total %zu, start %zu, end: %zu\n",
      (kFrameWnd * cfg_->UeAntNum()),
      (packets_per_frame_ *
       (cfg_->MacPacketLength() + MacPacket::kOffsetOfData)),
      (kFrameWnd * cfg_->UeAntNum()) *
          (packets_per_frame_ *
           (cfg_->MacPacketLength() + MacPacket::kOffsetOfData)),
      (size_t)tx_buffers_[0],
      (size_t)tx_buffers_[(kFrameWnd * cfg_->UeAntNum()) - 1]);

  MLPD_INFO(
      "Initializing MacSender, sending to mac thread at %s:%zu, frame "
      "duration = %.2f ms, slow start = %s\n",
      server_address_.c_str(), server_rx_port_, frame_duration_us_ / 1000.0,
      enable_slow_start == 1 ? "yes" : "no");

  task_ptok_ =
      static_cast<moodycamel::ProducerToken**>(Agora_memory::PaddedAlignedAlloc(
          Agora_memory::Alignment_t::kAlign64,
          (socket_thread_num * sizeof(moodycamel::ProducerToken*))));
  for (size_t i = 0; i < socket_thread_num; i++) {
    task_ptok_[i] = new moodycamel::ProducerToken(send_queue_);
  }

  num_workers_ready_atomic.store(0);
  // Create a master thread when started from simulator
  if (create_thread_for_master == true) {
    MLPD_INFO("MacSender: creating master thread\n");
    this->threads_.emplace_back(&MacSender::MasterThread, this,
                                socket_thread_num_);
  }

  // Add the data update thread (background data reader)
  this->threads_.emplace_back(&MacSender::DataUpdateThread, this, 0);
}

MacSender::~MacSender() {
  keep_running.store(false);

  for (auto& thread : this->threads_) {
    MLPD_INFO("MacSender: Joining threads\n");
    thread.join();
  }

  for (size_t i = 0; i < socket_thread_num_; i++) {
    delete (task_ptok_[i]);
  }
  std::free(task_ptok_);
  tx_buffers_.Free();
  MLPD_INFO("MacSender: Complete\n");
}

void MacSender::StartTx() {
  this->frame_start_ = new double[kNumStatsFrames]();
  this->frame_end_ = new double[kNumStatsFrames]();

  CreateWorkerThreads(socket_thread_num_);
  signal(SIGINT, InterruptHandler);
  MasterThread(0);  // Start the master thread

  delete[](this->frame_start_);
  delete[](this->frame_end_);
}

void MacSender::StartTXfromMain(double* in_frame_start, double* in_frame_end) {
  frame_start_ = in_frame_start;
  frame_end_ = in_frame_end;

  CreateWorkerThreads(socket_thread_num_);
}

void MacSender::LoadFrame(size_t frame) {
  auto req_tag_for_data = gen_tag_t::FrmSym(frame, 0);
  data_update_queue_.try_enqueue(req_tag_for_data.tag_);
}

void MacSender::ScheduleFrame(size_t frame) {
  for (size_t i = 0; i < cfg_->UeAntNum(); i++) {
    auto req_tag = gen_tag_t::FrmSymAnt(frame, 0, i);
    // Split up the antennas amoung the worker threads
    RtAssert(
        send_queue_.enqueue(*task_ptok_[i % socket_thread_num_], req_tag.tag_),
        "Send task enqueue failed");
  }
}

void* MacSender::MasterThread(size_t /*unused*/) {
  PinToCoreWithOffset(ThreadType::kMasterTX, core_offset_, 0);
  std::array<size_t, kFrameWnd> frame_data_count;
  frame_data_count.fill(0);

  // Wait for all worker threads to be ready (+1 for Master)
  for (size_t i = 0; i < kFrameLoadAdvance; i++) {
    LoadFrame(i);
  }

  num_workers_ready_atomic.fetch_add(1);
  while (num_workers_ready_atomic.load() < (socket_thread_num_ + 2)) {
    // Wait
  }
  MLPD_FRAME("MacSender: Master thread running\n");

  RtAssert(packets_per_frame_ > 0, "MacSender: No valid symbols to transmit");

  double timestamp_us = GetTime::GetTimeUs();
  uint64_t tick_start = GetTime::Rdtsc();
  double frame_start_us = timestamp_us;
  double frame_end_us = 0;
  this->frame_start_[0] =
      timestamp_us -
      ((cfg_->Frame().NumTotalSyms() * GetTicksForFrame(0)) / ticks_per_usec_);

  ScheduleFrame(0);
  LoadFrame(0 + kFrameLoadAdvance);

  while (keep_running.load() == true) {
    gen_tag_t ctag(0);  // The completion tag
    int ret = static_cast<int>(completion_queue_.try_dequeue(ctag.tag_));
    if (ret > 0) {
      const size_t comp_frame_slot = (ctag.frame_id_ % kFrameWnd);
      frame_data_count.at(comp_frame_slot)++;

      if (kDebugPrintSender) {
        std::printf("MacSender: Checking frame %d : %zu : %zu\n",
                    ctag.frame_id_, comp_frame_slot,
                    frame_data_count.at(comp_frame_slot));
      }
      // Check to see if the current frame is finished (UeNum / UeAntNum)
      if (frame_data_count.at(comp_frame_slot) == cfg_->UeAntNum()) {
        frame_end_us = timestamp_us;
        // Finished with the current frame data
        frame_data_count.at(comp_frame_slot) = 0;

        size_t next_frame_id = ctag.frame_id_ + 1;
        if ((kDebugSenderReceiver == true) ||
            (kDebugPrintPerFrameDone == true)) {
          std::printf("MacSender: Tx frame %d in %.2f ms, next frame %zu\n",
                      ctag.frame_id_, (frame_end_us - frame_start_us) / 1000.0,
                      next_frame_id);
        }
        this->frame_end_[(ctag.frame_id_ % kNumStatsFrames)] = frame_end_us;

        if (next_frame_id == cfg_->FramesToTest()) {
          keep_running.store(false);
          break; /* Finished */
        } else {
          // Wait the interframe tick time
          DelayTicks(tick_start, ticks_inter_frame_);
          tick_start += ticks_inter_frame_;
          frame_start_us = GetTime::GetTimeUs();
          // Set the frame start time to the start of next frame (now)
          this->frame_start_[(next_frame_id % kNumStatsFrames)] =
              frame_start_us;
          // Wait frame ticks to send the next frame
          DelayTicks(tick_start, GetTicksForFrame(ctag.frame_id_) *
                                     cfg_->Frame().NumTotalSyms());
          tick_start +=
              (GetTicksForFrame(ctag.frame_id_) * cfg_->Frame().NumTotalSyms());
          // Save the scheduled time to apply for the end of the frame
          timestamp_us = GetTime::GetTimeUs();
          ScheduleFrame(next_frame_id);
          LoadFrame(next_frame_id + kFrameLoadAdvance);
        }
      }
    }  // end (ret > 0)
  }
  MLPD_INFO("MacSender: main thread exit\n");
  WriteStatsToFile(cfg_->FramesToTest());
  return nullptr;
}

/* Worker will send 1 MacPacket data size to the radio)
 * since we are using UDP between the antennas and in the multiple antenna case
 * the packet data could get mixed at the mac_thread receiver
 */
void* MacSender::WorkerThread(size_t tid) {
  PinToCoreWithOffset(ThreadType::kWorkerTX, (core_offset_ + 2), tid);

  // Wait for all Sender threads (including master) to start runnung
  num_workers_ready_atomic.fetch_add(1);
  while (num_workers_ready_atomic.load() < (socket_thread_num_ + 2)) {
    // Wait
  }
  MLPD_FRAME("MacSender: worker thread %d running\n", tid);

  const size_t max_symbol_id = 1;
  const size_t radio_lo = (tid * cfg_->NumRadios()) / socket_thread_num_;
  const size_t radio_hi = ((tid + 1) * cfg_->NumRadios()) / socket_thread_num_;
  const size_t ant_num_this_thread =
      cfg_->NumRadios() / socket_thread_num_ +
      (static_cast<size_t>(tid) < cfg_->NumRadios() % socket_thread_num_ ? 1
                                                                         : 0);
  UDPClient udp_client;

  double begin = GetTime::GetTimeUs();
  size_t total_tx_packets = 0;
  size_t total_tx_packets_rolling = 0;
  size_t cur_radio = radio_lo;

  MLPD_INFO("MacSender: In thread %zu, %zu antennas, total antennas: %zu\n",
            tid, ant_num_this_thread, cfg_->NumRadios());

  std::array<size_t, kDequeueBulkSize> tags;
  while (keep_running.load() == true) {
    size_t num_tags = this->send_queue_.try_dequeue_bulk_from_producer(
        *(this->task_ptok_[tid]), tags.data(), kDequeueBulkSize);
    if (num_tags > 0) {
      for (size_t tag_id = 0; (tag_id < num_tags); tag_id++) {
        size_t start_tsc_send = GetTime::Rdtsc();

        auto tag = gen_tag_t(tags.at(tag_id));

        if ((kDebugPrintSender)) {
          std::printf("MacSender : worker %zu processing frame %d : %d \n", tid,
                      tag.frame_id_, tag.ant_id_);
        }
        // const size_t tx_bufs_idx = TagToTxBuffersIndex(tag);
        uint8_t* mac_packet_location = tx_buffers_[TagToTxBuffersIndex(tag)];

        // Mac Thread is currently looking for packets_per_frame_ bytes
        // since we read in mac packets, we need to skip over the headers
        for (size_t packet = 0; packet < packets_per_frame_; packet++) {
          auto* tx_packet = reinterpret_cast<MacPacket*>(mac_packet_location);
          // TODO: Port + ue_radio?
          udp_client.Send(server_address_, server_rx_port_,
                          reinterpret_cast<uint8_t*>(&tx_packet->data_[0u]),
                          cfg_->MacPayloadLength());
          mac_packet_location =
              mac_packet_location +
              (cfg_->MacPacketLength() + MacPacket::kOffsetOfData);
        }

        if (kDebugSenderReceiver) {
          std::printf(
              "Thread %zu (tag = %s) transmit frame %d, radio %zu, TX time: "
              "%.3f us\n",
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
  MLPD_FRAME("MacSender: worker thread %zu exit\n", tid);
  return nullptr;
}

uint64_t MacSender::GetTicksForFrame(size_t frame_id) const {
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

void MacSender::CreateWorkerThreads(size_t num_workers) {
  for (size_t i = 0u; i < num_workers; i++) {
    this->threads_.emplace_back(&MacSender::WorkerThread, this, i);
  }
}

/* Single threaded file reader to load a shared data structure */
void* MacSender::DataUpdateThread(size_t tid) {
  size_t buffer_updates = 0;
  PinToCoreWithOffset(ThreadType::kWorkerMacTXRX, core_offset_ + 1, 0);

  // Sender gets better performance when this thread is not pinned to core
  // PinToCoreWithOffset(ThreadType::kWorker, 13, 0);
  MLPD_INFO("MacSender: Data update thread %zu running on core %d\n", tid,
            sched_getcpu());
  std::ifstream tx_file;
  tx_file.open(data_filename_, (std::ifstream::in | std::ifstream::binary));
  assert(tx_file.is_open() == true);

  // Init the data buffers
  while ((keep_running.load() == true) && (buffer_updates < kBufferInit)) {
    size_t tag = 0;
    if (data_update_queue_.try_dequeue(tag) == true) {
      for (size_t i = 0; i < cfg_->UeAntNum(); i++) {
        auto tag_for_ue = gen_tag_t::FrmSymUe(((gen_tag_t)tag).frame_id_,
                                              ((gen_tag_t)tag).symbol_id_, i);
        UpdateTxBuffer(tx_file, tag_for_ue);
      }
      buffer_updates++;
    }
  }

  MLPD_INFO("MacSender: Data update initialized\n");
  // Unlock the rest of the workers
  num_workers_ready_atomic.fetch_add(1);
  // Normal run loop
  while (keep_running.load() == true) {
    size_t tag = 0;
    if (data_update_queue_.try_dequeue(tag) == true) {
      for (size_t i = 0; i < cfg_->UeAntNum(); i++) {
        auto tag_for_ue = gen_tag_t::FrmSymUe(((gen_tag_t)tag).frame_id_,
                                              ((gen_tag_t)tag).symbol_id_, i);
        UpdateTxBuffer(tx_file, tag_for_ue);
      }
    }
  }
  tx_file.close();
  return nullptr;
}

void MacSender::UpdateTxBuffer(std::ifstream& read, gen_tag_t tag) {
  // Load a frames worth of data
  uint8_t* mac_packet_location = tx_buffers_[TagToTxBuffersIndex(tag)];

  assert(read.is_open() == true);

  for (size_t i = 0; i < packets_per_frame_; i++) {
    auto* pkt = reinterpret_cast<MacPacket*>(mac_packet_location);
    pkt->frame_id_ = tag.frame_id_;
    pkt->symbol_id_ = get_data_symbol_id_(i);
    pkt->ue_id_ = tag.ue_id_;

    // Read a MacPayload into the data section
    read.read(pkt->data_, cfg_->MacPayloadLength());
    // Check for eof
    if (read.eof()) {
      std::printf(
          "MacSender: ***EndofFileStream - Frame %d, symbol %d, ue %d, bytes "
          "%zu\n",
          pkt->frame_id_, pkt->symbol_id_, pkt->ue_id_, read.gcount());
      read.close();
      read.open(data_filename_, std::ifstream::in | std::ifstream::binary);
    }
    // TODO MacPacketLength should be the size of the mac packet but is not.
    mac_packet_location = mac_packet_location +
                          (cfg_->MacPacketLength() + MacPacket::kOffsetOfData);
  }
  std::printf("MacSender: Loading packet for frame %d, ue %d, bytes %zu\n",
              tag.frame_id_, tag.ant_id_,
              cfg_->MacPayloadLength() * packets_per_frame_);
}

void MacSender::WriteStatsToFile(size_t tx_frame_count) const {
  std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  std::string filename = cur_directory + "/data/tx_result.txt";
  std::printf("Printing sender results to file \"%s\"...\n", filename.c_str());

  std::ofstream debug_file;
  debug_file.open(filename, std::ifstream::out);
  debug_file.setf(std::ios::fixed, std::ios::floatfield);
  debug_file.precision(2);

  RtAssert(debug_file.is_open() == true, "Failed to open stats file");
  for (size_t i = 0; i < tx_frame_count; i++) {
    debug_file << "Frame " << i
               << " Start: " << frame_start_[i % kNumStatsFrames]
               << " End: " << frame_end_[i % kNumStatsFrames] << " Time: "
               << (frame_end_[i % kNumStatsFrames] -
                   frame_start_[i % kNumStatsFrames])
               << std::endl;
  }
}
