/**
 * @file mac_sender.cc
 * @brief Implementation file for the simple mac sender class
 */
#include "mac_sender.h"

#include <csignal>
#include <thread>
#include <utility>

#include "datatype_conversion.h"
#include "file_receiver.h"
#include "gettime.h"
#include "logger.h"
#include "message.h"
#include "udp_client.h"
#include "video_receiver.h"

static const std::string kMacSendFromAddress = "127.0.0.1";
static constexpr uint16_t kMacSendFromPort = 0;

//#define USE_UDP_DATA_SOURCE
static constexpr bool kDebugPrintSender = false;
static constexpr size_t kFrameLoadAdvance = 10;
static constexpr size_t kBufferInit = 10;
static constexpr size_t kTxBufferElementAlignment = 64;

static constexpr size_t kSlowStartThresh1 = kFrameWnd;
static constexpr size_t kSlowStartThresh2 = (kFrameWnd * 4);
static constexpr size_t kSlowStartMulStage1 = 32;
static constexpr size_t kSlowStartMulStage2 = 8;
static constexpr size_t kMasterThreadId = 0;

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

MacSender::MacSender(Config* cfg, std::string& data_filename,
                     size_t mac_packet_length, size_t mac_payload_max_length,
                     size_t packets_per_frame, std::string server_address,
                     size_t server_rx_port,
                     std::function<size_t(size_t)> get_data_symbol_id,
                     size_t core_offset, size_t worker_thread_num,
                     size_t update_thread_num, size_t frame_duration_us,
                     size_t inter_frame_delay, size_t enable_slow_start,
                     bool create_thread_for_master)
    : cfg_(cfg),
      freq_ghz_(GetTime::MeasureRdtscFreq()),
      ticks_per_usec_(freq_ghz_ * 1e3f),
      worker_thread_num_(worker_thread_num),
      update_thread_num_(update_thread_num),
      enable_slow_start_(enable_slow_start),
      core_offset_(core_offset),
      inter_frame_delay_(inter_frame_delay),
      ticks_inter_frame_(inter_frame_delay_ * ticks_per_usec_),
      data_filename_(data_filename),
      // end -- Ul / Dl     UE / BS
      mac_packet_length_(mac_packet_length),
      mac_payload_max_length_(mac_payload_max_length),
      packets_per_frame_(packets_per_frame),
      server_address_(std::move(server_address)),
      server_rx_port_(server_rx_port),
      get_data_symbol_id_(std::move(get_data_symbol_id)),
      // end -- Ul / Dl     UE / BS
      has_master_thread_(create_thread_for_master) {
  if (frame_duration_us == 0) {
    frame_duration_us_ =
        (cfg->Frame().NumTotalSyms() * cfg->SampsPerSymbol() * 1000000ul) /
        cfg->Rate();
  } else {
    frame_duration_us_ = frame_duration_us;
  }

  ticks_all_ = static_cast<uint64_t>(
      ((frame_duration_us_ * ticks_per_usec_) / cfg->Frame().NumTotalSyms()));

  const uint64_t two_hundred_ms_ticks = static_cast<uint64_t>(
      (ticks_per_usec_ * 200000.0f) / cfg->Frame().NumTotalSyms());

  ticks_wnd1_ =
      std::max((ticks_all_ * kSlowStartMulStage1), two_hundred_ms_ticks);
  ticks_wnd2_ = ticks_all_ * kSlowStartMulStage2;

  // Match element alignment with buffer alignment
  const size_t padding = kTxBufferElementAlignment -
                         (mac_packet_length_ % kTxBufferElementAlignment);

  tx_buffer_pkt_offset_ = (mac_packet_length_ + padding);
  assert((tx_buffer_pkt_offset_ % kTxBufferElementAlignment) == 0);

  const size_t tx_packet_storage = (packets_per_frame_ * tx_buffer_pkt_offset_);
  // tx buffers will be an array of
  tx_buffers_.Malloc(kFrameWnd * cfg_->UeAntNum(), tx_packet_storage,
                     Agora_memory::Alignment_t::kAlign64);
  AGORA_LOG_TRACE(
      "Tx buffer size: dim1 %zu, dim2 %zu, total %zu, start %zu, end: %zu\n",
      (kFrameWnd * cfg_->UeAntNum()), tx_packet_storage,
      (kFrameWnd * cfg_->UeAntNum()) * tx_packet_storage,
      (size_t)tx_buffers_[0],
      (size_t)tx_buffers_[(kFrameWnd * cfg_->UeAntNum()) - 1]);

  AGORA_LOG_INFO(
      "Initializing MacSender, sending to mac thread at %s:%zu, frame "
      "duration = %.2f ms, slow start = %s\n",
      server_address_.c_str(), server_rx_port_, frame_duration_us_ / 1000.0,
      enable_slow_start == 1 ? "yes" : "no");

  task_ptok_ =
      static_cast<moodycamel::ProducerToken**>(Agora_memory::PaddedAlignedAlloc(
          Agora_memory::Alignment_t::kAlign64,
          (worker_thread_num_ * sizeof(moodycamel::ProducerToken*))));
  for (size_t i = 0; i < worker_thread_num_; i++) {
    task_ptok_[i] = new moodycamel::ProducerToken(send_queue_);
  }

  AGORA_LOG_TRACE("MacSender: Data update thread count: %zu\n",
                  update_thread_num_);

  // Add the data update thread (background data reader), need to add a variable
  // for the update source number
  static constexpr size_t kUpdateSourcePerThread = 1;
  for (size_t update_threads = 0; update_threads < update_thread_num_;
       update_threads++) {
    data_update_queue_.emplace_back(
        moodycamel::ConcurrentQueue<size_t>(kMessageQueueSize));
    // make producer token here?

    // 1 update/data stream per thread.
    threads_.emplace_back(&MacSender::DataUpdateThread, this, update_threads,
                          kUpdateSourcePerThread);
  }

  num_workers_ready_atomic.store(0);
  // Create a master thread when started from simulator
  if (has_master_thread_) {
    AGORA_LOG_INFO("MacSender: creating master thread\n");
    threads_.emplace_back(&MacSender::MasterThread, this, kMasterThreadId);
  }
}

MacSender::~MacSender() {
  keep_running.store(false);

  for (auto& thread : this->threads_) {
    AGORA_LOG_INFO("MacSender: Joining threads\n");
    thread.join();
  }

  for (size_t i = 0; i < worker_thread_num_; i++) {
    delete (task_ptok_[i]);
  }
  std::free(task_ptok_);
  tx_buffers_.Free();
  AGORA_LOG_INFO("MacSender: Complete\n");
}

void MacSender::StartTx() {
  this->frame_start_ = new double[kNumStatsFrames]();
  this->frame_end_ = new double[kNumStatsFrames]();

  CreateWorkerThreads(worker_thread_num_);
  std::signal(SIGINT, InterruptHandler);
  // Run the master thread (from current thread)
  MasterThread(kMasterThreadId);

  delete[](this->frame_start_);
  delete[](this->frame_end_);
}

void MacSender::StartTxfromMain(double* in_frame_start, double* in_frame_end) {
  frame_start_ = in_frame_start;
  frame_end_ = in_frame_end;

  CreateWorkerThreads(worker_thread_num_);
}

void MacSender::LoadFrame(size_t frame) {
  auto req_tag_for_data = gen_tag_t::FrmSym(frame, 0);
  for (size_t update = 0; update < update_thread_num_; update++) {
    data_update_queue_.at(update).try_enqueue(req_tag_for_data.tag_);
  }
}

void MacSender::ScheduleFrame(size_t frame) {
  //Switch to Enqueue Bulk?
  size_t ant_per_thread = cfg_->UeAntNum() / worker_thread_num_;
  if ((cfg_->UeAntNum() % worker_thread_num_) != 0) {
    ant_per_thread++;
  }

  for (size_t ant_id = 0; ant_id < cfg_->UeAntNum(); ant_id++) {
    const auto req_tag = gen_tag_t::FrmSymAnt(frame, 0, ant_id);
    // Split up the antennas among the worker threads
    RtAssert(
        send_queue_.enqueue(*task_ptok_[ant_id / ant_per_thread], req_tag.tag_),
        "Send task enqueue failed");
  }
}

void* MacSender::MasterThread(size_t tid) {
  const bool allow_core_sharing = has_master_thread_;
  PinToCoreWithOffset(ThreadType::kMasterTX, core_offset_, tid,
                      allow_core_sharing);
  std::array<size_t, kFrameWnd> frame_data_count;
  frame_data_count.fill(0);

  // Wait for all worker threads to be ready (+1 for Master)
  for (size_t i = 0; i < kFrameLoadAdvance; i++) {
    LoadFrame(i);
  }

  num_workers_ready_atomic.fetch_add(1);
  while (num_workers_ready_atomic.load() <
         (worker_thread_num_ + 1 + update_thread_num_)) {
    // Wait
  }
  AGORA_LOG_FRAME("MacSender: Master thread running\n");

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
        AGORA_LOG_INFO("MacSender: Checking frame %d : %zu : %zu\n",
                       ctag.frame_id_, comp_frame_slot,
                       frame_data_count.at(comp_frame_slot));
      }
      // Check to see if the current frame is finished (UeAntNum)
      if (frame_data_count.at(comp_frame_slot) == cfg_->UeAntNum()) {
        frame_end_us = timestamp_us;
        // Finished with the current frame data
        frame_data_count.at(comp_frame_slot) = 0;

        size_t next_frame_id = ctag.frame_id_ + 1;
        if ((kDebugSenderReceiver == true) ||
            (kDebugPrintPerFrameDone == true)) {
          AGORA_LOG_INFO("MacSender: Tx frame %d in %.2f ms, next frame %zu\n",
                         ctag.frame_id_,
                         (frame_end_us - frame_start_us) / 1000.0,
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
          uint64_t frame_ticks =
              GetTicksForFrame(next_frame_id) * cfg_->Frame().NumTotalSyms();
          DelayTicks(tick_start, frame_ticks);
          tick_start += frame_ticks;
          // Save the scheduled time to apply for the end of the frame
          timestamp_us = GetTime::GetTimeUs();
          ScheduleFrame(next_frame_id);
          LoadFrame(next_frame_id + kFrameLoadAdvance);
        }
      }
    }  // end (ret > 0)
  }
  AGORA_LOG_INFO("MacSender: main thread exit\n");
  WriteStatsToFile(cfg_->FramesToTest());
  return nullptr;
}

/* Worker will send 1 MacPacket data size to the radio)
 * since we are using UDP between the antennas and in the multiple antenna case
 * the packet data could get mixed at the mac_thread receiver
 */
void* MacSender::WorkerThread(size_t tid) {
  PinToCoreWithOffset(ThreadType::kWorkerTX,
                      (core_offset_ + 1 + update_thread_num_), tid);

  // Wait for all Sender threads (including master) to start runnung
  num_workers_ready_atomic.fetch_add(1);
  while (num_workers_ready_atomic.load() <
         (worker_thread_num_ + 1 + update_thread_num_)) {
    // Wait
  }
  AGORA_LOG_FRAME("MacSender[%zu]: worker thread running\n", tid);
  const size_t max_symbol_id = 1;

  size_t ant_per_thread = cfg_->UeAntNum() / worker_thread_num_;
  if ((cfg_->UeAntNum() % worker_thread_num_) != 0) {
    ant_per_thread++;
  }

  const size_t ue_ant_low = tid * ant_per_thread;
  const size_t ue_ant_high =
      std::min((ue_ant_low + ant_per_thread), cfg_->UeAntNum()) - 1;

  const size_t ant_this_thread = (ue_ant_high - ue_ant_low) + 1;

  if (ue_ant_low >= cfg_->UeAntNum()) {
    AGORA_LOG_WARN(
        "MacSender[%zu]: worker thread exiting because there are no antennas "
        "left to process %zu:%zu\n",
        tid, ue_ant_low, ue_ant_high);
    return nullptr;
  }

  //Send from local address
  UDPClient udp_client(kMacSendFromAddress, kMacSendFromPort);

  double begin = GetTime::GetTimeUs();
  size_t total_tx_packets = 0;
  size_t total_tx_packets_rolling = 0;
  size_t cur_ant = ue_ant_low;

  AGORA_LOG_INFO(
      "MacSender[%zu]: processing work for antennas %zu:%zu total: %zu:%zu\n",
      tid, ue_ant_low, ue_ant_high, ant_this_thread, cfg_->UeAntNum());

  std::array<size_t, kDequeueBulkSize> tags;
  while (keep_running.load() == true) {
    size_t num_tags = this->send_queue_.try_dequeue_bulk_from_producer(
        *(this->task_ptok_[tid]), tags.data(), kDequeueBulkSize);
    if (num_tags > 0) {
      for (size_t tag_id = 0; (tag_id < num_tags); tag_id++) {
        const size_t start_tsc_send = GetTime::Rdtsc();

        const auto tag = gen_tag_t(tags.at(tag_id));

        if ((kDebugPrintSender)) {
          AGORA_LOG_INFO("MacSender[%zu] : worker processing frame %d : %d \n",
                         tid, tag.frame_id_, tag.ant_id_);
        }
        const uint8_t* mac_packet_location =
            tx_buffers_[TagToTxBuffersIndex(tag)];

        // Send the mac data to the data sinc
        for (size_t packet = 0; packet < packets_per_frame_; packet++) {
          ///\todo Use assume_aligned<kTxBufferElementAlignment> when code has
          /// c++20 support
          const auto* tx_packet =
              reinterpret_cast<const MacPacketPacked*>(mac_packet_location);

          const size_t mac_packet_tx_size =
              mac_packet_length_ -
              (mac_payload_max_length_ - tx_packet->PayloadLength());

          AGORA_LOG_TRACE(
              "MacSender[%zu] sending frame %d:%d, packet %zu, symbol %d, size "
              "%zu\n",
              tid, tx_packet->Frame(), tag.frame_id_, packet,
              tx_packet->Symbol(), mac_packet_tx_size);

          udp_client.Send(server_address_, server_rx_port_,
                          reinterpret_cast<const std::byte*>(tx_packet),
                          mac_packet_tx_size);
          mac_packet_location += tx_buffer_pkt_offset_;
        }

        if (kDebugSenderReceiver) {
          AGORA_LOG_INFO(
              "MacSender%zu]: (tag = %s) transmit frame %d, ant %zu, TX "
              "time: %.3f us\n",
              tid, gen_tag_t(tag).ToString().c_str(), tag.frame_id_, cur_ant,
              GetTime::CyclesToUs(GetTime::Rdtsc() - start_tsc_send,
                                  freq_ghz_));
        }

        total_tx_packets_rolling++;
        total_tx_packets++;
        if (total_tx_packets_rolling ==
            ant_this_thread * max_symbol_id * 1000) {
          double end = GetTime::GetTimeUs();
          double byte_len =
              cfg_->PacketLength() * ant_this_thread * max_symbol_id * 1000.f;
          double diff = end - begin;
          AGORA_LOG_INFO(
              "MacSender%zu]: send %zu frames in %f secs, tput %f Mbps\n",
              (size_t)tid, total_tx_packets / (ant_this_thread * max_symbol_id),
              diff / 1e6, byte_len * 8 * 1e6 / diff / 1024 / 1024);
          begin = GetTime::GetTimeUs();
          total_tx_packets_rolling = 0;
        }

        if (cur_ant == ue_ant_high) {
          cur_ant = ue_ant_low;
        } else {
          cur_ant++;
        }
      }
      RtAssert(completion_queue_.enqueue_bulk(tags.data(), num_tags),
               "Completion enqueue failed");
    }  // if (num_tags > 0)
  }    // while (keep_running.load() == true)
  AGORA_LOG_FRAME("MacSender: worker thread %zu exit\n", tid);
  return nullptr;
}

uint64_t MacSender::GetTicksForFrame(size_t frame_id) const {
  if (enable_slow_start_ == 0) {
    return ticks_all_;
  } else if (frame_id < kSlowStartThresh1) {
    return ticks_wnd1_;
  } else if (frame_id < kSlowStartThresh2) {
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
void* MacSender::DataUpdateThread(size_t tid, size_t num_data_sources) {
  size_t buffer_updates = 0;
  PinToCoreWithOffset(ThreadType::kWorkerMacTXRX, core_offset_ + 1, tid);

  // Split the Ue data up between threads and sources
  size_t ue_per_thread = (cfg_->UeAntNum() / update_thread_num_);
  if (cfg_->UeAntNum() / update_thread_num_ > 0) {
    ue_per_thread++;
  }

  const size_t ue_ant_low = tid * ue_per_thread;
  const size_t ue_ant_high =
      std::min((ue_ant_low + ue_per_thread), cfg_->UeAntNum()) - 1;

  // Sender gets better performance when this thread is not pinned to core
  AGORA_LOG_INFO(
      "MacSender: Data update thread %zu running on core %d servicing ue "
      "%zu:%zu data\n",
      tid, sched_getcpu(), ue_ant_low, ue_ant_high);

#if defined(USE_UDP_DATA_SOURCE)
  std::vector<std::unique_ptr<VideoReceiver>> sources;
#else
  ///\todo need a list of file names for this
  std::vector<std::unique_ptr<FileReceiver>> sources;
#endif

  for (size_t source = 0; source < num_data_sources; source++) {
#if defined(USE_UDP_DATA_SOURCE)
    // Assumes that the num_data_sources are spread evenly between threads
    sources.emplace_back(std::make_unique<VideoReceiver>(
        VideoReceiver::kVideoStreamRxPort + (tid * num_data_sources) + source));
#else
    ///\todo need a list of file names for this
    sources.emplace_back(std::make_unique<FileReceiver>(data_filename_));
#endif
  }

  // Init the data buffers
  while ((keep_running.load() == true) && (buffer_updates < kBufferInit)) {
    size_t tag = 0;
    if (data_update_queue_.at(tid).try_dequeue(tag) == true) {
      for (size_t i = ue_ant_low; i <= ue_ant_high; i++) {
        auto tag_for_ue = gen_tag_t::FrmSymUe(((gen_tag_t)tag).frame_id_,
                                              ((gen_tag_t)tag).symbol_id_, i);
        size_t ant_source = i % num_data_sources;
        UpdateTxBuffer(sources.at(ant_source).get(), tag_for_ue);
      }
      buffer_updates++;
    }
  }

  AGORA_LOG_INFO("MacSender[%zu]: Data update initialized\n", tid);
  // Unlock the rest of the workers
  num_workers_ready_atomic.fetch_add(1);
  // Normal run loop
  while (keep_running.load() == true) {
    size_t tag = 0;
    if (data_update_queue_.at(tid).try_dequeue(tag) == true) {
      for (size_t i = ue_ant_low; i <= ue_ant_high; i++) {
        auto tag_for_ue = gen_tag_t::FrmSymUe(((gen_tag_t)tag).frame_id_,
                                              ((gen_tag_t)tag).symbol_id_, i);
        size_t ant_source = i % num_data_sources;
        UpdateTxBuffer(sources.at(ant_source).get(), tag_for_ue);
      }
    }
  }
  return nullptr;
}

void MacSender::UpdateTxBuffer(MacDataReceiver* data_source, gen_tag_t tag) {
  // Load a frames worth of data
  uint8_t* mac_packet_location = tx_buffers_[TagToTxBuffersIndex(tag)];

  for (size_t i = 0; i < packets_per_frame_; i++) {
    auto* pkt = reinterpret_cast<MacPacketPacked*>(mac_packet_location);
    // Read a MacPayload into the data section
    size_t loaded_bytes =
        data_source->Load(pkt->DataPtr(), mac_payload_max_length_);

    pkt->Set(tag.frame_id_, get_data_symbol_id_(i), tag.ue_id_, loaded_bytes);

    if (loaded_bytes > mac_payload_max_length_) {
      AGORA_LOG_ERROR(
          "MacSender [frame %d, ue %d]: Too much data was loaded from the "
          "source\n",
          tag.frame_id_, tag.ant_id_);
    } else if (loaded_bytes < mac_payload_max_length_) {
      AGORA_LOG_INFO(
          "MacSender [frame %d, ue %d]: Not enough mac data available sending "
          "%zu bytes of padding\n",
          tag.frame_id_, tag.ant_id_, mac_payload_max_length_ - loaded_bytes);
    }
    // MacPacketLength should be the size of the mac packet but is not.
    mac_packet_location += tx_buffer_pkt_offset_;
  }
  AGORA_LOG_INFO("MacSender [frame %d, ue %d]: Loaded packet for bytes %zu\n",
                 tag.frame_id_, tag.ant_id_,
                 mac_payload_max_length_ * packets_per_frame_);
}

void MacSender::WriteStatsToFile(size_t tx_frame_count) const {
  std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  std::string filename = cur_directory + "/files/experiment/max_tx_result.txt";
  AGORA_LOG_INFO("Printing mac sender results to file \"%s\"...\n",
                 filename.c_str());

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
