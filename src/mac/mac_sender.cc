/**
 * @file mac_sender.cc
 * @brief Implementation file for the simple mac sender class
 */
#include "mac_sender.h"

#include <thread>
#include <utility>

#include "datatype_conversion.h"
#include "file_receiver.h"
#include "logger.h"
#include "udp_client.h"
#include "video_receiver.h"

#define USE_UDP_DATA_SOURCE
static constexpr bool kDebugPrintSender = false;
static constexpr size_t kFrameLoadAdvance = 10;
static constexpr size_t kBufferInit = 10;
static constexpr size_t kTxBufferElementAlignment = 64;

static constexpr size_t kSlowStartMulStage1 = 32;
static constexpr size_t kSlowStartMulStage2 = 8;

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
      packets_per_frame_(packets_per_frame),
      server_address_(std::move(server_address)),
      server_rx_port_(server_rx_port),
      get_data_symbol_id_(std::move(get_data_symbol_id))
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
  ticks_wnd1_ = ticks_all_ * kSlowStartMulStage1;
  ticks_wnd2_ = ticks_all_ * kSlowStartMulStage2;

  // Match element alignment with buffer alignment
  const size_t padding = kTxBufferElementAlignment -
                         (cfg_->MacPacketLength() % kTxBufferElementAlignment);

  tx_buffer_pkt_offset_ = (cfg_->MacPacketLength() + padding);
  assert((tx_buffer_pkt_offset_ % kTxBufferElementAlignment) == 0);

  const size_t tx_packet_storage = (packets_per_frame_ * tx_buffer_pkt_offset_);
  // tx buffers will be an array of
  tx_buffers_.Malloc(kFrameWnd * cfg_->UeAntNum(), tx_packet_storage,
                     Agora_memory::Alignment_t::kAlign64);
  MLPD_TRACE(
      "Tx buffer size: dim1 %zu, dim2 %zu, total %zu, start %zu, end: %zu\n",
      (kFrameWnd * cfg_->UeAntNum()), tx_packet_storage,
      (kFrameWnd * cfg_->UeAntNum()) * tx_packet_storage,
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
          (worker_thread_num_ * sizeof(moodycamel::ProducerToken*))));
  for (size_t i = 0; i < worker_thread_num_; i++) {
    task_ptok_[i] = new moodycamel::ProducerToken(send_queue_);
  }

  num_workers_ready_atomic.store(0);
  // Create a master thread when started from simulator
  if (create_thread_for_master == true) {
    MLPD_INFO("MacSender: creating master thread\n");
    this->threads_.emplace_back(&MacSender::MasterThread, this,
                                worker_thread_num_);
  }

  // Add the data update thread (background data reader), need to add a variable
  // for the update source number
  for (size_t update_threads = 0; update_threads < update_thread_num_;
       update_threads++) {
    // 1 update/data stream per thread.
    this->threads_.emplace_back(&MacSender::DataUpdateThread, this,
                                update_threads, 1);

    data_update_queue_.emplace_back(
        moodycamel::ConcurrentQueue<size_t>(kMessageQueueSize));
    // make producer token here?
  }
}

MacSender::~MacSender() {
  keep_running.store(false);

  for (auto& thread : this->threads_) {
    MLPD_INFO("MacSender: Joining threads\n");
    thread.join();
  }

  for (size_t i = 0; i < worker_thread_num_; i++) {
    delete (task_ptok_[i]);
  }
  std::free(task_ptok_);
  tx_buffers_.Free();
  MLPD_INFO("MacSender: Complete\n");
}

void MacSender::StartTx() {
  this->frame_start_ = new double[kNumStatsFrames]();
  this->frame_end_ = new double[kNumStatsFrames]();

  CreateWorkerThreads(worker_thread_num_);
  signal(SIGINT, InterruptHandler);
  MasterThread(0);  // Start the master thread

  delete[](this->frame_start_);
  delete[](this->frame_end_);
}

void MacSender::StartTXfromMain(double* in_frame_start, double* in_frame_end) {
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
  for (size_t i = 0; i < cfg_->UeAntNum(); i++) {
    auto req_tag = gen_tag_t::FrmSymAnt(frame, 0, i);
    // Split up the antennas amoung the worker threads
    RtAssert(
        send_queue_.enqueue(*task_ptok_[i % worker_thread_num_], req_tag.tag_),
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
  while (num_workers_ready_atomic.load() <
         (worker_thread_num_ + 1 + update_thread_num_)) {
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
        MLPD_INFO("MacSender: Checking frame %d : %zu : %zu\n", ctag.frame_id_,
                  comp_frame_slot, frame_data_count.at(comp_frame_slot));
      }
      // Check to see if the current frame is finished (UeNum / UeAntNum)
      if (frame_data_count.at(comp_frame_slot) == cfg_->UeAntNum()) {
        frame_end_us = timestamp_us;
        // Finished with the current frame data
        frame_data_count.at(comp_frame_slot) = 0;

        size_t next_frame_id = ctag.frame_id_ + 1;
        if ((kDebugSenderReceiver == true) ||
            (kDebugPrintPerFrameDone == true)) {
          MLPD_INFO("MacSender: Tx frame %d in %.2f ms, next frame %zu\n",
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
  PinToCoreWithOffset(ThreadType::kWorkerTX,
                      (core_offset_ + 1 + update_thread_num_), tid);

  // Wait for all Sender threads (including master) to start runnung
  num_workers_ready_atomic.fetch_add(1);
  while (num_workers_ready_atomic.load() <
         (worker_thread_num_ + 1 + update_thread_num_)) {
    // Wait
  }
  MLPD_FRAME("MacSender: worker thread %d running\n", tid);

  const size_t max_symbol_id = 1;
  const size_t radio_lo = (tid * cfg_->NumRadios()) / worker_thread_num_;
  const size_t radio_hi = ((tid + 1) * cfg_->NumRadios()) / worker_thread_num_;
  const size_t ant_num_this_thread =
      cfg_->NumRadios() / worker_thread_num_ +
      (static_cast<size_t>(tid) < cfg_->NumRadios() % worker_thread_num_ ? 1
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
        const uint8_t* mac_packet_location =
            tx_buffers_[TagToTxBuffersIndex(tag)];

        // Send the mac data to the data sinc
        for (size_t packet = 0; packet < packets_per_frame_; packet++) {
          ///\todo Use assume_aligned<kTxBufferElementAlignment> when code has
          /// c++20 support
          const auto* tx_packet =
              reinterpret_cast<const MacPacketPacked*>(mac_packet_location);

          const size_t mac_packet_tx_size =
              cfg_->MacPacketLength() -
              (cfg_->MacPayloadMaxLength() - tx_packet->PayloadLength());

          // std::printf(
          //    "MacSender sending frame %d:%d, packet %zu, symbol %d, size "
          //    "%zu:%zu\n",
          //    tx_packet->frame_id_, tag.frame_id_, packet,
          //    tx_packet->symbol_id_, mac_packet_tx_size,
          //    mac_packet_storage_size);

          udp_client.Send(server_address_, server_rx_port_,
                          reinterpret_cast<const uint8_t*>(tx_packet),
                          mac_packet_tx_size);
          mac_packet_location += tx_buffer_pkt_offset_;
        }

        if (kDebugSenderReceiver) {
          std::printf(
              "MacSender: Thread %zu (tag = %s) transmit frame %d, radio %zu, "
              "TX time: %.3f us\n",
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
          std::printf(
              "MacSender: Thread %zu send %zu frames in %f secs, tput %f "
              "Mbps\n",
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
  MLPD_INFO(
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

  MLPD_INFO("MacSender[%zu]: Data update initialized\n", tid);
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
        data_source->Load(pkt->DataPtr(), cfg_->MacPayloadMaxLength());

    pkt->Set(tag.frame_id_, get_data_symbol_id_(i), tag.ue_id_, loaded_bytes);

    if (loaded_bytes > cfg_->MacPayloadMaxLength()) {
      MLPD_ERROR(
          "MacSender [frame %d, ue %d]: Too much data was loaded from the "
          "source\n",
          tag.frame_id_, tag.ant_id_);
    } else if (loaded_bytes < cfg_->MacPayloadMaxLength()) {
      MLPD_INFO(
          "MacSender [frame %d, ue %d]: Not enough mac data available sending "
          "%zu bytes of padding\n",
          tag.frame_id_, tag.ant_id_,
          cfg_->MacPayloadMaxLength() - loaded_bytes);
    }
    // MacPacketLength should be the size of the mac packet but is not.
    mac_packet_location += tx_buffer_pkt_offset_;
  }
  MLPD_INFO("MacSender [frame %d, ue %d]: Loaded packet for bytes %zu\n",
            tag.frame_id_, tag.ant_id_,
            cfg_->MacPayloadMaxLength() * packets_per_frame_);
}

void MacSender::WriteStatsToFile(size_t tx_frame_count) const {
  std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  std::string filename = cur_directory + "/data/tx_result.txt";
  MLPD_INFO("Printing sender results to file \"%s\"...\n", filename.c_str());

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
