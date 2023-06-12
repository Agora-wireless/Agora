/**
 * @file  recorder.cc
 * @brief Record received frames from massive-mimo base station in HDF5 format
 */
#include "recorder.h"

#include "logger.h"
#include "message.h"

namespace Agora_recorder {
// buffer length of each rx thread
const int Recorder::kSampleBufferFrameNum = 80;
// dequeue bulk size, used to reduce the overhead of dequeue in main thread
const int Recorder::kDequeueBulkSize = 5;
static const int kQueueSize = 36;

Recorder::Recorder(const Config* in_cfg, size_t core_start)
    : cfg_(in_cfg),
      main_dispatch_core_(core_start),
      recorder_core_(main_dispatch_core_ + 1) {
  num_writter_threads_ = 1;
  const size_t ant_per_rx_thread = cfg_->UeAntNum() / num_writter_threads_;

  rx_thread_buff_size_ =
      kSampleBufferFrameNum * cfg_->Frame().NumTotalSyms() * ant_per_rx_thread;

  message_queue_ =
      moodycamel::ConcurrentQueue<EventData>(rx_thread_buff_size_ * kQueueSize);

  AGORA_LOG_TRACE(
      "Recorder construction[%u]: recorder threads id: %u, wirtters: %zu, "
      "buffer size: %zu\n",
      num_writter_threads_, rx_thread_buff_size_);
}

Recorder::~Recorder() { Gc(); }

void Recorder::Gc() {
  AGORA_LOG_TRACE("Garbage collect\n");
  recorders_.clear();
}

void Recorder::DoIt() {
  const size_t total_antennas = cfg_->UeAntNum();
  size_t thread_antennas = 0;

  AGORA_LOG_TRACE("Recorder thread\n");
  // if ((cfg_->core_alloc() == true) &&  -- PinToCoreWithOffset
  //    (pin_to_core(kMainDispatchCore) != 0)) {
  //  AGORA_LOG_ERROR("Pinning main recorder thread to core 0 failed");
  //  throw std::runtime_error("Pinning main recorder thread to core 0 failed");
  //}

  if (num_writter_threads_ > 0) {
    thread_antennas = (total_antennas / num_writter_threads_);
    // If antennas are left, distribute them over the threads. This may assign
    // antennas that don't exist to the threads at the end. This isn't a
    // concern.
    if ((total_antennas % num_writter_threads_) != 0) {
      thread_antennas = (thread_antennas + 1);
    }

    for (size_t i = 0u; i < num_writter_threads_; i++) {
      const int thread_core = recorder_core_ + i;

      AGORA_LOG_INFO(
          "Creating recorder thread: %zu, with antennas %zu:%zu total %zu\n", i,
          (i * thread_antennas), ((i + 1) * thread_antennas) - 1,
          thread_antennas);
      auto& new_recorder = recorders_.emplace_back(
          std::make_unique<Agora_recorder::RecorderThread>(
              cfg_, i, thread_core, (rx_thread_buff_size_ * kQueueSize),
              (i * thread_antennas), thread_antennas, true));
      new_recorder->Start();
    }
  }

  moodycamel::ConsumerToken ctok(message_queue_);

  std::array<EventData, kDequeueBulkSize> events_list;

  while (cfg_->Running() == true) {
    // get a bulk of events from the receivers
    const auto ret = message_queue_.try_dequeue_bulk(ctok, events_list.data(),
                                                     kDequeueBulkSize);
    if (ret > 0) {
      AGORA_LOG_INFO("Message(s) received: %zu\n", ret);
    }
    // handle each event
    for (size_t bulk_count = 0; bulk_count < ret; bulk_count++) {
      EventData& event = events_list.at(bulk_count);

      // if kEventRxSymbol, dispatch to proper worker
      if (event.event_type_ == EventType::kPacketRX) {
        RxPacket* rx_packet = rx_tag_t(event.tags_[0u]).rx_packet_;
        auto* pkt = rx_packet->RawPacket();
        rx_packet->Use();

        //Recording Thread Router
        const size_t thread_index = pkt->ant_id_ / thread_antennas;
        // Pass the work off to the applicable worker
        // If no recorder threads, it is possible to handle the event directly.
        // worker_.handleEvent(do_record_task, 0);
        if (recorders_.at(thread_index)->DispatchWork(event) == false) {
          AGORA_LOG_ERROR("Record task enqueue failed\n");
          rx_packet->Free();
          throw std::runtime_error("Record task enqueue failed");
        }
      }
    }
  }

  /* Force the recorders to process all of the data they have left and exit
   * cleanly Send a stop to all the recorders */
  for (auto& recorder : recorders_) {
    recorder->Stop();
  }
  recorders_.clear();
}
};  // end namespace Agora_recorder
