/**
 * @file  recorder.cc
 * @brief Record received frames from massive-mimo base station in HDF5 format
 */
#include "recorder.h"

#include "logger.h"
//#include "macros.h"
#include "rx_memory.h"
#include "signal_handler.h"
//#include "utils.h"

namespace Agora_recorder {
// buffer length of each rx thread
const int Recorder::kSampleBufferFrameNum = 80;
// dequeue bulk size, used to reduce the overhead of dequeue in main thread
const int Recorder::KDequeueBulkSize = 5;

#if (DEBUG_PRINT)
const int kDsSim = 5;
#endif

static const int kQueueSize = 36;

Recorder::Recorder(Config* in_cfg, unsigned int core_start)
    : cfg_(in_cfg),
      kMainDispatchCore(core_start),
      kRecorderCore(kMainDispatchCore + 1),
      kRecvCore(kRecorderCore + 1) {
  num_writter_threads_ = 1;

  size_t ant_per_rx_thread = cfg_->GetNumAntennas() / num_writter_threads_;

  rx_thread_buff_size_ =
      kSampleBufferFrameNum * cfg_->Frame().NumTotalSyms() * ant_per_rx_thread;

  message_queue_ =
      moodycamel::ConcurrentQueue<EventData>(rx_thread_buff_size_ * kQueueSize);

  MLPD_TRACE(
      "Recorder construction: rx threads: %zu, recorder threads: %u, "
      "chunk size: %zu\n",
      rx_thread_num, cfg_->task_thread_num(), rx_thread_buff_size_);
}

Recorder::~Recorder() { this->Gc(); }

void Recorder::Gc() { MLPD_TRACE("Garbage collect\n"); }

void Recorder::DoIt() {
  size_t total_antennas = cfg_->GetNumAntennas();
  size_t thread_antennas = 0;

  MLPD_TRACE("Recorder work thread\n");
  // if ((this->cfg_->core_alloc() == true) &&  -- PinToCoreWithOffset
  //    (pin_to_core(kMainDispatchCore) != 0)) {
  //  MLPD_ERROR("Pinning main recorder thread to core 0 failed");
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

    for (unsigned int i = 0u; i < num_writter_threads_; i++) {
      int thread_core = -1;
      // if (this->cfg_->core_alloc() == true) {
      thread_core = kRecorderCore + i;
      //}

      MLPD_INFO(
          "Creating recorder thread: %u, with antennas %zu:%zu "
          "total %zu\n",
          i, (i * thread_antennas), ((i + 1) * thread_antennas) - 1,
          thread_antennas);
      Agora_recorder::RecorderThread* new_recorder =
          new Agora_recorder::RecorderThread(
              this->cfg_, i, thread_core,
              (this->rx_thread_buff_size_ * kQueueSize), (i * thread_antennas),
              thread_antennas, true);
      new_recorder->Start();
      this->recorders_.push_back(new_recorder);
    }
  }

  moodycamel::ConsumerToken ctok(this->message_queue_);

  std::array<EventData, KDequeueBulkSize> events_list;
  size_t ret = 0;

  while ((this->cfg_->Running() == true) &&
         (SignalHandler::GotExitSignal() == false)) {
    // get a bulk of events from the receivers
    ret = this->message_queue_.try_dequeue_bulk(ctok, events_list.data(),
                                                KDequeueBulkSize);
    // if (ret > 0)
    //{
    //    MLPD_TRACE("Message(s) received: %d\n", ret );
    //}
    // handle each event
    for (size_t bulk_count = 0; bulk_count < ret; bulk_count++) {
      EventData& event = events_list.at(bulk_count);

      // if kEventRxSymbol, dispatch to proper worker
      if (event.event_type_ == EventType::kPacketRX) {
        Packet* pkt =
            AgoraNetwork::rx_tag_t(event.tags_[0]).rx_packet_->RawPacket();

        //Recording Thread Router
        size_t thread_index = pkt->ant_id_ / thread_antennas;
        Agora_recorder::RecorderThread::RecordEventData do_record_task;
        do_record_task.event_type_ =
            Agora_recorder::RecorderThread::RecordEventType::kTaskRecordRx;
        do_record_task.record_event_ = event;

        // Pass the work off to the applicable worker
        // If no worker threads, it is possible to handle the event directly.
        // this->worker_.handleEvent(do_record_task, 0);
        if (this->recorders_.at(thread_index)->DispatchWork(do_record_task) ==
            false) {
          MLPD_ERROR("Record task enqueue failed\n");
          throw std::runtime_error("Record task enqueue failed");
        }
      }
    }
  }
  this->cfg_->Running(false);

  /* Force the recorders to process all of the data they have left and exit
   * cleanly Send a stop to all the recorders to allow the finalization to be
   * done in parrallel */
  for (auto recorder : this->recorders_) {
    recorder->Stop();
  }
  for (auto recorder : this->recorders_) {
    delete recorder;
  }
  this->recorders_.clear();
}

size_t Recorder::GetRecordedFrameNum() { return this->max_frame_number_; }

extern "C" {
Recorder* Recorder_new(Config* in_cfg) {
  Recorder* rec = new Recorder(in_cfg);
  return rec;
}

void Recorder_start(Recorder* rec) { rec->DoIt(); }
int Recorder_getRecordedFrameNum(Recorder* rec) {
  return rec->GetRecordedFrameNum();
}
const char* Recorder_getTraceFileName(Recorder* rec) {
  return rec->GetTraceFileName().c_str();
}
}
};  // end namespace Agora_recorder
