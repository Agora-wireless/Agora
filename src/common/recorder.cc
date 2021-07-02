/**
 * @file  recorder.cc
 * @brief Record received frames from massive-mimo base station in HDF5 format
 */
#include "recorder.h"
#include "logger.h"
#include "signal_handler.h"
#include "symbols.h"

namespace Agora_recorder {
// buffer length of each rx thread
const int Recorder::kSampleBufferFrameNum = 80;
// dequeue bulk size, used to reduce the overhead of dequeue in main thread
const int Recorder::KDequeueBulkSize = 5;

static const int kQueueSize = 36;

Recorder::Recorder(Config *in_cfg, unsigned int core_start)
    : cfg_(in_cfg),
      kMainDispatchCore(core_start),
      kRecorderCore(kMainDispatchCore + 1),
      kRecvCore(kRecorderCore + 1) {
  num_writter_threads_ = 1;

  size_t ant_per_rx_thread = cfg_->GetNumAntennas() / num_writter_threads_;

  hdf5_name_ = cfg_->TraceFile();

  rx_thread_buff_size_ =
      kSampleBufferFrameNum * cfg_->Frame().NumTotalSyms() * ant_per_rx_thread;

  message_queue_ =
      moodycamel::ConcurrentQueue<EventData>(rx_thread_buff_size_ * kQueueSize);

  if (this->InitHDF5() < 0) {
      throw std::runtime_error("Could not init the output file");
  }
  this->OpenHDF5();

  MLPD_TRACE(
      "Recorder construction: rx threads: %zu, recorder threads: %u, "
      "chunk size: %zu\n",
      rx_thread_num, cfg_->task_thread_num(), rx_thread_buff_size_);
}

Recorder::~Recorder() {
  GetInstance().Gc();
}

void Recorder::Gc() {
  CloseHDF5();
  FinishHDF5();
  MLPD_TRACE("Garbage collect\n");
}

herr_t Recorder::InitHDF5() {
  herr_t ret = 0;
  try {
    H5::Exception::dontPrint();

    file_ = new H5::H5File(hdf5_name_, H5F_ACC_TRUNC);
  } catch(H5::FileIException &error) {
    error.printErrorStack();
    ret = -1;
  }

  return ret;
}

void Recorder::OpenHDF5() {
  MLPD_TRACE("Open HDF5 file: %s\n", this->hdf5_name_.c_str());
  this->file_->openFile(this->hdf5_name_, H5F_ACC_RDWR);
}

void Recorder::CloseHDF5() {
  if (this->file_ == nullptr) {
    MLPD_WARN("File does not exist while calling close: %s\n",
        this->hdf5_name_.c_str());
  } else {
    this->file_->close();
  }
}

void Recorder::FinishHDF5() {
  MLPD_TRACE("Finish HD5F file\n");
  if (this->file_ != nullptr) {
      MLPD_TRACE("Deleting the file ptr for: %s\n", hdf5_name_.c_str());
      delete this->file_;
      this->file_ = nullptr;
  }
}

void Recorder::DoIt() {
  GetInstance().DoIt_();
}

void Recorder::DoIt_() {
  size_t total_antennas = cfg_->GetNumAntennas();
  size_t thread_antennas = 0;

  MLPD_TRACE("Recorder work thread\n");

  if (num_writter_threads_ > 0) {
    thread_antennas = (total_antennas / num_writter_threads_);
    // If antennas are left, distribute them over the threads. This may assign
    // antennas that don't exist to the threads at the end. This isn't a
    // concern.
    if ((total_antennas % num_writter_threads_) != 0) {
      thread_antennas = (thread_antennas + 1);
    }

    for (unsigned int i = 0u; i < num_writter_threads_; i++) {
      int thread_core = kRecorderCore + i;

      MLPD_INFO(
          "Creating recorder thread: %u, with antennas %zu:%zu "
          "total %zu\n",
          i, (i * thread_antennas), ((i + 1) * thread_antennas) - 1,
          thread_antennas);
      Agora_recorder::RecorderThread* new_recorder =
          new Agora_recorder::RecorderThread(
              cfg_, file_, i, thread_core,
              (rx_thread_buff_size_ * kQueueSize), (i * thread_antennas),
              thread_antennas, true);
      new_recorder->Start();
      recorders_.push_back(new_recorder);
    }
  }

  moodycamel::ConsumerToken ctok(this->message_queue_);

  std::array<EventData, KDequeueBulkSize> events_list;
  size_t ret = 0;

  while ((cfg_->Running() == true) &&
         (SignalHandler::GotExitSignal() == false)) {
    // get a bulk of events from the receivers
    ret = message_queue_.try_dequeue_bulk(ctok, events_list.data(),
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
        Packet* pkt = rx_tag_t(event.tags_[0]).rx_packet_->RawPacket();

        //Recording Thread Router
        size_t thread_index = pkt->ant_id_ / thread_antennas;
        Agora_recorder::RecordEventData do_record_task;
        do_record_task.event_type_ =
            Agora_recorder::kTaskRecordRx;
        do_record_task.record_event_ = event;

        // Pass the work off to the applicable worker
        // If no worker threads, it is possible to handle the event directly.
        // this->worker_.handleEvent(do_record_task, 0);
        if (recorders_.at(thread_index)->DispatchWork(do_record_task) ==
            false) {
          MLPD_ERROR("Record task enqueue failed\n");
          throw std::runtime_error("Record task enqueue failed");
        }
      }
    }
  }
  cfg_->Running(false);

  /* Force the recorders to process all of the data they have left and exit
   * cleanly Send a stop to all the recorders to allow the finalization to be
   * done in parrallel */
  for (auto recorder : this->recorders_) {
    recorder->Stop();
  }
  for (auto recorder : this->recorders_) {
    delete recorder;
  }
  recorders_.clear();
}
};  // end namespace Agora_recorder
