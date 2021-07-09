/**
 * @file  recorder.cc
 * @brief Record received frames from massive-mimo base station in HDF5 format
 */
#include "recorder.h"
#include "logger.h"
#include "signal_handler.h"
#include "symbols.h"

namespace Recorder {
// buffer length of each rx thread
const int Recorder::kSampleBufferFrameNum = 80;
// dequeue bulk size, used to reduce the overhead of dequeue in main thread
const int Recorder::KDequeueBulkSize = 5;

static const int kQueueSize = 36;

Recorder::Recorder(Config *in_cfg, unsigned int core_start) :
  kMainDispatchCore(core_start),
  kRecorderCore(kMainDispatchCore + 1),
  kRecvCore(kRecorderCore + 1) {

  if(in_cfg == nullptr) {
    throw std::runtime_error("Cannot initialize with null config!");
  }
  cfg_ = in_cfg;
  num_writter_threads_ = 1;

  size_t ant_per_rx_thread = cfg_->GetNumAntennas() / num_writter_threads_;

  H5std_string file_name_ = cfg_->TraceFile();

  rx_thread_buff_size_ =
      kSampleBufferFrameNum * cfg_->Frame().NumTotalSyms() * ant_per_rx_thread;

  message_queue_ =
      moodycamel::ConcurrentQueue<EventData>(rx_thread_buff_size_ * kQueueSize);

  if (this->InitHDF5(file_name_) < 0) {
      throw std::runtime_error("Could not init the output file");
  }

  MLPD_TRACE(
      "Recorder construction: rx threads: %zu, recorder threads: %u, "
      "chunk size: %zu\n",
      rx_thread_num, cfg_->task_thread_num(), rx_thread_buff_size_);
}

Recorder::~Recorder() {
  Gc();
}

void Recorder::Gc() {
  running_.store(false);
  // Call Destructor on all recorders
  recorders_.clear();
}

herr_t Recorder::InitHDF5(H5std_string file_name) {
  herr_t ret = 0;
  try {
    H5::Exception::dontPrint();

    /* Open HDF5 file and create a root group */
    h5_file_ = std::make_unique<H5::H5File>(file_name, H5F_ACC_TRUNC);
    h5_file_->createGroup(dataset_root_prefix);
  } catch(H5::FileIException &error) {
    error.printErrorStack();
    ret = -1;
  }

  return ret;
}

void Recorder::DoIt(std::vector<std::unique_ptr<RecorderWorkerFactory>> &factories) {
  GetInstance().DoItInternal(factories);
}

void Recorder::DoItInternal(std::vector<std::unique_ptr<RecorderWorkerFactory>> &factories) {
  size_t total_antennas = cfg_->GetNumAntennas();
  size_t thread_antennas = 0;

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

      recorders_.push_back(std::make_unique<RecorderThread>(
          cfg_, factories, h5_file_.get(), i, thread_core,
          (rx_thread_buff_size_ * kQueueSize), (i * thread_antennas),
          thread_antennas, true));
    }
  }

  moodycamel::ConsumerToken ctok(this->message_queue_);

  std::array<EventData, KDequeueBulkSize> events_list;
  size_t ret = 0;

  while ((running_.load() == true) &&
         (SignalHandler::GotExitSignal() == false)) {
    // get a bulk of events from the receivers
    ret = message_queue_.try_dequeue_bulk(ctok, events_list.data(),
                                                KDequeueBulkSize);
    for (size_t bulk_count = 0; bulk_count < ret; bulk_count++) {
      EventData& event = events_list.at(bulk_count);

      // if kEventRxSymbol, dispatch to proper worker
      if (event.event_type_ == EventType::kPacketRX) {
        Packet* pkt = rx_tag_t(event.tags_[0]).rx_packet_->RawPacket();

        //Recording Thread Router
        size_t thread_index = pkt->ant_id_ / thread_antennas;
        // Pass the work off to the applicable worker
        // If no worker threads, it is possible to handle the event directly.
        // this->worker_.handleEvent(do_record_task, 0);
        if (recorders_.at(thread_index)->DispatchWork(event) ==
            false) {
          MLPD_ERROR("Record task enqueue failed\n");
          throw std::runtime_error("Record task enqueue failed");
        }
      }
    }
  }

  if(running_.load() == true) {
    // In case of exit signal, cleanup
    Gc();
  }
}
};  // end namespace Recorder
