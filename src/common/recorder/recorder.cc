/**
 * @file  recorder.cc
 * @brief Record received frames from massive-mimo base station in HDF5 format
 */
#include "recorder.h"

#include "logger.h"
#include "recorder/recorder_packet.h"
#include "signal_handler.h"
#include "symbols.h"

namespace Recorder {
// buffer length of each rx thread
const int Recorder::kSampleBufferFrameNum = 80;
// dequeue bulk size, used to reduce the overhead of dequeue in main thread
const int Recorder::KDequeueBulkSize = 5;

static const int kQueueSize = 36;

Recorder::Recorder(Config *in_cfg, unsigned int core_start,
                   size_t num_writer_threads)
    : kMainDispatchCore(core_start),
      kRecorderCore(kMainDispatchCore + 1),
      kRecvCore(kRecorderCore + 1),
      num_writer_threads_(num_writer_threads) {
  if (in_cfg == nullptr) {
    throw std::runtime_error("Cannot initialize with null config!");
  }
  cfg_ = in_cfg;

  H5std_string file_name_ = cfg_->TraceFile();
  if (InitHDF5(file_name_) < 0) {
    throw std::runtime_error("Could not init the output file");
  }

  UpdateAntennaMapping(num_writer_threads);

  size_t ant_per_rx_thread = cfg_->GetNumAntennas() / num_writer_threads;
  rx_thread_buff_size_ =
      kSampleBufferFrameNum * cfg_->Frame().NumTotalSyms() * ant_per_rx_thread;
}

Recorder::~Recorder() {
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
  } catch (H5::FileIException &error) {
    error.printErrorStack();
    ret = -1;
  }

  return ret;
}

void Recorder::UpdateAntennaMapping(size_t num_writer_threads) {
  size_t total_antennas = cfg_->GetNumAntennas();
  thread_antennas_ = 0;

  if (num_writer_threads > 0) {
    thread_antennas_ = (total_antennas / num_writer_threads);
    // If antennas are left, distribute them over the threads. This may assign
    // antennas that don't exist to the threads at the end. This isn't a
    // concern.
    if ((total_antennas % num_writer_threads) != 0) {
      thread_antennas_ = (thread_antennas_ + 1);
    }
  }
}

size_t Recorder::RouteToThread(size_t tag) {
  uint32_t ant_id = recorder_tag_t(tag).recorder_packet_->GetAntId();

  //Recording Thread Router
  size_t thread_index = ant_id / thread_antennas_;
  // Pass the work off to the applicable worker
  // If no worker threads, it is possible to handle the event directly.
  // this->worker_.handleEvent(do_record_task, 0);
  return thread_index;
}

void Recorder::DoIt(
    std::vector<std::unique_ptr<RecorderWorkerFactory>> &factories) {
  GetInstance().DoItInternal(factories);
}

void Recorder::DoItInternal(
    std::vector<std::unique_ptr<RecorderWorkerFactory>> &factories) {
  if (num_writer_threads_ > 0) {
    for (unsigned int i = 0u; i < num_writer_threads_; i++) {
      int thread_core = kRecorderCore + i;

      recorders_.push_back(std::make_unique<RecorderThread>(
          cfg_, factories, h5_file_.get(), i, thread_core,
          (rx_thread_buff_size_ * kQueueSize), (i * thread_antennas_),
          thread_antennas_, true));
    }
  }
}

void Recorder::RecordInternal(size_t tag) {
  size_t thread_index = RouteToThread(tag);
  EventData event(recorder_tag_t(tag).recorder_packet_->GetEventType(), tag);
  if (recorders_.at(thread_index)->DispatchWork(event) == false) {
    MLPD_ERROR("Record task enqueue failed\n");
    throw std::runtime_error("Record task enqueue failed");
  }
}

};  // end namespace Recorder
