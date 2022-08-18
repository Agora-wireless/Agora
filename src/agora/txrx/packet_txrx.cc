/**
 * @file packet_txrx.cc
 * @brief Implementation of PacketTxRx initialization functions, and datapath
 * functions for communicating with simulators.
 */

#include "packet_txrx.h"

#include "logger.h"

static constexpr size_t kNotifyWaitMs = 100;
static constexpr size_t kWorkerStartWaitMs = 10;
static constexpr size_t kWorkerStartWaitMsMax = 5000;

PacketTxRx::PacketTxRx(AgoraTxRx::TxRxTypes type, Config* const cfg,
                       size_t core_offset,
                       moodycamel::ConcurrentQueue<EventData>* event_notify_q,
                       moodycamel::ConcurrentQueue<EventData>* tx_pending_q,
                       moodycamel::ProducerToken** notify_producer_tokens,
                       moodycamel::ProducerToken** tx_producer_tokens,
                       Table<char>& rx_buffer, size_t packet_num_in_buffer,
                       Table<size_t>& frame_start, char* tx_buffer)
    : cfg_(cfg),
      core_offset_(core_offset),
      event_notify_q_(event_notify_q),
      tx_pending_q_(tx_pending_q),
      notify_producer_tokens_(notify_producer_tokens),
      tx_producer_tokens_(tx_producer_tokens),
      proceed_(false),
      tx_memory_(reinterpret_cast<std::byte* const>(tx_buffer)),
      frame_start_(frame_start),
      type_(type) {
  size_t total_radios;
  size_t requested_worker_threads;
  if (type_ == AgoraTxRx::TxRxTypes::kBaseStation) {
    total_radios = cfg->NumRadios();
    requested_worker_threads = cfg->SocketThreadNum();
    num_channels_ = cfg->NumChannels();
  } else {
    total_radios = cfg->UeNum();
    requested_worker_threads = cfg->UeSocketThreadNum();
    num_channels_ = cfg->NumUeChannels();
  }

  /// Will make (packet_num_in_buffer % total_radios) unused buffers
  const size_t buffers_per_interface = packet_num_in_buffer / total_radios;

  /// Make sure all antennas on an interface is assigned to the same worker
  AGORA_LOG_INFO(
      "PacketTxRx: Number of workers %zu, Buffers per interface %zu, Number of "
      "Total buffers %zu\n",
      requested_worker_threads, buffers_per_interface, packet_num_in_buffer);

  /// For each requested worker, start assigning interfaces / buffers
  size_t target_interface_count = total_radios / requested_worker_threads;
  const size_t remaining_interfaces = total_radios % requested_worker_threads;
  if (remaining_interfaces > 0) {
    /// Front load the workers
    target_interface_count++;
  }

  /// Worker <-> Interface assignment logic
  /// - Front load the workers with the target number of interfaces
  rx_packets_.resize(requested_worker_threads);
  size_t actual_worker_threads = requested_worker_threads;
  for (size_t worker = 0; worker < actual_worker_threads; worker++) {
    for (size_t interface = 0; interface < target_interface_count;
         interface++) {
      interface_to_worker_.push_back(worker);
      AGORA_LOG_FRAME("Interface: %zu, assigned to worker %zu\n",
                      interface_to_worker_.size(), worker);

      /// Distribute the buffers per interface
      for (size_t buffer = 0; buffer < buffers_per_interface; buffer++) {
        auto* pkt_loc = reinterpret_cast<Packet*>(
            rx_buffer[worker] +
            (((interface * buffers_per_interface) + buffer) *
             cfg_->PacketLength()));
        rx_packets_.at(worker).emplace_back(pkt_loc);
      }

      /// If last interface has been assigned, exit assignment.
      if (interface_to_worker_.size() == total_radios) {
        /// The +1 is for worker_index to worker count conversion
        worker = worker + 1;
        actual_worker_threads = worker;
        break;
      }
    }
  }
  if (actual_worker_threads != requested_worker_threads) {
    AGORA_LOG_WARN(
        "Using less than the number of requested worker threads %zu:%zu\n",
        actual_worker_threads, requested_worker_threads);
  }
  rx_packets_.resize(actual_worker_threads);

  AGORA_LOG_FRAME(
      "PacketTxRx: Number of workers %zu, Interfaces per thread: ~%zu, Buffers "
      "per thread: ~%zu\n",
      actual_worker_threads, target_interface_count,
      target_interface_count * buffers_per_interface);

  worker_thread_count_ = actual_worker_threads;
}

PacketTxRx::~PacketTxRx() { StopTxRx(); }

bool PacketTxRx::StopTxRx() {
  cfg_->Running(false);
  for (auto& worker_threads : worker_threads_) {
    worker_threads->Stop();
  }
  return true;
}

bool PacketTxRx::StartTxRx(Table<complex_float>& calib_dl_buffer,
                           Table<complex_float>& calib_ul_buffer) {
  unused(calib_dl_buffer);
  unused(calib_ul_buffer);

  const size_t num_interfaces = interface_to_worker_.size();

  size_t interface = 0;
  //Create the worker objects per the mapping completed in the constructor
  for (size_t worker_id = 0; worker_id < NumberTotalWorkers(); worker_id++) {
    size_t interface_offset = interface;
    //Search can be optimized but probably not necessary as this isn't a high frequncy call
    // assumes that the workers are assigned sequentially, ordered, and a match exists
    while (interface < num_interfaces) {
      if (interface_to_worker_.at(interface) == worker_id) {
        interface++;
      } else {
        /// Can exit here because this list is ordered
        break;
      }
    }

    const size_t num_worker_interfaces = interface - interface_offset;
    CreateWorker(worker_id, num_worker_interfaces, interface_offset,
                 frame_start_[worker_id], rx_packets_.at(worker_id),
                 tx_memory_);
    AGORA_LOG_FRAME("Creating worker %zu, with interfaces %zu, offset %zu\n",
                    worker_id, num_worker_interfaces, interface_offset);
  }

  AGORA_LOG_FRAME("PacketTxRx: StartTxRx threads %zu\n",
                  worker_threads_.size());
  for (auto& worker : worker_threads_) {
    worker->Start();
    size_t waited_ms = 0;
    while (worker->Started() == false) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(kWorkerStartWaitMs));
      waited_ms += kWorkerStartWaitMs;
      if (waited_ms >= kWorkerStartWaitMsMax) {
        throw std::runtime_error(
            "TxRx worker did not start in reasonable time");
      }
    }
    AGORA_LOG_TRACE("PacketTxRx: worker %zu has started \n", worker->Id());
  }
  AGORA_LOG_TRACE("PacketTxRx: notifying workers\n");
  NotifyWorkers();
  AGORA_LOG_INFO("PacketTxRx: workers synchronized\n");
  return true;
}

size_t PacketTxRx::AntNumToWorkerId(size_t ant_num) const {
  return (interface_to_worker_.at(ant_num / num_channels_));
}

void PacketTxRx::NotifyWorkers() {  //Sync the workers
  if (proceed_ == false) {
    std::this_thread::sleep_for(std::chrono::milliseconds(kNotifyWaitMs));
    {
      std::unique_lock<std::mutex> locker(mutex_);
      proceed_ = true;
      cond_.notify_all();
    }
  }
}