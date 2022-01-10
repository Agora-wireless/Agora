/**
 * @file packet_txrx.cc
 * @brief Implementation of PacketTxRx initialization functions, and datapath
 * functions for communicating with simulators.
 */

#include "packet_txrx.h"

#include "logger.h"
#include "txrx_worker_sim.h"

PacketTxRx::PacketTxRx(Config* const cfg, size_t core_offset,
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
      mutex_(),
      cond_(),
      proceed_(false),
      tx_memory_(reinterpret_cast<std::byte* const>(tx_buffer)),
      frame_start_(frame_start),
      /// Interface to worker map (using vector because id's are sequential starting at 0)
      interface_to_worker_() {
  const size_t total_radios = cfg->NumRadios();
  const size_t requested_worker_threads = cfg->SocketThreadNum();
  /// Will make (packet_num_in_buffer % total_radios) unused buffers
  const size_t buffers_per_interface = packet_num_in_buffer / total_radios;

  /// Make sure all antennas on an interface is assigned to the same worker
  MLPD_INFO(
      "Number of workers: %zu, Buffers per interface %zu, Number of Total "
      "buffers %zu\n",
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
      MLPD_FRAME("Interface: %zu, assigned to worker %zu\n",
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
    MLPD_WARN(
        "Using less than the number of requested worker threads %zu:%zu\n",
        actual_worker_threads, requested_worker_threads);
  }
  rx_packets_.resize(actual_worker_threads);

  MLPD_INFO(
      "Number of workers: %zu, Interfaces per thread: ~%zu, Buffers per "
      "thread: ~%zu\n",
      actual_worker_threads, target_interface_count,
      target_interface_count * buffers_per_interface);
}

PacketTxRx::~PacketTxRx() {
  cfg_->Running(false);
  for (auto& worker_threads : worker_threads_) {
    worker_threads->Stop();
  }
}

bool PacketTxRx::StartTxRx(Table<complex_float>& calib_dl_buffer,
                           Table<complex_float>& calib_ul_buffer) {
  unused(calib_dl_buffer);
  unused(calib_ul_buffer);

  const size_t num_interfaces = interface_to_worker_.size();
  const size_t num_workers = rx_packets_.size();

  size_t interface = 0;
  //Create the worker objects per the mapping completed in the constructor
  for (size_t worker_id = 0; worker_id < num_workers; worker_id++) {
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
    MLPD_INFO("Creating worker %zu, with interfaces %zu, offset %zu\n",
              worker_id, num_worker_interfaces, interface_offset);
  }

  MLPD_INFO("PacketTxRx: StartTxRx threads %zu\n", worker_threads_.size());
  for (auto& worker : worker_threads_) {
    worker->Start();
    while (worker->Started() == false) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    MLPD_INFO("PacketTxRx: worker %zu has started \n", worker->Id());
  }
  MLPD_INFO("PacketTxRx: notifying workers\n");
  NotifyWorkers();
  MLPD_INFO("PacketTxRx: workers notified\n");
  return true;
}

size_t PacketTxRx::AntNumToWorkerId(size_t ant_num) const {
  return (interface_to_worker_.at(ant_num / cfg_->NumChannels()));
}

/*
void PacketTxRx::SyncWorkers() {
  ///Wait for all workers to be ready
  for (auto& worker : worker_threads_) {
    while (worker->Started() == false) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    MLPD_INFO("PacketTxRx[%zu] : worker has started \n", worker->Id());
  }
  MLPD_INFO("PacketTxRx: notifying workers\n");
  NotifyWorkers();
  MLPD_INFO("PacketTxRx: workers notified\n");
} */

void PacketTxRx::NotifyWorkers() {  //Sync the workers
  if (proceed_ == false) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    {
      std::unique_lock<std::mutex> locker(mutex_);
      proceed_ = true;
      cond_.notify_all();
    }
  }
}

bool PacketTxRx::CreateWorker(size_t tid, size_t interface_count,
                              size_t interface_offset, size_t* rx_frame_start,
                              std::vector<RxPacket>& rx_memory,
                              std::byte* const tx_memory) {
  MLPD_INFO(
      "PacketTxRx[%zu]: Creating worker handling %zu interfaces starting at "
      "%zu - antennas %zu:%zu\n",
      tid, interface_count, interface_offset,
      interface_offset * cfg_->NumChannels(),
      ((interface_offset * cfg_->NumChannels()) +
       (interface_count * cfg_->NumChannels()) - 1));

  //This is the spot to choose what type of TxRxWorker you want....
  RtAssert((kUseArgos == false) && (kUseUHD == false),
           "This class does not support hardware implementations");
  worker_threads_.emplace_back(std::make_unique<TxRxWorkerSim>(
      core_offset_, tid, interface_count, interface_offset, cfg_,
      rx_frame_start, event_notify_q_, tx_pending_q_, *tx_producer_tokens_[tid],
      *notify_producer_tokens_[tid], rx_memory, tx_memory, mutex_, cond_,
      proceed_));
  return true;
}