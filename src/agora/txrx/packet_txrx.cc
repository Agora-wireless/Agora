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
      tx_memory_(reinterpret_cast<std::byte* const>(tx_buffer)),
      frame_start_(frame_start) {
  size_t num_worker_threads = cfg->SocketThreadNum();
  size_t num_ant_per_worker = (cfg->NumAntennas() / num_worker_threads) +
                              ((cfg->NumAntennas() % num_worker_threads) != 0);

  /// Interface to worker map (using vector because id's are sequential starting at 0)
  interface_to_worker_.resize(cfg->NumRadios());

  event_notify_q_ = event_notify_q;
  tx_pending_q_ = tx_pending_q;
  notify_producer_tokens_ = notify_producer_tokens;
  tx_producer_tokens_ = tx_producer_tokens;

  const size_t interfaces_per_worker = num_ant_per_worker / cfg_->NumChannels();
  /// Make sure we can fit each channel in the thread buffer without rollover
  RtAssert((num_ant_per_worker % cfg_->NumChannels()) == 0,
           "Socket threads are misaligned with the number of channels\n");

  size_t min_threads = cfg->NumAntennas() / num_ant_per_worker;
  if (min_threads < num_worker_threads) {
    MLPD_WARN(
        "Using less than requested number of socket worker threads %zu : %zu\n",
        min_threads, num_worker_threads);
  } else {
    min_threads = num_worker_threads;
  }

  /// Spread buffers evenly
  const size_t buffers_per_thread = packet_num_in_buffer / min_threads;

  rx_packets_.resize(min_threads);
  for (size_t i = 0; i < min_threads; i++) {
    rx_packets_.at(i).reserve(buffers_per_thread);
    for (size_t number_packets = 0; number_packets < buffers_per_thread;
         number_packets++) {
      auto* pkt_loc = reinterpret_cast<Packet*>(
          rx_buffer[i] + (number_packets * cfg_->PacketLength()));
      rx_packets_.at(i).emplace_back(pkt_loc);
    }
  }
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
  const size_t interfaces_per_worker = num_interfaces / num_workers;
  RtAssert((num_interfaces % num_workers == 0),
           "Incorrect number of interfaces\n");
  size_t handled_interfaces = 0;
  for (size_t worker_id = 0; worker_id < num_workers; worker_id++) {
    const size_t interface_count =
        std::min(interfaces_per_worker, num_interfaces - handled_interfaces);
    CreateWorker(worker_id, interface_count, handled_interfaces,
                 frame_start_[worker_id], rx_packets_.at(worker_id),
                 tx_memory_);

    for (size_t interface = 0; interface < interface_count; interface++) {
      interface_to_worker_.at(handled_interfaces) = worker_id;
      handled_interfaces++;
    }
  }

  MLPD_INFO("PacketTxRx: StartTxRx threads %zu\n", worker_threads_.size());
  for (auto& worker : worker_threads_) {
    worker->Start();
  }
  return true;
}

size_t PacketTxRx::AntNumToWorkerId(size_t ant_num) const {
  return (interface_to_worker_.at(ant_num / cfg_->NumChannels()));
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
      *notify_producer_tokens_[tid], rx_memory, tx_memory));
  return true;
}