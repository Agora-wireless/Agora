/**
 * @file packet_txrx_client_sim.cc
 * @brief Implementation of PacketTxRxClientSim initialization functions, and datapath
 * functions for the client code.
 */

#include "packet_txrx_client_sim.h"

#include "logger.h"
#include "txrx_worker_client_sim.h"

PacketTxRxClientSim::PacketTxRxClientSim(
    Config* const cfg, size_t core_offset,
    moodycamel::ConcurrentQueue<EventData>* event_notify_q,
    moodycamel::ConcurrentQueue<EventData>* tx_pending_q,
    moodycamel::ProducerToken** notify_producer_tokens,
    moodycamel::ProducerToken** tx_producer_tokens, Table<char>& rx_buffer,
    size_t packet_num_in_buffer, Table<size_t>& frame_start, char* tx_buffer)
    : PacketTxRx(AgoraTxRx::TxRxTypes::kUserEquiptment, cfg, core_offset,
                 event_notify_q, tx_pending_q, notify_producer_tokens,
                 tx_producer_tokens, rx_buffer, packet_num_in_buffer,
                 frame_start, tx_buffer) {}

PacketTxRxClientSim::~PacketTxRxClientSim() = default;

bool PacketTxRxClientSim::CreateWorker(size_t tid, size_t interface_count,
                                       size_t interface_offset,
                                       size_t* rx_frame_start,
                                       std::vector<RxPacket>& rx_memory,
                                       std::byte* const tx_memory) {
  const size_t num_channels = NumChannels();
  AGORA_LOG_INFO(
      "PacketTxRx[%zu]: Creating worker handling %zu user interfaces starting "
      "at %zu - antennas %zu:%zu\n",
      tid, interface_count, interface_offset, interface_offset * num_channels,
      ((interface_offset * num_channels) + (interface_count * num_channels) -
       1));

  //This is the spot to choose what type of TxRxWorker you want....
  worker_threads_.emplace_back(std::make_unique<TxRxWorkerClientSim>(
      core_offset_, tid, interface_count, interface_offset, cfg_,
      rx_frame_start, event_notify_q_, tx_pending_q_, *tx_producer_tokens_[tid],
      *notify_producer_tokens_[tid], rx_memory, tx_memory, mutex_, cond_,
      proceed_));
  return true;
}