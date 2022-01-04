/**
 * @file txrx_worker.cc
 * @brief Implementation of PacketTxRx initialization functions, and datapath
 * functions for communicating with simulators.
 */

#include "txrx_worker.h"

#include "logger.h"

TxRxWorker::TxRxWorker(size_t core_offset, size_t tid, size_t interface_count,
                       size_t interface_offset, Config* const config,
                       size_t* rx_frame_start,
                       moodycamel::ConcurrentQueue<EventData>* event_notify_q,
                       moodycamel::ConcurrentQueue<EventData>* tx_pending_q,
                       moodycamel::ProducerToken& tx_producer,
                       moodycamel::ProducerToken& notify_producer,
                       std::vector<RxPacket>& rx_memory,
                       std::byte* const tx_memory)
    : cfg_(config),
      tid_(tid),
      core_offset_(core_offset),
      num_interfaces_(interface_count),
      interface_offset_(interface_offset),
      channels_per_interface_(config->NumChannels()),
      ant_per_cell_(config->BsAntNum() / config->NumCells()),
      rx_frame_start_(rx_frame_start),
      rx_memory_(rx_memory),
      tx_memory_(tx_memory),
      event_notify_q_(event_notify_q),
      tx_pending_q_(tx_pending_q),
      tx_producer_token_(tx_producer),
      notify_producer_token_(notify_producer) {
  running_ = false;
  started_ = false;
}

TxRxWorker::~TxRxWorker() { Stop(); }

void TxRxWorker::Start() {
  MLPD_FRAME("TxRxWorker[%zu] starting\n", tid_);
  thread_ = std::thread(&TxRxWorker::DoTxRx, this);
}

void TxRxWorker::Stop() {
  MLPD_FRAME("TxRxWorker[%zu] stopping\n", tid_);
  cfg_->Running(false);
  if (thread_.joinable()) {
    thread_.join();
  }
}