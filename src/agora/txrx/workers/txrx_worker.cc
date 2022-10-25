/**
 * @file txrx_worker.cc
 * @brief Implementation of PacketTxRx initialization functions, and datapath
 * functions for communicating with simulators.
 */

#include "txrx_worker.h"

#include "logger.h"

TxRxWorker::TxRxWorker(size_t core_offset, size_t tid, size_t interface_count,
                       size_t interface_offset, size_t channels_per_interface,
                       Config* const config, size_t* rx_frame_start,
                       moodycamel::ConcurrentQueue<EventData>* event_notify_q,
                       moodycamel::ConcurrentQueue<EventData>* tx_pending_q,
                       moodycamel::ProducerToken& tx_producer,
                       moodycamel::ProducerToken& notify_producer,
                       std::vector<RxPacket>& rx_memory,
                       std::byte* const tx_memory, std::mutex& sync_mutex,
                       std::condition_variable& sync_cond,
                       std::atomic<bool>& can_proceed)
    : tid_(tid),
      core_offset_(core_offset),
      num_interfaces_(interface_count),
      interface_offset_(interface_offset),
      channels_per_interface_(channels_per_interface),
      rx_frame_start_(rx_frame_start),
      running_(false),
      mutex_(sync_mutex),
      cond_(sync_cond),
      can_proceed_(can_proceed),
      cfg_(config),
      rx_memory_idx_(0),
      rx_memory_(rx_memory),
      tx_memory_(tx_memory),
      event_notify_q_(event_notify_q),
      tx_pending_q_(tx_pending_q),
      tx_producer_token_(tx_producer),
      notify_producer_token_(notify_producer),
      started_(false) {}

TxRxWorker::~TxRxWorker() { Stop(); }

void TxRxWorker::Start() {
  AGORA_LOG_FRAME("TxRxWorker[%zu] starting\n", tid_);
  if (!thread_.joinable()) {
    thread_ = std::thread(&TxRxWorker::DoTxRx, this);
  } else {
    throw std::runtime_error(
        "TxRxWorker::Start() called with thread already assigned.  Ensure you "
        "have called Stop() before calling Start() a second time.");
  }
}

void TxRxWorker::Stop() {
  cfg_->Running(false);
  if (thread_.joinable()) {
    AGORA_LOG_FRAME("TxRxWorker[%zu] stopping\n", tid_);
    thread_.join();
  }
}

///Using a latch might be better but adds c++20 requirement
void TxRxWorker::WaitSync() {
  // Use mutex to sychronize data receiving across threads
  {
    std::unique_lock<std::mutex> locker(mutex_);
    AGORA_LOG_TRACE("TxRxWorker[%zu]: waiting for sync\n", tid_);
    started_ = true;
    cond_.wait(locker, [this] { return can_proceed_.load(); });
  }
  AGORA_LOG_INFO("TxRxWorker[%zu]: synchronized\n", tid_);
}

bool TxRxWorker::NotifyComplete(const EventData& complete_event) {
  auto enqueue_status =
      event_notify_q_->enqueue(notify_producer_token_, complete_event);
  if (enqueue_status == false) {
    AGORA_LOG_ERROR("TxRxWorker[%zu]: socket message enqueue failed\n", tid_);
    throw std::runtime_error("TxRxWorker: socket message enqueue failed");
  }
  return enqueue_status;
}

std::vector<EventData> TxRxWorker::GetPendingTxEvents(size_t max_events) {
  size_t max_dequeue_items;
  if (max_events == 0) {
    max_dequeue_items = num_interfaces_ * channels_per_interface_;
  } else {
    max_dequeue_items = max_events;
  }
  std::vector<EventData> tx_events(max_dequeue_items);

  //Single producer ordering in q is preserved
  const size_t dequeued_items = tx_pending_q_->try_dequeue_bulk_from_producer(
      tx_producer_token_, tx_events.data(), max_dequeue_items);

  tx_events.resize(dequeued_items);
  return tx_events;
}

//Rx memory management
// This function as implmented is not thread safe
RxPacket& TxRxWorker::GetRxPacket() {
  RxPacket& new_packet = rx_memory_.at(rx_memory_idx_);
  AGORA_LOG_TRACE("TxRxWorker [%zu]: Getting new rx packet at location %ld\n",
                  tid_, reinterpret_cast<intptr_t>(&new_packet));

  // if rx_buffer is full, exit
  if (new_packet.Empty() == false) {
    AGORA_LOG_ERROR("TxRxWorker [%zu]: rx buffer full, memory overrun\n", tid_);
    throw std::runtime_error("rx buffer full, memory overrun");
  }
  // Mark the packet as used
  new_packet.Use();

  rx_memory_idx_ = (rx_memory_idx_ + 1);
  //Round robbin
  if (rx_memory_idx_ == rx_memory_.size()) {
    rx_memory_idx_ = 0;
  }
  return new_packet;
}

//Rx memory management
// Assumes you are returning the last RxPacket obtained by GetRxPacket
// Could be dangerous if you call this on memory that has been passed to another object
// This function as implmented is not thread safe
void TxRxWorker::ReturnRxPacket(RxPacket& unused_packet) {
  size_t new_index;
  //Decrement the rx_memory_idx
  if (rx_memory_idx_ == 0) {
    new_index = rx_memory_.size() - 1;
  } else {
    new_index = rx_memory_idx_ - 1;
  }
  const RxPacket& returned_packet = rx_memory_.at(new_index);
  //Make sure we are returning the correct packet, used for extra error checking
  if (&returned_packet != &unused_packet) {
    AGORA_LOG_WARN(
        "TxRxWorker [%zu]: returned memory that wasn't used last at address "
        "%ld\n",
        tid_, reinterpret_cast<intptr_t>(&unused_packet));
  } else {
    //If they are the same, reuse the old position
    rx_memory_idx_ = new_index;
  }
  // if the returned packet is free, something is wrong
  if (unused_packet.Empty()) {
    AGORA_LOG_ERROR(
        "TxRxWorker [%zu]: rx buffer returned free memory at address %ld\n",
        tid_, reinterpret_cast<intptr_t>(&unused_packet));
    throw std::runtime_error("TxRxWorker: rx buffer returned free memory");
  }
  // Mark the packet as free
  unused_packet.Free();
}

//Returns the location of the tx packet for a given frame / symbol / antenna
Packet* TxRxWorker::GetTxPacket(size_t frame, size_t symbol, size_t ant) {
  const size_t data_symbol_idx_dl =
      Configuration()->Frame().GetDLSymbolIdx(symbol);
  const size_t offset =
      (Configuration()->GetTotalDataSymbolIdxDl(frame, data_symbol_idx_dl) *
       Configuration()->BsAntNum()) +
      ant;

  return reinterpret_cast<Packet*>(
      &tx_memory_[offset * Configuration()->DlPacketLength()]);
}

//Returns the location of the tx packet for a given frame / symbol / antenna (uplink / user)
Packet* TxRxWorker::GetUlTxPacket(size_t frame, size_t symbol, size_t ant) {
  const size_t data_symbol_idx_ul =
      Configuration()->Frame().GetULSymbolIdx(symbol);
  const size_t offset =
      (Configuration()->GetTotalDataSymbolIdxUl(frame, data_symbol_idx_ul) *
       Configuration()->UeAntNum()) +
      ant;

  if (TxRxWorker::kDebugTxMemory) {
    AGORA_LOG_INFO(
        "GetUlTxPacket: (Frame %zu Symbol %zu Ant %zu) Tx Offset %zu:%zu "
        "location %ld\n",
        frame, symbol, ant, offset, offset * Configuration()->PacketLength(),
        (intptr_t)(&tx_memory_[offset * Configuration()->PacketLength()]));
  }

  return reinterpret_cast<Packet*>(
      &tx_memory_[offset * Configuration()->PacketLength()]);
}