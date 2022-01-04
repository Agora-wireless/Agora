/**
 * @file packet_txrx_radio.cc
 * @brief Implementation of PacketTxRxRadio initialization functions, and datapath
 * functions for communicating with hardware.
 */

#include "packet_txrx_radio.h"

#include "logger.h"
#include "txrx_worker_argos.h"
#include "txrx_worker_usrp.h"

PacketTxRxRadio::PacketTxRxRadio(
    Config* const cfg, size_t core_offset,
    moodycamel::ConcurrentQueue<EventData>* event_notify_q,
    moodycamel::ConcurrentQueue<EventData>* tx_pending_q,
    moodycamel::ProducerToken** notify_producer_tokens,
    moodycamel::ProducerToken** tx_producer_tokens, Table<char>& rx_buffer,
    size_t packet_num_in_buffer, Table<size_t>& frame_start, char* tx_buffer)
    : PacketTxRx(cfg, core_offset, event_notify_q, tx_pending_q,
                 notify_producer_tokens, tx_producer_tokens, rx_buffer,
                 packet_num_in_buffer, frame_start, tx_buffer) {
  radio_config_ = std::make_unique<RadioConfig>(cfg);
}

PacketTxRxRadio::~PacketTxRxRadio() {
  cfg_->Running(false);
  radio_config_->RadioStop();
  for (auto& worker_threads : worker_threads_) {
    worker_threads->Stop();
  }
  radio_config_.reset();
}

bool PacketTxRxRadio::StartTxRx(Table<complex_float>& calib_dl_buffer,
                                Table<complex_float>& calib_ul_buffer) {
  MLPD_INFO("PacketTxRxRadio: StartTxRx threads %zu\n", worker_threads_.size());

  bool status = radio_config_->RadioStart();

  //RadioStart creates the following: radio_config_->GetCalibDl() and radio_config_->GetCalibUl();
  if (cfg_->Frame().NumDLSyms() > 0) {
    std::memcpy(
        calib_dl_buffer[kFrameWnd - 1], radio_config_->GetCalibDl(),
        cfg_->OfdmDataNum() * cfg_->BfAntNum() * sizeof(arma::cx_float));
    std::memcpy(
        calib_ul_buffer[kFrameWnd - 1], radio_config_->GetCalibUl(),
        cfg_->OfdmDataNum() * cfg_->BfAntNum() * sizeof(arma::cx_float));
  }

  if (status == false) {
    std::fprintf(stderr, "PacketTxRxRadio: Failed to start radio\n");
  } else {
    PacketTxRx::StartTxRx(calib_dl_buffer, calib_ul_buffer);

    for (auto& worker : worker_threads_) {
      while (worker->Started() == false) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      MLPD_INFO("PacketTxRxRadio[%zu] : worker started \n", worker->Id());
    }
    radio_config_->Go();
  }
  return status;
}

bool PacketTxRxRadio::CreateWorker(size_t tid, size_t interface_count,
                                   size_t interface_offset,
                                   size_t* rx_frame_start,
                                   std::vector<RxPacket>& rx_memory,
                                   std::byte* const tx_memory) {
  MLPD_INFO(
      "PacketTxRxRadio[%zu]: Creating worker handling %zu interfaces starting "
      "at %zu - antennas %zu:%zu\n",
      tid, interface_count, interface_offset,
      interface_offset * cfg_->NumChannels(),
      ((interface_offset * cfg_->NumChannels()) +
       (interface_count * cfg_->NumChannels()) - 1));

  //This is the spot to choose what type of TxRxWorker you want....
  if (kUseArgos) {
    worker_threads_.emplace_back(std::make_unique<TxRxWorkerArgos>(
        core_offset_, tid, interface_count, interface_offset, cfg_,
        rx_frame_start, event_notify_q_, tx_pending_q_,
        *tx_producer_tokens_[tid], *notify_producer_tokens_[tid], rx_memory,
        tx_memory, radio_config_.get()));
  } else if (kUseUHD) {
    worker_threads_.emplace_back(std::make_unique<TxRxWorkerUsrp>(
        core_offset_, tid, interface_count, interface_offset, cfg_,
        rx_frame_start, event_notify_q_, tx_pending_q_,
        *tx_producer_tokens_[tid], *notify_producer_tokens_[tid], rx_memory,
        tx_memory, radio_config_.get()));
  } else {
    RtAssert(false, "This class does not support the current configuration");
  }
  return true;
}