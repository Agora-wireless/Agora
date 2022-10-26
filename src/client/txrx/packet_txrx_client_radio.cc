/**
 * @file packet_txrx_client_radio.cc
 * @brief Implementation of PacketTxRxClientRadio initialization functions, and datapath
 * functions for communicating with user hardware.
 */

#include "packet_txrx_client_radio.h"

#include "logger.h"
#include "txrx_worker_client_hw.h"
//#include "txrx_worker_usrp.h"

static constexpr size_t kRadioTriggerWaitMs = 100;

PacketTxRxClientRadio::PacketTxRxClientRadio(
    Config* const cfg, size_t core_offset,
    moodycamel::ConcurrentQueue<EventData>* event_notify_q,
    moodycamel::ConcurrentQueue<EventData>* tx_pending_q,
    moodycamel::ProducerToken** notify_producer_tokens,
    moodycamel::ProducerToken** tx_producer_tokens, Table<char>& rx_buffer,
    size_t packet_num_in_buffer, Table<size_t>& frame_start, char* tx_buffer)
    : PacketTxRx(AgoraTxRx::TxRxTypes::kUserEquiptment, cfg, core_offset,
                 event_notify_q, tx_pending_q, notify_producer_tokens,
                 tx_producer_tokens, rx_buffer, packet_num_in_buffer,
                 frame_start, tx_buffer) {
  radio_config_ =
      std::make_unique<ClientRadioConfig>(cfg, Radio::kSoapySdrStream);
}

PacketTxRxClientRadio::~PacketTxRxClientRadio() {
  cfg_->Running(false);
  for (auto& worker_threads : worker_threads_) {
    worker_threads->Stop();
  }
  AGORA_LOG_INFO("PacketTxRxClientRadio: shutting down radios\n");
  radio_config_->RadioStop();
  radio_config_.reset();
}

bool PacketTxRxClientRadio::StartTxRx(Table<complex_float>& calib_dl_buffer,
                                      Table<complex_float>& calib_ul_buffer) {
  AGORA_LOG_FRAME("PacketTxRxClientRadio: StartTxRx threads %zu\n",
                  worker_threads_.size());
  const bool status = radio_config_->RadioStart();

  if (status == false) {
    std::fprintf(stderr, "PacketTxRxClientRadio: Failed to start radio\n");
  } else {
    PacketTxRx::StartTxRx(calib_dl_buffer, calib_ul_buffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(kRadioTriggerWaitMs));
    AGORA_LOG_INFO(
        "PacketTxRxClientRadio : All workers started triggering the radio\n");
    radio_config_->Go();
  }
  return status;
}

bool PacketTxRxClientRadio::CreateWorker(size_t tid, size_t interface_count,
                                         size_t interface_offset,
                                         size_t* rx_frame_start,
                                         std::vector<RxPacket>& rx_memory,
                                         std::byte* const tx_memory) {
  const size_t num_channels = NumChannels();
  AGORA_LOG_INFO(
      "PacketTxRxClientRadio[%zu]: Creating worker handling %zu interfaces "
      "starting at %zu - antennas %zu:%zu\n",
      tid, interface_count, interface_offset, interface_offset * num_channels,
      ((interface_offset * num_channels) + (interface_count * num_channels) -
       1));

  //This is the spot to choose what type of TxRxWorker you want....
  if (kUseArgos) {
    worker_threads_.emplace_back(std::make_unique<TxRxWorkerClientHw>(
        core_offset_, tid, interface_count, interface_offset, cfg_,
        rx_frame_start, event_notify_q_, tx_pending_q_,
        *tx_producer_tokens_[tid], *notify_producer_tokens_[tid], rx_memory,
        tx_memory, mutex_, cond_, proceed_, *radio_config_.get()));
  } else {
    RtAssert(false, "This class does not support the current configuration");
  }
  return true;
}