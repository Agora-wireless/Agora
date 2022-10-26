/**
 * @file packet_txrx_radio.h
 * @brief Common definations for PacketTxRxRadio. Including datapath
 * functions for communicating with hardware.
 */

#ifndef PACKETTXRX_RADIO_H_
#define PACKETTXRX_RADIO_H_

#include "common_typedef_sdk.h"
#include "packet_txrx.h"
#include "radio_lib.h"

/**
 * @brief Implementations of this class provide packet I/O for Agora.
 *
 * In the "Argos" mode, this class provides SoapySDR-based communication for
 * Agora (running on the base station server or client) for communicating
 * with real wireless hardware peers (antenna hubs for the server, UE devices
 * for the client).
 */

class PacketTxRxRadio : public PacketTxRx {
 public:
  PacketTxRxRadio(Config* const cfg, size_t core_offset,
                  moodycamel::ConcurrentQueue<EventData>* event_notify_q,
                  moodycamel::ConcurrentQueue<EventData>* tx_pending_q,
                  moodycamel::ProducerToken** notify_producer_tokens,
                  moodycamel::ProducerToken** tx_producer_tokens,
                  Table<char>& rx_buffer, size_t packet_num_in_buffer,
                  Table<size_t>& frame_start, char* tx_buffer);
  ~PacketTxRxRadio() final;
  bool StartTxRx(Table<complex_float>& calib_dl_buffer,
                 Table<complex_float>& calib_ul_buffer) final;

 private:
  bool CreateWorker(size_t tid, size_t interface_count, size_t interface_offset,
                    size_t* rx_frame_start, std::vector<RxPacket>& rx_memory,
                    std::byte* const tx_memory) final;

  std::unique_ptr<RadioConfig> radio_config_;
};

#endif  // PACKETTXRX_RADIO_H_
