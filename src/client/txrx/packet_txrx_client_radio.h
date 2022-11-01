/**
 * @file packet_txrx_client_radio.h
 * @brief Common definations for PacketTxRxClientRadio. Including datapath
 * functions for communicating with user hardware.
 */

#ifndef PACKETTXRX_CLIENT_RADIO_H_
#define PACKETTXRX_CLIENT_RADIO_H_

#include "client_radio.h"
#include "common_typedef_sdk.h"
#include "packet_txrx.h"

/**
 * @brief Implementations of this class provide packet I/O for Agora.
 *
 * In the "Argos" mode, this class provides SoapySDR-based communication for
 * Agora (running on the base station server or client) for communicating
 * with real wireless hardware peers (antenna hubs for the server, UE devices
 * for the client).
 */

class PacketTxRxClientRadio : public PacketTxRx {
 public:
  PacketTxRxClientRadio(Config* const cfg, size_t core_offset,
                        moodycamel::ConcurrentQueue<EventData>* event_notify_q,
                        moodycamel::ConcurrentQueue<EventData>* tx_pending_q,
                        moodycamel::ProducerToken** notify_producer_tokens,
                        moodycamel::ProducerToken** tx_producer_tokens,
                        Table<char>& rx_buffer, size_t packet_num_in_buffer,
                        Table<size_t>& frame_start, char* tx_buffer);
  ~PacketTxRxClientRadio() final;
  bool StartTxRx(Table<complex_float>& calib_dl_buffer,
                 Table<complex_float>& calib_ul_buffer) final;

 private:
  bool CreateWorker(size_t tid, size_t interface_count, size_t interface_offset,
                    size_t* rx_frame_start, std::vector<RxPacket>& rx_memory,
                    std::byte* const tx_memory) final;

  std::unique_ptr<ClientRadioConfig> radio_config_;
};

#endif  // PACKETTXRX_CLIENT_RADIO_H_
