/**
 * @file packet_txrx_sim.h
 * @brief Common definations for PacketTxRxSim. Including datapath
 * functions for communicating with the base station code.
 */

#ifndef PACKETTXRX_SIM_H_
#define PACKETTXRX_SIM_H_

#include "packet_txrx.h"

/**
 * @brief Implementations of this class provide packet I/O for Agora.
 *
 * In the "Argos" mode, this class provides SoapySDR-based communication for
 * Agora (running on the base station server or client) for communicating
 * with real wireless hardware peers (antenna hubs for the server, UE devices
 * for the client).
 */

class PacketTxRxSim : public PacketTxRx {
 public:
  PacketTxRxSim(Config* const cfg, size_t core_offset,
                moodycamel::ConcurrentQueue<EventData>* event_notify_q,
                moodycamel::ConcurrentQueue<EventData>* tx_pending_q,
                moodycamel::ProducerToken** notify_producer_tokens,
                moodycamel::ProducerToken** tx_producer_tokens,
                Table<char>& rx_buffer, size_t packet_num_in_buffer,
                Table<size_t>& frame_start, char* tx_buffer);
  ~PacketTxRxSim() final = default;

 private:
  bool CreateWorker(size_t tid, size_t interface_count, size_t interface_offset,
                    size_t* rx_frame_start, std::vector<RxPacket>& rx_memory,
                    std::byte* const tx_memory) final;
};

#endif  // PACKETTXRX_SIM_H_
