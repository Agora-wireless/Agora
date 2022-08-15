/**
 * @file txrx_worker_client_sim.h
 * @brief txrx worker thread definition.  This is the simulator declaration for the userequiptment code
 */

#ifndef TXRX_WORKER_CLIENT_SIM_H_
#define TXRX_WORKER_CLIENT_SIM_H_

#include <memory>
#include <vector>

#include "message.h"
#include "txrx_worker.h"
#include "udp_client.h"
#include "udp_server.h"

class TxRxWorkerClientSim : public TxRxWorker {
 public:
  TxRxWorkerClientSim(size_t core_offset, size_t tid, size_t interface_count,
                      size_t interface_offset, Config* const config,
                      size_t* rx_frame_start,
                      moodycamel::ConcurrentQueue<EventData>* event_notify_q,
                      moodycamel::ConcurrentQueue<EventData>* tx_pending_q,
                      moodycamel::ProducerToken& tx_producer,
                      moodycamel::ProducerToken& notify_producer,
                      std::vector<RxPacket>& rx_memory,
                      std::byte* const tx_memory, std::mutex& sync_mutex,
                      std::condition_variable& sync_cond,
                      std::atomic<bool>& can_proceed);
  TxRxWorkerClientSim() = delete;
  ~TxRxWorkerClientSim() final;

  void DoTxRx() final;

 private:
  size_t DequeueSend();
  std::vector<Packet*> RecvEnqueue(size_t interface_id);

  //1 for each responsible interface (ie radio)
  //socket for incomming messages (received data)
  std::vector<std::unique_ptr<UDPServer>> udp_servers_;

  //socket for outgoing messages (data to transmit)
  std::vector<std::unique_ptr<UDPClient>> udp_clients_;

  //Helper tx vectors
  std::vector<uint8_t> tx_pkt_zeros_;
  std::vector<uint8_t> tx_pkt_pilot_;
};
#endif  // TXRX_WORKER_CLIENT_SIM_H_
