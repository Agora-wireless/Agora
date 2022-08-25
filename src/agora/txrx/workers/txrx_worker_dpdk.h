/**
 * @file txrx_worker_dpdk.h
 * @brief txrx worker dpdk definition.  This is the simulator declaration with dpdk enhancements
 */

#ifndef TXRX_WORKER_DPDK_H_
#define TXRX_WORKER_DPDK_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "dpdk_transport.h"
#include "txrx_worker.h"

class TxRxWorkerDpdk : public TxRxWorker {
 public:
  TxRxWorkerDpdk() = delete;
  TxRxWorkerDpdk(size_t core_offset, size_t tid, size_t interface_count,
                 size_t interface_offset, Config* const config,
                 size_t* rx_frame_start,
                 moodycamel::ConcurrentQueue<EventData>* event_notify_q,
                 moodycamel::ConcurrentQueue<EventData>* tx_pending_q,
                 moodycamel::ProducerToken& tx_producer,
                 moodycamel::ProducerToken& notify_producer,
                 std::vector<RxPacket>& rx_memory, std::byte* const tx_memory,
                 std::mutex& sync_mutex, std::condition_variable& sync_cond,
                 std::atomic<bool>& can_proceed,
                 std::vector<std::pair<uint16_t, uint16_t>> dpdk_phy,
                 rte_mempool* mbuf_pool);
  ~TxRxWorkerDpdk() final;
  void DoTxRx() final;
  void Start() final;
  void Stop() final;

 private:
  std::vector<Packet*> RecvEnqueue(uint16_t port_id, uint16_t queue_id);
  size_t DequeueSend();
  // Returns true if packet should be ignored - will garbage collect.  Handles arp requests
  bool Filter(rte_mbuf* packet, uint16_t port_id, uint16_t queue_id);

  uint32_t bs_rru_addr_;     // IPv4 address of the simulator sender
  uint32_t bs_server_addr_;  // IPv4 address of the Agora server
  // dpdk port_id / device : queue_id
  const std::vector<std::pair<uint16_t, uint16_t>> dpdk_phy_port_queues_;
  // Shared memory pool for rx and tx
  rte_mempool* mbuf_pool_;
  std::vector<rte_ether_addr> src_mac_;
  std::vector<rte_ether_addr> dest_mac_;
};
#endif  // TXRX_WORKER_DPDK_H_
