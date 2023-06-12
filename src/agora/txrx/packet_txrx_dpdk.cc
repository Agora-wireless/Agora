/**
 * @file packet_txrx_dpdk.cc
 * @brief Implementation of PacketTxRxDpdk datapath functions for communicating
 * with DPDK
 */

#include "packet_txrx_dpdk.h"

#include <arpa/inet.h>

#include "logger.h"
#include "txrx_worker_dpdk.h"

PacketTxRxDpdk::PacketTxRxDpdk(
    Config* const cfg, size_t core_offset,
    moodycamel::ConcurrentQueue<EventData>* event_notify_q,
    moodycamel::ConcurrentQueue<EventData>* tx_pending_q,
    moodycamel::ProducerToken** notify_producer_tokens,
    moodycamel::ProducerToken** tx_producer_tokens, Table<char>& rx_buffer,
    size_t packet_num_in_buffer, Table<size_t>& frame_start, char* tx_buffer)
    : PacketTxRx(AgoraTxRx::TxRxTypes::kBaseStation, cfg, core_offset,
                 event_notify_q, tx_pending_q, notify_producer_tokens,
                 tx_producer_tokens, rx_buffer, packet_num_in_buffer,
                 frame_start, tx_buffer) {
  const size_t num_dpdk_eth_dev = cfg_->DpdkNumPorts();
  const size_t worker_threads = NumberTotalWorkers();
  DpdkTransport::DpdkInit(core_offset_ - 1, worker_threads);
  AGORA_LOG_INFO(
      "PacketTxRxDpdk: Requested devices %zu (offset: %d), %d available, "
      "socket: %d\n",
      num_dpdk_eth_dev, cfg_->DpdkPortOffset(), rte_eth_dev_count_avail(),
      rte_socket_id());
  RtAssert(
      (num_dpdk_eth_dev + cfg_->DpdkPortOffset()) <= rte_eth_dev_count_avail(),
      "Too few eth devices available compared to the requested number "
      "(DpdkNumPorts)");
  mbuf_pool_ = DpdkTransport::CreateMempool(num_dpdk_eth_dev);

  // Assuming that all devices / ports have the same IP address?
  int ret = inet_pton(AF_INET, cfg_->BsRruAddr().c_str(), &bs_rru_addr_);
  RtAssert(ret == 1, "Invalid sender IP address");
  ret = inet_pton(AF_INET, cfg_->BsServerAddr().c_str(), &bs_server_addr_);
  RtAssert(ret == 1, "Invalid server IP address");

  std::vector<uint16_t> eth_dev_ids;
  if (cfg->DpdkMacAddrs().length() > 0) {
    eth_dev_ids = DpdkTransport::GetPortIDFromMacAddr(num_dpdk_eth_dev,
                                                      cfg->DpdkMacAddrs());
  } else {
    for (uint16_t i = 0; i < num_dpdk_eth_dev; i++) {
      eth_dev_ids.push_back(i + cfg->DpdkPortOffset());
    }
  }

  //Each dpdk dev (port_id) will have a tx/rx queue for each local port / interface that will be assigned to the port
  const size_t total_interfaces = NumberTotalInterfaces();
  const size_t total_queues = total_interfaces;
  const size_t total_eth_devices = eth_dev_ids.size();
  size_t queues_per_nic = total_queues / total_eth_devices;
  if ((total_queues % total_eth_devices) != 0) {
    queues_per_nic++;
  }
  RtAssert((worker_threads % total_eth_devices) == 0,
           "Number of socket treads must be divisible by the number of Dpdk "
           "ethernet devices");
  size_t interface_id = 0;
  worker_dev_queue_assignment_.resize(NumberTotalWorkers());
  for (auto& eth_device : eth_dev_ids) {
    const int init_status =
        DpdkTransport::NicInit(eth_device, mbuf_pool_, queues_per_nic);
    if (init_status != 0) {
      rte_exit(EXIT_FAILURE, "Cannot init nic with id %u\n", eth_device);
    }

    // Previously, Assigned all of the interfaces to workers
    // Now assign dev / queues to each worker.
    for (size_t queue = 0; queue < queues_per_nic; queue++) {
      //Assign 1 queue to each interface
      const size_t worker_id = InterfaceToWorker(interface_id);
      worker_dev_queue_assignment_.at(worker_id).push_back(
          std::make_pair(eth_device, queue));

      interface_id++;
      if (interface_id == total_interfaces) {
        break;
      }
    }
  }
  AGORA_LOG_INFO("DPDK main core id %d, worker lcores (worker + main): %d\n",
                 rte_get_main_lcore(), rte_lcore_count());
}

PacketTxRxDpdk::~PacketTxRxDpdk() {
  StopTxRx();

  rte_flow_error flow_error;
  AGORA_LOG_FRAME("~PacketTxRxDpdk: dpdk eal cleanup\n");
  rte_mempool_free(mbuf_pool_);

  uint16_t eth_port = UINT16_MAX;
  for (auto& worker_devices : worker_dev_queue_assignment_) {
    for (auto& device_queue : worker_devices) {
      const auto check_port = device_queue.first;
      if (eth_port != check_port) {
        eth_port = check_port;
        // All workers should have exited, shutdown the resources nicely
        auto ret_status = rte_flow_flush(eth_port, &flow_error);
        if (ret_status != 0) {
          AGORA_LOG_ERROR(
              "Flow cannot be flushed %d message: %s\n", flow_error.type,
              flow_error.message ? flow_error.message : "(no stated reason)");
        }

        ret_status = rte_eth_dev_stop(eth_port);
        if (ret_status < 0) {
          AGORA_LOG_ERROR("Failed to stop port %u: %s", eth_port,
                          rte_strerror(-ret_status));
        }
        ret_status = rte_eth_dev_close(eth_port);
        if (ret_status < 0) {
          AGORA_LOG_ERROR("Failed to close device %u: %s", eth_port,
                          rte_strerror(-ret_status));
        }
        AGORA_LOG_INFO("PacketTxRxDpdk::Shutdown down dev port %d\n", eth_port);
      }
    }
  }
  rte_delay_ms(100);
  rte_eal_cleanup();
}

bool PacketTxRxDpdk::CreateWorker(size_t tid, size_t interface_count,
                                  size_t interface_offset,
                                  size_t* rx_frame_start,
                                  std::vector<RxPacket>& rx_memory,
                                  std::byte* const tx_memory) {
  RtAssert(kUseDPDK, "DPDK Mode must be enabled to CreateWorker\n");

  const size_t num_channels = NumChannels();
  AGORA_LOG_INFO(
      "PacketTxRxDpdk[%zu]: Creating worker handling %zu interfaces starting "
      "at %zu - antennas %zu:%zu\n",
      tid, interface_count, interface_offset, interface_offset * num_channels,
      ((interface_offset * num_channels) + (interface_count * num_channels) -
       1));

  //interface_count = number of ports (logical) to monitor
  //interface_offset = starting port (logical)
  unsigned int thread_l_core = tid;
  for (size_t lcore_idx = 0; lcore_idx <= tid; lcore_idx++) {
    // 1 to skip main core, 0 to disable wrap
    thread_l_core = rte_get_next_lcore(thread_l_core, 1, 0);
  }

  // Verify the lcore id is enabled (should have be inited with proper id)
  const int enabled = rte_lcore_is_enabled(thread_l_core);
  if (enabled == false) {
    throw std::runtime_error("The lcore " + std::to_string(thread_l_core) +
                             " tid passed to CreateWorker is not enabled");
  }

  // launch communication and task thread onto specific core
  worker_threads_.emplace_back(std::make_unique<TxRxWorkerDpdk>(
      core_offset_, thread_l_core, interface_count, interface_offset, cfg_,
      rx_frame_start, event_notify_q_, tx_pending_q_, *tx_producer_tokens_[tid],
      *notify_producer_tokens_[tid], rx_memory, tx_memory, mutex_, cond_,
      proceed_, worker_dev_queue_assignment_.at(tid), mbuf_pool_));
  AGORA_LOG_INFO("PacketTxRxDpdk: worker %zu assigned to lcore %d \n", tid,
                 thread_l_core);

  return true;
}
