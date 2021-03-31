/**
 * @file ul_mac_receiver.h
 * @brief Declaration file for the simple uplink mac receiver class (interfaces
 * with the basestation)
 */
#include "ul_mac_receiver.h"

#include "signal_handler.h"
#include "udp_server.h"

static const bool kDebugMacReceiver = false;

UlMacReceiver::UlMacReceiver(Config* const cfg, size_t rx_thread_num,
                             size_t core_offset)
    : rx_thread_num_(rx_thread_num), core_id_(core_offset), cfg_(cfg) {}

std::vector<std::thread> UlMacReceiver::StartRecv() {
  std::vector<std::thread> created_threads;

  std::printf("UlMacReceiver:  Start Recv threads %zu\n", rx_thread_num_);
  created_threads.resize(rx_thread_num_);

  for (size_t i = 0; i < rx_thread_num_; i++) {
    created_threads.at(i) = std::thread(&UlMacReceiver::LoopRecv, this, i);
  }
  return created_threads;
}

void* UlMacReceiver::LoopRecv(size_t tid) {
  // TODO: Temp
  size_t ue_id = tid;
  size_t core_offset = core_id_ + rx_thread_num_;
  PinToCoreWithOffset(ThreadType::kWorkerRX, core_offset, tid);

  size_t sock_buf_size = (1024 * 1024 * 64 * 8) - 1;
  auto udp_server =
      std::make_unique<UDPServer>(cfg_->MacTxPort() + ue_id, sock_buf_size);

  udp_server->MakeBlocking(1);

  // TODO: Should each UE have a rx port?
  std::printf("Set up UDP socket server listening to port %zu\n",
              cfg_->MacTxPort());

  // Malloc a rx buffer
  // rx_buffer
  const size_t packet_length = cfg_->MacDataBytesNumPerframe();
  uint8_t rx_buffer[packet_length];

  while ((SignalHandler::GotExitSignal() == false) &&
         (cfg_->Running() == true)) {
    ssize_t recvlen =
        udp_server->RecvFrom(&rx_buffer[0u], packet_length,
                             cfg_->BsServerAddr(), cfg_->MacTxPort() + ue_id);

    if (recvlen < 0) {
      std::perror("recv failed");
      throw std::runtime_error("Receiver: recv failed");
    } else if (static_cast<size_t>(recvlen) == packet_length) {
      // Write the data packet to a file or push to file writter queue

      if (kDebugMacReceiver) {
        // Read information from received packet
        printf("RX thread %zu received data size %zu\n ", tid, packet_length);
      }
    } else if (recvlen != 0) {
      std::perror("recv failed with less than requested bytes");
      throw std::runtime_error(
          "Receiver: recv failed with less than requested bytes");
    }
  }
  return nullptr;
}
