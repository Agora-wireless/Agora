/**
 * @brief Declaration file for the simple downlink mac receiver class
 * (interfaces with the client)
 */
#include "dl_mac_receiver.h"

#include "signal_handler.h"
#include "udp_server.h"

static const bool kDebugMacReceiver = true;

DlMacReceiver::DlMacReceiver(Config* const cfg, size_t rx_thread_num,
                             size_t core_offset)
    : rx_thread_num_(rx_thread_num), core_id_(core_offset), cfg_(cfg) {}

std::vector<std::thread> DlMacReceiver::StartRecv() {
  std::vector<std::thread> created_threads;

  std::printf("DlMacReceiver:  Start Recv threads %zu\n", rx_thread_num_);
  created_threads.resize(rx_thread_num_);

  for (size_t i = 0; i < rx_thread_num_; i++) {
    created_threads.at(i) = std::thread(&DlMacReceiver::LoopRecv, this, i);
  }
  return created_threads;
}

void* DlMacReceiver::LoopRecv(size_t tid) {
  // TODO: Temp
  size_t ue_id = tid;
  size_t core_offset = core_id_ + rx_thread_num_;
  PinToCoreWithOffset(ThreadType::kWorkerRX, core_offset, tid);

  const size_t sock_buf_size = (1024 * 1024 * 64 * 8) - 1;
  auto udp_server =
      std::make_unique<UDPServer>(cfg_->UeMacTxPort() + ue_id, sock_buf_size);

  udp_server->MakeBlocking(1);

  // TODO: Should each UE have a rx port?
  std::printf("DlMacReceiver: Set up UDP socket server listening to port %zu\n",
              cfg_->UeMacTxPort() + ue_id);

  // Malloc a rx buffer
  const size_t packet_length = cfg_->DlMacDataBytesNumPerframe();
  uint8_t rx_buffer[packet_length];

  while ((SignalHandler::GotExitSignal() == false) &&
         (cfg_->Running() == true)) {
    ssize_t recvlen =
        udp_server->RecvFrom(&rx_buffer[0u], packet_length,
                             cfg_->BsServerAddr(), cfg_->UeMacTxPort() + ue_id);
    if (recvlen < 0) {
      std::perror("recv failed");
      throw std::runtime_error("Receiver: recv failed");
    } else if (static_cast<size_t>(recvlen) == packet_length) {
      // Write the data packet to a file or push to file writter queue
      if (kDebugMacReceiver) {
        std::printf("DlMacReceiver: Thread %zu,  Received Data:", tid);
        for (size_t i = 0; i < packet_length; i++) {
          std::printf(" %02x", rx_buffer[i]);
        }
        std::printf("\n");
      }
    } else if (recvlen != 0) {
      std::printf(
          "DlMacReceiver: Recv failed with less than requested bytes %zu\n",
          recvlen);
      throw std::runtime_error(
          "DlMacReceiver: recv failed with less than requested bytes");
    }
  }
  std::printf("DlMacReceiver: Finished\n");
  return nullptr;
}
