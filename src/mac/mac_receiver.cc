/**
 * @file  mac_receiver.cc
 * @brief Declaration file for the simple mac receiver class
 */
#include "mac_receiver.h"

#include "logger.h"
#include "signal_handler.h"
#include "udp_client.h"
#include "udp_server.h"

#define STREAM_UDP_DATA
static constexpr char kVideoStreamingAddr[] = "10.42.0.22";
static constexpr uint16_t kVideoStreamingPort = 1230u;

static const bool kDebugMacReceiver = true;

MacReceiver::MacReceiver(Config* const cfg, size_t num_frame_data_bytes,
                         std::string server_address, size_t tx_port,
                         size_t rx_thread_num, size_t core_offset)
    : data_bytes_(num_frame_data_bytes),
      server_address_(server_address),
      server_tx_port_(tx_port),
      rx_thread_num_(rx_thread_num),
      core_id_(core_offset),
      cfg_(cfg) {}

std::vector<std::thread> MacReceiver::StartRecv() {
  std::vector<std::thread> created_threads;

  std::printf("MacReceiver:  Start Recv threads %zu\n", rx_thread_num_);
  created_threads.resize(rx_thread_num_);

  for (size_t i = 0; i < rx_thread_num_; i++) {
    created_threads.at(i) = std::thread(&MacReceiver::LoopRecv, this, i);
  }
  return created_threads;
}

void* MacReceiver::LoopRecv(size_t tid) {
  // TODO: Temp
  size_t ue_id = tid;
  size_t core_offset = core_id_ + rx_thread_num_;
  PinToCoreWithOffset(ThreadType::kWorkerRX, core_offset, tid);

  const size_t sock_buf_size = (1024 * 1024 * 64 * 8) - 1;
  auto udp_server =
      std::make_unique<UDPServer>(server_tx_port_ + ue_id, sock_buf_size);

#if defined(STREAM_UDP_DATA)
  auto udp_video_streamer = std::make_unique<UDPClient>();
#endif

  udp_server->MakeBlocking(1);
  std::printf("MacReceiver: Set up UDP socket server listening to port %zu\n",
              server_tx_port_ + ue_id);

  // Create a rx buffer
  //////// NEED TO FIX THIS -> how do we know how many data bytes????? (might not matter)
  const size_t max_packet_length = data_bytes_;
  auto* rx_buffer = new uint8_t[max_packet_length];

  while ((SignalHandler::GotExitSignal() == false) &&
         (cfg_->Running() == true)) {
    ssize_t recvlen =
        udp_server->RecvFrom(&rx_buffer[0u], max_packet_length, server_address_,
                             server_tx_port_ + ue_id);
    if (recvlen < 0) {
      std::perror("recv failed");
      throw std::runtime_error("Receiver: recv failed");
    } else if ((recvlen > 0) &&
               static_cast<size_t>(recvlen) <= max_packet_length) {
#if defined(STREAM_UDP_DATA)
      udp_video_streamer->Send(std::string(kVideoStreamingAddr),
                               kVideoStreamingPort + ue_id, &rx_buffer[0u],
                               recvlen);
#endif

      if (kDebugMacReceiver) {
        std::printf("MacReceiver: Thread %zu, Data Bytes: %zu:%zu, Data:", tid,
                    recvlen, max_packet_length);
        for (size_t i = 0; i < static_cast<size_t>(recvlen); i++) {
          std::printf(" %02x", rx_buffer[i]);
        }
        std::printf("\n");
      }

      if (static_cast<size_t>(recvlen) != max_packet_length) {
        MLPD_INFO(
            "MacReceiver: Thread %zu received less than max data bytes "
            "%zu:%zu\n",
            tid, recvlen, max_packet_length);
      }
    } /*else if (recvlen != 0) {
      std::printf(
          "MacReceiver: Recv failed with less than requested bytes %zu\n",
          recvlen);
      throw std::runtime_error(
          "MacReceiver: recv failed with less than requested bytes");
    }*/
  }
  delete[] rx_buffer;
  std::printf("MacReceiver: Finished\n");
  return nullptr;
}
