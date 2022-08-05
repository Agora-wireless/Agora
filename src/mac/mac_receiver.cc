/**
 * @file  mac_receiver.cc
 * @brief Declaration file for the simple mac receiver class
 */
#include "mac_receiver.h"

#include <utility>

#include "logger.h"
#include "signal_handler.h"
#include "udp_client.h"
#include "udp_server.h"

static const bool kDebugMacReceiver = true;
static const std::string kMacRxAddress = "";
static const std::string kMacTxAddress = "127.0.0.1";
static constexpr uint16_t kMacTxPort = 0;

MacReceiver::MacReceiver(Config* const cfg, size_t num_frame_data_bytes,
                         std::string phy_server_address, size_t phy_port,
                         size_t rx_thread_num, size_t core_offset)
    : data_bytes_(num_frame_data_bytes),
      phy_address_(std::move(phy_server_address)),
      phy_port_(phy_port),
      enable_udp_output_(false),
      udp_dest_port_(0),
      udp_dest_address_(""),
      rx_thread_num_(rx_thread_num),
      core_id_(core_offset),
      cfg_(cfg) {}

MacReceiver::MacReceiver(Config* const cfg, size_t num_frame_data_bytes,
                         std::string phy_server_address, size_t phy_port,
                         std::string fwd_data_udp_address, size_t fwd_port,
                         size_t rx_thread_num, size_t core_offset)
    : data_bytes_(num_frame_data_bytes),
      phy_address_(std::move(phy_server_address)),
      phy_port_(phy_port),
      enable_udp_output_(true),
      udp_dest_port_(fwd_port),
      udp_dest_address_(std::move(fwd_data_udp_address)),
      rx_thread_num_(rx_thread_num),
      core_id_(core_offset),
      cfg_(cfg) {}

std::vector<std::thread> MacReceiver::StartRecv() {
  std::vector<std::thread> created_threads;

  AGORA_LOG_INFO("MacReceiver:  Start Recv threads %zu\n", rx_thread_num_);
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

  static constexpr size_t kSockBufSize = (1024 * 1024 * 64 * 8) - 1;
  auto udp_server = std::make_unique<UDPServer>(
      kMacRxAddress, phy_port_ + ue_id, kSockBufSize);

  std::unique_ptr<UDPClient> udp_streamer;
  if (enable_udp_output_) {
    udp_streamer = std::make_unique<UDPClient>(kMacTxAddress, kMacTxPort);
  }

  udp_server->MakeBlocking(1);
  AGORA_LOG_INFO(
      "MacReceiver: Set up UDP socket server listening to port %zu\n",
      phy_port_ + ue_id);

  // Create a rx buffer
  const size_t max_packet_length = data_bytes_;
  auto* rx_buffer = new std::byte[max_packet_length];

  while ((SignalHandler::GotExitSignal() == false) &&
         (cfg_->Running() == true)) {
    const ssize_t recvlen = udp_server->Recv(phy_address_, phy_port_ + ue_id,
                                             &rx_buffer[0u], max_packet_length);
    if (recvlen < 0) {
      std::perror("recv failed");
      throw std::runtime_error("Receiver: recv failed");
    } else if ((recvlen > 0) &&
               static_cast<size_t>(recvlen) <= max_packet_length) {
      if (enable_udp_output_) {
        udp_streamer->Send(udp_dest_address_, udp_dest_port_ + ue_id,
                           &rx_buffer[0u], recvlen);
      }

      if (kDebugMacReceiver) {
        AGORA_LOG_INFO("MacReceiver: Thread %zu, Data Bytes: %zu:%zu, Data:",
                       tid, recvlen, max_packet_length);
        for (size_t i = 0; i < static_cast<size_t>(recvlen); i++) {
          AGORA_LOG_INFO(" %02x", static_cast<uint8_t>(rx_buffer[i]));
        }
        AGORA_LOG_INFO("\n");
      }

      if (static_cast<size_t>(recvlen) != max_packet_length) {
        AGORA_LOG_INFO(
            "MacReceiver: Thread %zu received less than max data bytes "
            "%zu:%zu\n",
            tid, recvlen, max_packet_length);
      }
    }
  }
  delete[] rx_buffer;
  AGORA_LOG_INFO("MacReceiver: Finished\n");
  return nullptr;
}
