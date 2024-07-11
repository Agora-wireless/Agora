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

static const bool kDebugMacReceiver = false;
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

  AGORA_LOG_INFO("MacReceiver: Starting %zu Recv thread(s) @ core %zu\n",
                 rx_thread_num_, core_id_);
  created_threads.resize(rx_thread_num_);

  for (size_t i = 0; i < rx_thread_num_; i++) {
    created_threads.at(i) = std::thread(&MacReceiver::LoopRecv, this, i);
  }
  return created_threads;
}

void* MacReceiver::LoopRecv(size_t tid) {
  // TODO: Temp
  const size_t ue_id = tid;
  PinToCoreWithOffset(ThreadType::kWorkerRX, core_id_ + 1, tid);

  static constexpr size_t kSockBufSize = (1024 * 1024 * 64 * 8) - 1;
  auto udp_server = std::make_unique<UDPServer>(
      kMacRxAddress, phy_port_ + ue_id, kSockBufSize);

  std::unique_ptr<UDPClient> udp_streamer;
  std::ofstream create_file;
  if (enable_udp_output_) {
    udp_streamer = std::make_unique<UDPClient>(kMacTxAddress, kMacTxPort);
  } else {
    std::string data_filename =
        TOSTRING(PROJECT_DIRECTORY) +
        std::string("/files/experiment/rx_ul_increment_file.bin");
    AGORA_LOG_INFO(
        "Generating test binary file for user uplink %s.  Frames: "
        "%zu, Bytes per frame: %zu\n",
        data_filename.c_str(), cfg_->FramesToTest(), data_bytes_);

    create_file.open(
        data_filename,
        (std::ofstream::out | std::ofstream::binary | std::ofstream::trunc));
    assert(create_file.is_open() == true);
  }

  udp_server->MakeBlocking(1);
  AGORA_LOG_INFO(
      "MacReceiver[%zu]: Set up UDP socket server listening to port %zu\n", tid,
      phy_port_ + ue_id);

  // Create a rx buffer
  auto* rx_buffer = new std::byte[data_bytes_];

  while ((SignalHandler::GotExitSignal() == false) &&
         (cfg_->Running() == true)) {
    const ssize_t recvlen = udp_server->Recv(phy_address_, phy_port_ + ue_id,
                                             &rx_buffer[0u], data_bytes_);
    if (recvlen < 0) {
      std::perror("recv failed");
      throw std::runtime_error("MacReceiver: recv failed");
    } else if ((recvlen > 0) && static_cast<size_t>(recvlen) <= data_bytes_) {
      if (enable_udp_output_) {
        udp_streamer->Send(udp_dest_address_, udp_dest_port_ + ue_id,
                           &rx_buffer[0u], recvlen);
      } else {
        create_file.write(reinterpret_cast<char*>(rx_buffer), data_bytes_);
      }

      if (kDebugMacReceiver) {
        AGORA_LOG_INFO("MacReceiver[%zu]: Data Bytes: %zu:%zu, Data:", tid,
                       recvlen, data_bytes_);
        for (size_t i = 0; i < static_cast<size_t>(recvlen); i++) {
          AGORA_LOG_INFO(" %02x", static_cast<uint8_t>(rx_buffer[i]));
        }
        AGORA_LOG_INFO("\n");
      }

      if (static_cast<size_t>(recvlen) != data_bytes_) {
        AGORA_LOG_INFO(
            "MacReceiver[%zu]: received less than max data bytes %zu:%zu\n",
            tid, recvlen, data_bytes_);
      }
    }
  }
  if (!enable_udp_output_) {
    create_file.close();
  }
  delete[] rx_buffer;
  AGORA_LOG_INFO("MacReceiver[%zu]: Finished\n", tid);
  return nullptr;
}
