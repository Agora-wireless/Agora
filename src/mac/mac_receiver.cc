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
#include <gflags/gflags.h>

DEFINE_string(rx_file, "", "Output result file for throughput");

static const bool kDebugMacReceiver = true;

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

  // Throughput: init
  tp = new Throughput(rx_thread_num_, FLAGS_rx_file.c_str());

  MLPD_INFO("MacReceiver:  Start Recv threads %zu\n", rx_thread_num_);
  created_threads.resize(rx_thread_num_);

  for (size_t i = 0; i < rx_thread_num_; i++) {
    created_threads.at(i) = std::thread(&MacReceiver::LoopRecv, this, i);
  }
  return created_threads;
}

void* MacReceiver::LoopRecv(size_t tid) {
  // one user per tid for now
  const size_t ue_id = tid;
  const size_t core_offset = core_id_ + rx_thread_num_;
  PinToCoreWithOffset(ThreadType::kWorkerRX, core_offset, tid);

  static constexpr size_t kSockBufSize = (1024 * 1024 * 64 * 8) - 1;
  auto udp_server =
      std::make_unique<UDPServer>(phy_port_ + ue_id, kSockBufSize);

  std::unique_ptr<UDPClient> udp_streamer;
  if (enable_udp_output_) {
    udp_streamer = std::make_unique<UDPClient>();
  }

  MLPD_INFO(
      "MacReceiver[%zu]: Set up UDP socket server listening to port %zu\n", tid,
      phy_port_ + ue_id);

  // Create a rx buffer
  const size_t max_packet_length = data_bytes_;
  auto* rx_buffer = new uint8_t[max_packet_length];

  while ((SignalHandler::GotExitSignal() == false) &&
         (cfg_->Running() == true)) {
    const ssize_t recvlen = udp_server->RecvFrom(
        &rx_buffer[0u], max_packet_length, phy_address_, phy_port_ + ue_id);
    
    // Throughput: timestamp here
    tp->Stamp(tid, recvlen, 0, ue_id);

    if (recvlen < 0) {
      std::perror("MacReceiver: recv failed");
      throw std::runtime_error("MacReceiver: recv failed");
    } else if ((recvlen > 0) &&
               static_cast<size_t>(recvlen) <= max_packet_length) {
      if (enable_udp_output_) {
        udp_streamer->Send(udp_dest_address_, udp_dest_port_ + ue_id,
                           &rx_buffer[0u], recvlen);
      }

      if (kDebugMacReceiver) {
        std::stringstream ss;
        ss << "MacReceiver[" << tid << "]: Data Bytes: " << recvlen << ":"
           << max_packet_length;

        ss << std::hex << std::setfill('0');
        for (size_t i = 0; i < static_cast<size_t>(recvlen); i++) {
          ss << " 0x" << std::setw(2) << static_cast<int>(rx_buffer[i]);
        }
        ss << std::dec;
        std::printf("%s\n", ss.str().c_str());
      }

      if (static_cast<size_t>(recvlen) != max_packet_length) {
        MLPD_INFO(
            "MacReceiver[%zu]: received less than max data bytes "
            "%zu:%zu\n",
            tid, recvlen, max_packet_length);
      }
    }
  }
  
  // Throughput: free memory
  if (tid == 0) {
    delete tp;
  }
  // Throughput

  delete[] rx_buffer;
  std::printf("MacReceiver[%zu]: Finished\n", tid);
  return nullptr;
}
