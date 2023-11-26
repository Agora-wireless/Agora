/*
 * @file client_comm.h
 * @brief Declaration file for the Client Wired Comm class
 */
#ifndef USERWIREDCHANNEL_H_
#define USERWIREDCHANNEL_H_

#include <cstddef>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "concurrentqueue.h"
#include "config.h"
#include "message.h"
#include "udp_comm.h"

/**
 * @brief The Client Listen thread that runs alongside the PHY processing at the Agora
 * server.
 *
 * This thread receives UDP data/stats packets from remote user app and forwards them to
 * Agora. It receives control messages from agora and forwards UDP data
 * packets to user app.
 */
class WiredControlChannel {
 public:
  // Default log file for RP layer outputs
  static constexpr char kDefaultLogFilename[] =
      "files/experiment/wcc_log_server.txt";
  static constexpr size_t kUdpRxBufferPadding = 2048u;

  WiredControlChannel(Config* cfg, size_t tx_core_offset, size_t rx_core_offset,
                      std::string local_addr, size_t local_port,
                      std::string remote_addr, size_t remote_port,
                      moodycamel::ConcurrentQueue<EventData>* rx_queue,
                      moodycamel::ConcurrentQueue<EventData>* tx_queue,
                      const std::string& log_filename = "");

  ~WiredControlChannel();

  // The main RP thread event loop. It receives uplink data bits from the
  // master thread and sends them to remote applications.
  void RunTxEventLoop();
  void RunRxEventLoop();

 private:
  // Communicate with RP via UDP
  void SendEventToLocalPhy(std::byte* data);
  void ReceivePacketFromRemotePhy();
  void ReceiveEventFromLocalPhy();
  void SendPacketToRemotePhy(EventData event);
  inline std::byte* GetNextPacketBuff() {
    return rx_socket_buf_[(pkt_tracker_++) % kFrameWnd];
  }

  Config* const cfg_;

  // RDTSC frequency in GHz
  const double freq_ghz_;
  // Clock ticks, we check for new packets from applications every [tsc_delta_]
  const size_t tsc_delta_;
  // The CPU core on which this thread runs
  const size_t tx_core_offset_;
  const size_t rx_core_offset_;

  // Log file used to store RP layer outputs
  FILE* log_file_;
  std::string log_filename_;

  // UDP endpoint used for sending & receiving messages
  std::unique_ptr<UDPComm> udp_comm_;

  // A preallocated buffer to store UDP packets received via recv()
  std::vector<std::byte> udp_pkt_buf_;
  Table<std::byte> rx_socket_buf_;
  size_t pkt_tracker_;

  // FIFO queue for receiving & sending messages from the master thread
  moodycamel::ConcurrentQueue<EventData>* rx_queue_;
  moodycamel::ConcurrentQueue<EventData>* tx_queue_;
};

#endif  // USERWIREDCHANNEL_H_
