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
class UserWiredChannel {
 public:
  // Default log file for RP layer outputs
  static constexpr char kDefaultLogFilename[] =
      "files/experiment/rp_log_server.txt";
  static constexpr size_t kUdpRxBufferPadding = 2048u;

  UserWiredChannel(Config* const cfg, size_t core_offset,
                   moodycamel::ConcurrentQueue<EventData>* rx_queue,
                   moodycamel::ConcurrentQueue<EventData>* tx_queue,
                   const std::string& log_filename = "");

  ~UserWiredChannel();

  // The main RP thread event loop. It receives uplink data bits from the
  // master thread and sends them to remote applications.
  void RunEventLoop();
  RxPacket* ReceiveUdpPacketsFromClient();

 private:
  // Communicate with RP via UDP
  void SendEventToAgora(RxPacket* rxpkt);
  void ReceiveEventFromAgora();
  void SendUdpPacketsToClient(EventData event);

  Config* const cfg_;

  // RDTSC frequency in GHz
  const double freq_ghz_;
  // Clock ticks, we check for new packets from applications every [tsc_delta_]
  const size_t tsc_delta_;
  // The CPU core on which this thread runs
  const size_t core_offset_;

  // Log file used to store RP layer outputs
  FILE* log_file_;
  std::string log_filename_;

  // UDP endpoint used for sending & receiving messages
  std::unique_ptr<UDPComm> udp_comm_;

  // A preallocated buffer to store UDP packets received via recv()
  std::vector<RxPacket*> udp_pkt_buf_;
  size_t pkt_tracker_;

  // FIFO queue for receiving & sending messages from the master thread
  moodycamel::ConcurrentQueue<EventData>* rx_queue_;
  moodycamel::ConcurrentQueue<EventData>* tx_queue_;
};

#endif  // USERWIREDCHANNEL_H_
