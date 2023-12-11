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

  WiredControlChannel(Config* cfg, size_t tx_core_offset, size_t rx_core_offset,
                      std::string local_addr, size_t local_port,
                      std::string remote_addr, size_t remote_port,
                      moodycamel::ConcurrentQueue<EventData>* rx_queue,
                      moodycamel::ConcurrentQueue<EventData>* tx_queue,
                      moodycamel::ProducerToken* rx_producer,
                      moodycamel::ProducerToken* tx_producer,
                      char* rx_socket_buf, size_t rx_pkt_num,
                      const std::string& log_filename = "");

  ~WiredControlChannel();

  // The main thread event loops. It receives uplink data bits from the
  // remote applicationsand sends them to the master thread.
  void RunRxEventLoop();
  void RunTxEventLoop();

 private:
  // Communicate with Remote Phy via UDP
  void SendPacketToRemotePhy(EventData event);
  bool SendEventToLocalPhy(RxPacket& data);
  bool ReceivePacketFromRemotePhy();
  bool ReceiveEventFromLocalPhy();
  RxPacket& GetRxPacket();
  void ReturnRxPacket(RxPacket& unused_packet);

  Config* const cfg_;

  // RDTSC frequency in GHz
  const double freq_ghz_;
  // Clock ticks, we check for new packets from applications every [tsc_delta_]
  const size_t tsc_delta_;
  // The CPU core on which this thread runs
  const size_t rx_core_offset_;
  const size_t tx_core_offset_;

  // Log file used to store RP layer outputs
  FILE* log_file_;
  std::string log_filename_;

  // UDP endpoint used for sending & receiving messages
  std::unique_ptr<UDPComm> udp_comm_;

  // A preallocated buffer to store UDP packets received via recv()
  std::vector<std::byte> udp_pkt_buf_;
  std::vector<RxPacket> rx_packets_;
  char* rx_socket_buf_;
  size_t pkt_tracker_;
  size_t max_pkt_cnt_;

  // FIFO queue for receiving & sending messages from the master thread
  moodycamel::ConcurrentQueue<EventData>* rx_queue_;
  moodycamel::ConcurrentQueue<EventData>* tx_queue_;
  moodycamel::ProducerToken* rx_producer_;
  moodycamel::ProducerToken* tx_producer_;
};

#endif  // USERWIREDCHANNEL_H_
