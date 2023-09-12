/*
 * @file rp_thread.h
 * @brief Declaration file for the ResourceProvisionerThread class
 */
#ifndef RP_THREAD_H_
#define RP_THREAD_H_

#include <queue>

#include "agora_buffer.h"
#include "concurrentqueue.h"
#include "config.h"
#include "crc.h"
#include "gettime.h"
#include "ran_config.h"
#include "symbols.h"
#include "udp_client.h"
#include "udp_server.h"
#include "rp_config.h"

/**
 * @brief The RP thread that runs alongside the PHY processing at the Agora
 * server or client.
 *
 * This thread receives UDP data packets from remote RP app and forwards them to
 * Agora. It receives control messages from Agora and forwards UDP data
 * packets to RP app.
 */
class ResourceProvisionerThread {
 public:
  // Default log file for RP layer outputs
  static constexpr char kDefaultLogFilename[] = "files/config/examples/rp_log_server.txt";
  static constexpr size_t kUdpRxBufferPadding = 2048u;

  ResourceProvisionerThread(
      Config* const cfg, size_t core_offset,
      moodycamel::ConcurrentQueue<EventData>* rx_queue,
      moodycamel::ConcurrentQueue<EventData>* tx_queue,
      const std::string& log_filename = "");

  ~ResourceProvisionerThread();

  // The main RP thread event loop. It receives uplink data bits from the
  // master thread and sends them to remote applications.
  void RunEventLoop();

 private:
  // Communicate with RP via UDP
  void RequestEventFromAgora();
  void SendEventToAgora(const char* payload);
  void ReceiveUdpPacketsFromRp();
  void ReceiveEventFromAgora();
  void SendUdpPacketsToRp(EventData event);

  Config* const cfg_;

  const double freq_ghz_;  // RDTSC frequency in GHz
  const size_t tsc_delta_; // Clock ticks, we check for new packets from applications every [tsc_delta_]
  const size_t core_offset_;  // The CPU core on which this thread runs

  FILE* log_file_;  // Log file used to store MAC layer outputs
  std::string log_filename_;

  // UDP endpoint used for sending & receiving messages
  // std::unique_ptr<UDPClient> udp_client_;
  // std::unique_ptr<UDPServer> udp_server_;
  std::unique_ptr<UDPComm> udp_comm_;

  // A preallocated buffer to store UDP packets received via recv()
  std::vector<std::byte> udp_pkt_buf_;

  // FIFO queue for receiving & sending messages from the master thread
  moodycamel::ConcurrentQueue<EventData>* rx_queue_;
  moodycamel::ConcurrentQueue<EventData>* tx_queue_;
};

#endif  // RP_THREAD_H_