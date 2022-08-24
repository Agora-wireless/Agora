/**
 * @file mac_thread.h
 * @brief Declaration file for the MacThreadClient class
 */
#ifndef MAC_THREAD_H_
#define MAC_THREAD_H_

#include <queue>

#include "concurrentqueue.h"
#include "config.h"
#include "crc.h"
#include "gettime.h"
#include "message.h"
#include "ran_config.h"
#include "symbols.h"
#include "udp_comm.h"
#include "udp_server.h"

/**
 * @brief The MAC thread that runs alongside the PHY processing at the Agora
 * server or client.
 *
 * This thread receives UDP data packets from remote apps and forwards them to
 * Agora. It receives decoded symbols from Agora and forwards UDP data
 * packets to applications.
 */
class MacThreadClient {
 public:
  // Default log file for MAC layer outputs
  static constexpr char kDefaultLogFilename[] = "data/mac_log_client";

  // Maximum number of outstanding UDP packets per UE that we allocate recv()
  // buffer space for
  static constexpr size_t kMaxPktsPerUE = 64;

  // Length of SNR moving average window
  // TODO: map this to time?
  static constexpr size_t kSNRWindowSize = 100;

  MacThreadClient(
      Config* const cfg, size_t core_offset,
      PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& decoded_buffer,
      Table<int8_t>* ul_bits_buffer, Table<int8_t>* ul_bits_buffer_status,
      moodycamel::ConcurrentQueue<EventData>* rx_queue,
      moodycamel::ConcurrentQueue<EventData>* tx_queue,
      const std::string& log_filename = "");

  ~MacThreadClient();

  // The main MAC thread event loop. It receives uplink data bits from the
  // master thread and sends them to remote applications.
  void RunEventLoop();

 private:
  // Receive events from Agora PHY master thread. Forwards
  // to appropriate function in MAC.
  void ProcessRxFromPhy();

  // Receive decoded codeblocks from the PHY master thread. Send
  // fully-received frames for UE #i to kRemoteHostname::(kBaseRemotePort + i)
  void ProcessCodeblocksFromPhy(EventData event);

  // Receive SNR report from PHY master thread. Use for RB scheduling.
  // TODO: process CQI report here as well.
  void ProcessSnrReportFromPhy(EventData event);

  // At client, process control information received from control
  // channel and forward to PHY UE, so it transmits data in the scheduled
  // time slots.
  void ProcessControlInformation();

  // Receive user data bits (downlink bits at the MAC thread running at the
  // server, uplink bits at the MAC thread running at the client) and forward
  // them to the PHY.
  void ProcessUdpPacketsFromApps(RBIndicator ri);
  void ProcessUdpPacketsFromAppsClient(const char* payload, RBIndicator ri);

  // If Mode::kServer, this thread is running at the Agora server. Else at
  // the client.
  // const Mode mode_;
  Config* const cfg_;

  const double freq_ghz_;  // RDTSC frequency in GHz
  // We check for new MAC packets from applications every [tsc_delta_]
  // clock ticks
  const size_t tsc_delta_;

  const size_t core_offset_;  // The CPU core on which this thread runs

  FILE* log_file_;  // Log file used to store MAC layer outputs
  std::string log_filename_;

  // UDP endpoint used for sending messages
  std::unique_ptr<UDPComm> udp_comm_;

  // UDP endpoint for receiving control channel messages
  std::unique_ptr<UDPServer> udp_control_channel_;

  // A preallocated buffer to store UDP packets received via recv()
  std::vector<std::byte> udp_pkt_buf_;

  // A preallocated buffer to store UDP control information
  // received via recv()
  std::vector<std::byte> udp_control_buf_;

  // The timestamp at which we last received a UDP packet from an application
  size_t last_mac_pkt_rx_tsc_ = 0;

  // The frame ID of the next MAC packet we'll hand over to the PHY
  size_t next_tx_frame_id_ = 0;

  // The radio ID of the next MAC packet we'll hand over to the PHY
  size_t next_radio_id_ = 0;

  // The frame ID of the next TTI that the scheduler plans for
  size_t scheduler_next_frame_id_ = 0;

  FastRand fast_rand_;

  // Server-only members
  struct {
    // Staging buffers to accumulate decoded uplink code blocks for each UE
    std::array<std::vector<std::byte>, kMaxUEs> frame_data_;

    // n_filled_in_frame_[i] is the number of bytes received in the current
    // frame for UE #i
    std::array<size_t, kMaxUEs> n_filled_in_frame_;

    // snr_[i] contains a moving window of SNR measurement for UE #i
    std::array<std::queue<float>, kMaxUEs> snr_;

    // Placing at the end because it is variable size based on configuration
    std::vector<std::vector<size_t>> data_size_;
  } server_;

  // TODO: decoded_buffer_ is used by only the server, so it should be moved
  // to server_ for clarity.
  PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& decoded_buffer_;

  struct {
    // ul_bits_buffer_id_[i] is the index of the uplink data bits buffer to
    // next use for radio #i
    std::array<size_t, kMaxUEs> ul_bits_buffer_id_;

    Table<int8_t>* ul_bits_buffer_;
    Table<int8_t>* ul_bits_buffer_status_;
  } client_;

  // FIFO queue for receiving messages from the master thread
  moodycamel::ConcurrentQueue<EventData>* rx_queue_;

  // FIFO queue for sending messages to the master thread
  moodycamel::ConcurrentQueue<EventData>* tx_queue_;

  // CRC
  std::unique_ptr<DoCRC> crc_obj_;
};

#endif  // MAC_THREAD_H_
