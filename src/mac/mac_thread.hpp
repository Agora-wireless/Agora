#pragma once

#include "Symbols.hpp"
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "crc.hpp"
#include "gettime.h"
#include "net.hpp"
#include "udp_client.h"
#include "udp_server.h"

/**
 * @brief The MAC thread that runs alongside the PHY processing at the Millipede
 * server or client.
 *
 * This thread receives UDP data packets from remote apps and forwards them to
 * Millipede. It receives decoded symbols from Millipede and forwards UDP data
 * packets to applications.
 */
class MacThread {
public:
    enum class Mode {
        kServer, // The MAC thread is running the the Millipede server
        kClient // The MAC thread is running at the Millipede client
    };

    // Default log file for MAC layer outputs
    const char* kDefaultLogFilename = "/tmp/mac_log";

    // After receiving decoded codeblocks from the PHY (uplink at the
    // server, downlink at the client), we send UDP packets to kRemoteHostname
    const char* kRemoteHostname = "127.0.0.1";

    // Millipede sends UDP packets for UE #i (uplink packets at the server,
    // downlink packets at the client) with destination port kBaseRemotePort + i
    static constexpr size_t kBaseRemotePort = 8080;

    // Millipede listens for UDP packets from applications (downlink packets at
    // the server, uplink packets at the client) on kLocalPort
    static constexpr size_t kLocalPort = 8070;

    // Maximum number of outstanding UDP packets per UE that we allocate recv()
    // buffer space for
    static constexpr size_t kMaxPktsPerUE = 64;

    MacThread(Mode mode, Config* cfg, size_t core_offset,
        PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t>& decoded_buffer,
        Table<uint8_t>* ul_bits_buffer, Table<uint8_t>* ul_bits_buffer_status,
        Table<uint8_t>* dl_bits_buffer, Table<uint8_t>* dl_bits_buffer_status,
        moodycamel::ConcurrentQueue<Event_data>* rx_queue,
        moodycamel::ConcurrentQueue<Event_data>* tx_queue,
        std::string log_filename = "");

    ~MacThread();

    // The main MAC thread event loop. It receives uplink data bits from the
    // master thread and sends them to remote applications.
    void run_event_loop();

private:
    // Receive decoded codeblocks from the PHY master thread. Send
    // fully-received frames for UE #i to kRemoteHostname::(kBaseRemotePort + i)
    void process_codeblocks_from_master();

    // Receive user data bits (downlink bits at the MAC thread running at the
    // server, uplink bits at the MAC thread running at the client) and forward
    // them to the PHY.
    void process_udp_packets_from_apps();
    void process_udp_packets_from_apps_server(const MacPacket* pkt);
    void process_udp_packets_from_apps_client(const char* pkt);

    // If Mode::kServer, this thread is running at the Millipede server. Else at
    // the client.
    const Mode mode_;
    Config* cfg_;

    const double freq_ghz_; // RDTSC frequency in GHz
    // We check for new MAC packets from applications every [tsc_delta_]
    // clock ticks
    const size_t tsc_delta_;

    const size_t core_offset_; // The CPU core on which this thread runs

    FILE* log_file_; // Log file used to store MAC layer outputs
    std::string log_filename_ = kDefaultLogFilename; // Name of the log file

    UDPClient* udp_client; // UDP endpoint used for sending messages
    UDPServer* udp_server; // UDP endpoint used for receiving messages

    // TODO: decoded_buffer_ is used by only the server, so it should be moved
    // to server_ for clarity.
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t>& decoded_buffer_;

    Table<uint8_t>* dl_bits_buffer_;
    Table<uint8_t>* dl_bits_buffer_status_;

    // A preallocated buffer to store UDP packets received via recv()
    std::vector<uint8_t> udp_pkt_buf_;

    // The timestamp at which we last received a UDP packet from an application
    size_t last_mac_pkt_rx_tsc_ = 0;

    // The frame ID of the next MAC packet we'll hand over to the PHY
    size_t next_frame_id_ = 0;

    // The radio ID of the next MAC packet we'll hand over to the PHY
    size_t next_radio_id_ = 0;

    FastRand fast_rand_;

    // Server-only members
    struct {
        // Staging buffers to accumulate decoded uplink code blocks for each UE
        std::vector<uint8_t> frame_data_[kMaxUEs];

        // n_filled_in_frame_[i] is the number of bytes received in the current
        // frame for UE #i
        std::array<size_t, kMaxUEs> n_filled_in_frame_;
    } server_;

    // Client-only members
    struct {
        // ul_bits_buffer_id_[i] is the index of the uplink data bits buffer to
        // next use for radio #i
        std::array<size_t, kMaxUEs> ul_bits_buffer_id_;

        Table<uint8_t>* ul_bits_buffer_;
        Table<uint8_t>* ul_bits_buffer_status_;
    } client_;

    // FIFO queue for receiving messages from the master thread
    moodycamel::ConcurrentQueue<Event_data>* rx_queue_;

    // FIFO queue for sending messages to the master thread
    moodycamel::ConcurrentQueue<Event_data>* tx_queue_;

    // CRC
    DoCRC* crc_obj;
};
