#pragma once

#include "Symbols.hpp"
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "gettime.h"
#include "net.hpp"
#include "udp_client.h"
#include "udp_server.h"

/**
 * @brief The MAC thread that runs alongside the PHY processing at the Millipede
 * server.
 *
 * On the uplink, this thread takes symbols decoded by Millipede and sends them
 * to higher-level applications.
 */
class MacThread {
public:
    // Default log file for MAC layer outputs
    const char* kDefaultLogFilename = "/tmp/millipede_mac_log";

    // Decoded uplink packets will be sent to kRemoteHostname
    const char* kRemoteHostname = "127.0.0.1";

    // We'll send decoded uplink packets for UE #i with destination port
    // number UDP port kBaseRemotePort + i
    static constexpr size_t kBaseRemotePort = 8080;

    // We'll listen for downlink data packets on UDP port kLocalPort
    static constexpr size_t kLocalPort = 8070;

    // Maximum number of outstanding downlink packets per UE that we allocate
    // buffer space for
    static constexpr size_t kMaxDLPktsPerUE = 64;

    MacThread(Config* cfg, size_t core_offset, Table<int8_t>* dl_bits_buffer,
        Table<int>* dl_bits_buffer_status, Table<uint8_t>* ul_bits_buffer,
        moodycamel::ConcurrentQueue<Event_data>* rx_queue,
        moodycamel::ConcurrentQueue<Event_data>* tx_queue,
        moodycamel::ProducerToken* rx_ptok, moodycamel::ProducerToken* tx_ptok,
        std::string log_filename = "");

    ~MacThread();

    // The main MAC thread event loop. It receives uplink data bits from the
    // master thread and sends them to remote applications.
    void run_event_loop();

private:
    // Receive decoded codeblocks from the PHY master thread. Send
    // fully-received frames for UE #i to kRemoteHostname::(kBaseRemotePort + i)
    void process_events_from_master();

    // Receive user bits and forward them to the PHY master thread
    void process_downlink_packets();

    Config* cfg_;
    const size_t core_offset_; // The CPU core on which this thread runs

    FILE* log_file_; // Log file used to store MAC layer outputs
    std::string log_filename_ = kDefaultLogFilename; // Name of the log file

    UDPClient udp_client; // UDP endpoint used for sending messages
    UDPServer udp_server; // UDP endpoint used for receiving messages

    Table<uint8_t>* ul_bits_buffer_; // Uplink bits decoded by the PHY

    // Staging buffers to accumulate decoded uplink code blocks for each UE
    std::vector<uint8_t> frame_data[kMaxUEs];

    std::vector<uint8_t> dl_pkt_buf; // Buffer to receive downlink packets

    // The number of bytes received in the current frame for each UE
    std::array<size_t, kMaxUEs> num_filled_bytes_in_frame;

    // Downlink: Not implemented yet
    Table<int8_t>* dl_bits_buffer_;
    Table<int>* dl_bits_buffer_status_;

    // FIFO queues for receiving messages from the master thread
    moodycamel::ConcurrentQueue<Event_data>* rx_queue_;
    moodycamel::ProducerToken* tx_ptok_;

    // FIFO queues for sending messages to the master thread
    moodycamel::ConcurrentQueue<Event_data>* tx_queue_;
    moodycamel::ProducerToken* rx_ptok_;
};
