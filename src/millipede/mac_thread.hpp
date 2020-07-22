#pragma once

#include "Symbols.hpp"
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "gettime.h"
#include "net.hpp"
#include "udp_client.h"

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

    // Decoded uplink packets for UE #i will be sent with destination UDP port
    // kBaseRemotePort + i
    static constexpr size_t kBaseRemotePort = 8080;

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
    Config* cfg_;
    const size_t core_offset_; // The CPU core on which this thread runs

    FILE* log_file_; // Log file used to store MAC layer outputs
    std::string log_filename_ = kDefaultLogFilename; // Name of the log file

    UDPClient udp_client; // UDP endpoint used for sending messages

    Table<uint8_t>* ul_bits_buffer_; // Uplink bits decoded by the PHY

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
