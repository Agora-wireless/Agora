#pragma once

#include "Symbols.hpp"
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "gettime.h"
#include "net.hpp"

/**
 * @brief The MAC thread that runs alongside the PHY processing at the Millipede
 * server.
 */
class MacThread {
public:
    MacThread(Config* cfg, size_t core_offset, Table<int8_t>* dl_bits_buffer,
        Table<int>* dl_bits_buffer_status, Table<uint8_t>* ul_bits_buffer,
        moodycamel::ConcurrentQueue<Event_data>* rx_queue,
        moodycamel::ConcurrentQueue<Event_data>* tx_queue,
        moodycamel::ProducerToken* rx_ptok, moodycamel::ProducerToken* tx_ptok);

    ~MacThread();

    void run_event_loop();

private:
    Config* cfg_;
    const size_t core_offset_;

    Table<int8_t>* dl_bits_buffer_;
    Table<int>* dl_bits_buffer_status_;
    Table<uint8_t>* ul_bits_buffer_;

    moodycamel::ConcurrentQueue<Event_data>* rx_queue_;
    moodycamel::ProducerToken* rx_ptok_;

    moodycamel::ConcurrentQueue<Event_data>* tx_queue_;
    moodycamel::ProducerToken* tx_ptok_;
};
