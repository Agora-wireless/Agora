/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */

#ifndef RECEIVER
#define RECEIVER

#include "Symbols.hpp"
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "gettime.h"
#include "net.hpp"
#include "signalHandler.hpp"
#include <algorithm>
#include <arpa/inet.h>
#include <cassert>
#include <chrono>
#include <ctime>
#include <fcntl.h>
#include <iostream>
#include <mutex>
#include <netinet/in.h>
#include <numeric>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <vector>
#include <iomanip>
#include "ip_bridge.hpp"
#include "crc.hpp"

typedef unsigned short ushort;
class SimpleBSMac {
public:
    SimpleBSMac(Config* cfg, size_t rx_thread_num = 1, size_t core_offset = 1);

    ~SimpleBSMac();

    /**
     * Called in main threads to start the socket threads
     * in_buffer: ring buffer to save packets
     * in_buffer_status: record the status of each memory block (0: empty, 1:
     * full) in_buffer_frame_num: number of packets the ring buffer could hold
     * in_buffer_length: size of ring buffer
     * in_core_id: attach socket threads to {in_core_id, ..., in_core_id +
     * RX_THREAD_NUM - 1}
     */
    bool startRecv();

    /**
     * receive thread
     * context: SimpleBSMacContext type
     */
    void* loopRecv(int tid);

private:
    pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

    size_t rx_thread_num_;
    size_t tx_thread_num_;

    size_t core_id_;
    Config* cfg;

    // TUN interface
    IPbridge* ipbridge;
    unsigned char* data_to_tun;
    uint32_t tun_payload_size_bytes;

    // CRC
    DoCRC* crc_up;
};

#endif
