#ifndef RadioTXRX_HEADER
#define RadioTXRX_HEADER

#include "client_radio.hpp"
#include "concurrentqueue.h"
#include "datatype_conversion.h"
#include "net.hpp"
#include "utils.h"
#include <arpa/inet.h>
#include <cmath>
#include <complex>
#include <fcntl.h>
#include <fstream>
#include <netinet/in.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <vector>

class RadioConfig;

/**
 * @brief Implementations of this class provide packet I/O for Agora clients (UEs).
 *
 * In the vanilla mode, this class provides socket packet I/O for communicating
 * with channel simulator.
 *
 * In the "Argos" mode, this class uses SoapySDR-based functions (from client_radio.cpp)
 * to communicate with Agora client wireless hardware. Since clients require over-the-air
 * time synchronization with Agora BS, this class provides two types of functions
 * for this purpose:
 * 1) TX/RX with hardware-offloaded synchronization (loop_tx_rx_argos) where sync happens
 *    in the wireless hardware, where only downlink symbols' samples are received (guard
 *    symbols are discarded in hardware) and uplink symbols are sent to the wireless
 *    hardware to be sent at the correct time w.r.t BS.
 * 2) software-based syncronization (loop_tx_rx_argos_sync) where all of sychronization
 *    and time-keeping w.r.t. BS happens in this function as well as downlink and uplink
 *    receive and transmit functions.
 */
class RadioTXRX {
public:
    struct RadioTXRXContext {
        RadioTXRX* ptr;
        int tid;
    };

public:
    RadioTXRX(Config* cfg, int n_rx_thread, int in_core_id);
    /**
     * N_THREAD: socket thread number
     * mode: tx=1 or rx=0 operation
     * in_queue: message queue to communicate with main thread
     */
    RadioTXRX(Config* cfg, int n_tx_thread, int in_core_id,
        moodycamel::ConcurrentQueue<Event_data>* in_queue,
        moodycamel::ConcurrentQueue<Event_data>* in_queue_task,
        moodycamel::ProducerToken** in_rx_ptoks,
        moodycamel::ProducerToken** in_tx_ptoks);
    ~RadioTXRX();

    /**
     * @brief called in main threads to start the socket threads
     *
     * @param in_buffer: rx ring buffer to save received packets
     *
     * @param in_buffer_status: record the status of each memory block (0: empty, 1:
     * full)
     *
     * @param in_buffer_frame_num: number of sample packets (one symbol per packet)
     * the ring buffer could hold
     *
     * @param in_buffer_length: size of ring buffer in bytes
     *
     * @param in_tx_buffer: tx ring buffer of samples processed by PHY for transmission
     *
     * @param in_buffer_frame_num: number of samples packets (one symbol per packet)
     * the tx ring buffer could hold
     *
     * @param in_buffer_length: size of ring buffer in bytes
     */
    bool startTXRX(Table<char>& in_buffer, Table<int>& in_buffer_status,
        int in_buffer_frame_num, int in_buffer_length, char* in_tx_buffer,
        int* in_tx_buffer_status, int in_tx_buffer_frame_num,
        int in_tx_buffer_length);

    /**
     * @brief receives a packet from channel simulator through socket index (radio_id)
     * and writes to an offset (rx_offset) in the receive buffer (buffer_)
     */
    struct Packet* recv_enqueue(int tid, int radio_id, int rx_offset);

    /**
     * @brief transmits a tx packet that is ready from PHY through socket to channel simualtor
     */
    int dequeue_send(int tid);

    /**
     * @brief receives a packet from hardware through radio index (radio_id)
     * and writes to an offset (rx_offset) in the receive buffer (buffer_)
     */
    struct Packet* recv_enqueue_argos(int tid, size_t radio_id,
        size_t& frame_id, size_t& symbol_id, size_t rx_offset);

    /**
     * @brief transmits a tx samples packet that is ready from PHY through client wireless hardware
     */
    int dequeue_send_argos(int tid, long long time0);

    /**
     * @brief loop thread function that performs sample Packet I/O in simulation mode.
     */
    void* loop_tx_rx(int tid);

    /**
     * @brief loop thread function that performs sample TX/RX to/from client wireless hardware
     * where the time synchornization with BS occurs in the hardware. The fucntion reads timestamps
     * from received symbols and uses the timestamps to schedules transmit of processed uplink symbols
     * with some offset in the future from client wireless hardware.
     */
    void* loop_tx_rx_argos(int tid);

    /**
     * @brief loop thread function that performs sample TX/RX to/from client wireless hardware
     * The function performs correlation on the received samples first to find a beacon transmitted
     * by the BS continuously. Once found locks to that beacon as the time reference for the start
     * of BS frames. It tries to re-lock to the beacon every 1000 frames in case of time drifts.
     * Based on the beacon time reference, it writes downlink symbols to rx buffers and schedules
     * uplink symbols for transmission from the hardware.
     */
    void* loop_tx_rx_argos_sync(int tid);
    void* loop_tx_rx_usrp_sync(int tid);

    /**
     *
     *
     */
    void cfo_estimation(const int sync_index,
        const std::vector<std::complex<float>>& beacon_buff);
    complex_float* cfo_correction(bool downlink, complex_float* samples_vec, size_t len);
    double get_cfo(){ return cfo_; };

private:
    pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
    Config* config_;
    ClientRadioConfig* radioconfig_; // Used only in Argos mode
    std::vector<struct sockaddr_in> servaddr_; /* server address */
    std::vector<int> socket_;

    Table<char>* buffer_;
    Table<int>* buffer_status_;
    int buffer_length_;
    int buffer_frame_num_;

    char* tx_buffer_;
    int* tx_buffer_status_;
    int tx_buffer_frame_num_;
    int tx_buffer_length_;

    int thread_num_;
    int tx_thread_num_;
    // pointer of message_queue_
    moodycamel::ConcurrentQueue<Event_data>* message_queue_;
    moodycamel::ConcurrentQueue<Event_data>* task_queue_;
    moodycamel::ProducerToken** rx_ptoks_;
    moodycamel::ProducerToken** tx_ptoks_;
    int core_id_;
    int tx_core_id_;

    // helper buffers
    std::vector<void*> pilot_buff0;
    std::vector<void*> pilot_buff1;

    // CFO
    double cfo_;
};
#endif
