
#ifndef RadioTXRX_HEADER
#define RadioTXRX_HEADER

#include "client_radio.hpp"
#include "concurrentqueue.h"
#include "utils.h"
#include "net.hpp"
#include <arpa/inet.h>
#include <complex>
#include <fstream>
#include <fcntl.h>
#include <netinet/in.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <vector>

class RadioConfig;

class RadioTXRX {
public:
    // static const int OFDM_FRAME_LEN = SYMBOL_LEN; //OFDM_CA_NUM +
    // OFDM_PREFIX_LEN + OFDM_POSTFIX_LEN; header 4 int for: frame_id,
    // symbol_id, cell_id, ant_id ushort for: I/Q samples static const int
    // package_length = sizeof(int) * 4 + sizeof(ushort) * OFDM_FRAME_LEN * 2;
    // use for create pthread
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
     * called in main threads to start the socket threads
     * in_buffer: ring buffer to save packets
     * in_buffer_status: record the status of each memory block (0: empty, 1:
     * full) in_buffer_frame_num: number of packets the ring buffer could hold
     * in_buffer_length: size of ring buffer
     * in_core_id: attach socket threads to {in_core_id, ..., in_core_id +
     * N_THREAD - 1}
     */
    bool startTXRX(Table<char>& in_buffer, Table<int>& in_buffer_status,
        int in_buffer_frame_num, int in_buffer_length, char* in_tx_buffer,
        int* in_tx_buffer_status, int in_tx_buffer_frame_num,
        int in_tx_buffer_length);
    /**
     * receive thread
     * context: PackageReceiverContext type
     */
    struct Packet* recv_enqueue(int tid, int radio_id, int rx_offset);
    int dequeue_send(int tid);
    int dequeue_send_argos(int tid);
    void* loop_tx_rx(int tid);
    void* loop_tx_rx_argos(int tid);
    void* loop_tx_rx_argos_sync(int tid);

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
};
#endif
