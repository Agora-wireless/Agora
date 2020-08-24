
#ifndef RadioTXRX_HEADER
#define RadioTXRX_HEADER

#include "client_radio.hpp"
#include "concurrentqueue.h"
#include "utils.h"
#include <arpa/inet.h>
#include <complex>
#include <fstream>
#include <netinet/in.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <vector>

class RadioConfig;

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
    int dequeue_send(int tid);
    void* loopTXRX(int tid);
    void* loopSYNC_TXRX(int tid);

private:
    pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
    Config* config_;
    ClientRadioConfig* radioconfig_; // Used only in Argos mode
    struct sockaddr_in servaddr_[10]; /* server address */
    struct sockaddr_in servaddr_tx_[10]; /* server address for tx*/
    struct sockaddr_in cliaddr_[10]; /* client address */
    int* rx_socket_;
    int* tx_socket_;

    Table<char>* buffer_;
    Table<int>* buffer_status_;
    int buffer_length_;
    int buffer_frame_num_;

    char* tx_buffer_;
    char* pilot_buffer_;
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
