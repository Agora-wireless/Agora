/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */

#include "txrx.hpp"

PacketTXRX::PacketTXRX(Config* cfg, int COMM_THREAD_NUM, int in_core_offset)
{
    config_ = cfg;
    comm_thread_num_ = COMM_THREAD_NUM;

    core_id_ = in_core_offset;
    tx_core_id_ = in_core_offset + COMM_THREAD_NUM;

    /* initialize random seed: */
    srand(time(NULL));

    radioconfig_ = new RadioConfig(config_);
}

PacketTXRX::PacketTXRX(Config* cfg, int COMM_THREAD_NUM, int in_core_offset,
    moodycamel::ConcurrentQueue<Event_data>* in_queue_message,
    moodycamel::ConcurrentQueue<Event_data>* in_queue_task,
    moodycamel::ProducerToken** in_rx_ptoks,
    moodycamel::ProducerToken** in_tx_ptoks)
    : PacketTXRX(cfg, COMM_THREAD_NUM, in_core_offset)
{
    message_queue_ = in_queue_message;
    task_queue_ = in_queue_task;
    rx_ptoks_ = in_rx_ptoks;
    tx_ptoks_ = in_tx_ptoks;
}

PacketTXRX::~PacketTXRX()
{
    radioconfig_->radioStop();
    delete radioconfig_;
}

bool PacketTXRX::startTXRX(Table<char>& in_buffer, Table<int>& in_buffer_status,
    int in_buffer_frame_num, long long in_buffer_length,
    Table<double>& in_frame_start, char* in_tx_buffer, int* in_tx_buffer_status,
    int in_tx_buffer_frame_num, int in_tx_buffer_length)
{
    buffer_ = &in_buffer; // for save data
    buffer_status_ = &in_buffer_status; // for save status
    frame_start_ = &in_frame_start;

    // check length
    buffer_frame_num_ = in_buffer_frame_num;
    // assert(in_buffer_length == packet_length * buffer_frame_num_); // should
    // be integer
    buffer_length_ = in_buffer_length;
    tx_buffer_ = in_tx_buffer; // for save data
    tx_buffer_status_ = in_tx_buffer_status; // for save status
    tx_buffer_frame_num_ = in_tx_buffer_frame_num;
    // assert(in_tx_buffer_length == packet_length * buffer_frame_num_); //
    // should be integer
    tx_buffer_length_ = in_tx_buffer_length;
    // new thread
    // pin_to_core_with_offset(RX, core_id_, 0);

    if (!radioconfig_->radioStart())
        return false;

    printf("create TXRX threads\n");
    for (int i = 0; i < comm_thread_num_; i++) {
        pthread_t txrx_thread;
        auto context = new EventHandlerContext<PacketTXRX>;
        context->obj_ptr = this;
        context->id = i;
        if (pthread_create(&txrx_thread, NULL,
                pthread_fun_wrapper<PacketTXRX, &PacketTXRX::loopTXRX>, context)
            != 0) {
            perror("socket communication thread create failed");
            exit(0);
        }
    }

    sleep(1);
    pthread_cond_broadcast(&cond);
    // sleep(1);
    radioconfig_->go();
    return true;
}

void* PacketTXRX::loopTXRX(int tid)
{
    pin_to_core_with_offset(ThreadType::kWorkerTXRX, core_id_, tid);
    double* rx_frame_start = (*frame_start_)[tid];
    int rx_offset = 0;
    int radio_lo = tid * config_->nRadios / comm_thread_num_;
    int radio_hi = (tid + 1) * config_->nRadios / comm_thread_num_;
    printf("receiver thread %d has %d radios\n", tid, radio_hi - radio_lo);

    //// Use mutex to sychronize data receiving across threads
    pthread_mutex_lock(&mutex);
    printf("Thread %d: waiting for release\n", tid);
    pthread_cond_wait(&cond, &mutex);
    pthread_mutex_unlock(&mutex); // unlocking for all other threads

#if MEASURE_TIME
    int prev_frame_id = -1;
#endif
    int radio_id = radio_lo;
    while (config_->running) {
        if (-1 != dequeue_send(tid))
            continue;
        // receive data
        struct Packet* pkt = recv_enqueue(tid, radio_id, rx_offset);
        if (pkt == NULL)
            continue;
        rx_offset = (rx_offset + config_->nChannels) % buffer_frame_num_;
#if MEASURE_TIME
        int frame_id = pkt->frame_id;
        if (frame_id > prev_frame_id) {
            *(rx_frame_start + frame_id) = get_time();
            prev_frame_id = frame_id;
            if (frame_id % 512 == 200) {
                _mm_prefetch(
                    (char*)(rx_frame_start + frame_id + 512), _MM_HINT_T0);
            }
        }
#endif

        if (++radio_id == radio_hi)
            radio_id = radio_lo;
    }
    return 0;
}

struct Packet* PacketTXRX::recv_enqueue(int tid, int radio_id, int rx_offset)
{
    moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];
    char* rx_buffer = (*buffer_)[tid];
    int* rx_buffer_status = (*buffer_status_)[tid];
    int packet_length = config_->packet_length;

    // if rx_buffer is full, exit
    int nChannels = config_->nChannels;
    struct Packet* pkt[nChannels];
    void* samp[nChannels];
    for (int ch = 0; ch < nChannels; ++ch) {
        // if rx_buffer is full, exit
        if (rx_buffer_status[rx_offset + ch] == 1) {
            printf("Receive thread %d rx_buffer full, offset: %d\n", tid,
                rx_offset);
            config_->running = false;
            break;
        }
        pkt[ch] = (struct Packet*)&rx_buffer[(rx_offset + ch) * packet_length];
        samp[ch] = pkt[ch]->data;
    }

    long long frameTime;
    if (!config_->running
        || radioconfig_->radioRx(radio_id, samp, frameTime) <= 0) {
        return NULL;
    }

    int frame_id = (int)(frameTime >> 32);
    int symbol_id = (int)((frameTime >> 16) & 0xFFFF);
    int ant_id = radio_id * nChannels;
    for (int ch = 0; ch < nChannels; ++ch) {
        new (pkt[ch]) Packet(frame_id, symbol_id, 0 /* cell_id */, ant_id + ch);
        // move ptr & set status to full
        rx_buffer_status[rx_offset + ch]
            = 1; // has data, after it is read, it is set to 0

        // Push kPacketRX event into the queue.
        Event_data rx_message(
            EventType::kPacketRX, rx_tag_t(tid, rx_offset + ch)._tag);

        if (!message_queue_->enqueue(*local_ptok, rx_message)) {
            printf("socket message enqueue failed\n");
            exit(0);
        }
    }
    return pkt[0];
}

int PacketTXRX::dequeue_send(int tid)
{
    Event_data task_event;
    if (!task_queue_->try_dequeue_from_producer(*tx_ptoks_[tid], task_event))
        return -1;

    // printf("tx queue length: %d\n", task_queue_->size_approx());
    if (task_event.event_type != EventType::kPacketTX) {
        printf("Wrong event type!");
        exit(0);
    }

    int BS_ANT_NUM = config_->BS_ANT_NUM;
    int data_symbol_num_perframe = config_->data_symbol_num_perframe;

    int offset = task_event.data;
    int ant_id = offset % BS_ANT_NUM;
    int symbol_id = offset / BS_ANT_NUM % data_symbol_num_perframe;
    int frame_id = offset / (BS_ANT_NUM * data_symbol_num_perframe);

    symbol_id += config_->UE_ANT_NUM;
    frame_id += TX_FRAME_DELTA;

    void* txbuf[2];
    int nChannels = config_->nChannels;
    int ch = ant_id % nChannels;
#if DEBUG_DOWNLINK
    std::vector<std::complex<int16_t>> zeros(config_->sampsPerSymbol);
    if (ant_id != (int)config_->ref_ant)
        txbuf[ch] = zeros.data();
    else if (config_->getDownlinkPilotId(frame_id, symbol_id) >= 0)
        txbuf[ch] = config_->pilot_ci16.data();
    else
        txbuf[ch]
            = (void*)config_
                  ->dl_IQ_symbol[config_->getDlSFIndex(frame_id, symbol_id)
                      - DL_PILOT_SYMS];
#else
    int data_offset = (offset % tx_buffer_frame_num_) * config_->packet_length;
    char* cur_buffer_ptr = tx_buffer_ + data_offset;
    struct Packet* pkt = (struct Packet*)cur_buffer_ptr;
    txbuf[ch] = (void*)pkt->data;
#endif
    int last = config_->DLSymbols[0].back();
    int flags = (symbol_id != last) ? 1 // HAS_TIME
                                    : 2; // HAS_TIME & END_BURST, fixme
    long long frameTime = ((long long)frame_id << 32) | (symbol_id << 16);
    radioconfig_->radioTx(ant_id / nChannels, txbuf, flags, frameTime);

#if DEBUG_BS_SENDER
    printf("In TX thread %d: Transmitted frame %d, symbol %d, "
           "ant %d, offset: %d, msg_queue_length: %zu\n",
        tid, frame_id, symbol_id, ant_id, offset,
        message_queue_->size_approx());
#endif

    Event_data tx_message(EventType::kPacketTX, offset);
    moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];
    if (!message_queue_->enqueue(*local_ptok, tx_message)) {
        printf("socket message enqueue failed\n");
        exit(0);
    }
    return offset;
}
