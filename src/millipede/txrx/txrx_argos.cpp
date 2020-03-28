/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */

#include "txrx.hpp"

PacketTXRX::PacketTXRX(
    Config* cfg, int RX_THREAD_NUM, int TX_THREAD_NUM, int in_core_offset)
{
    socket_ = new int[RX_THREAD_NUM];
    config_ = cfg;
    rx_thread_num_ = RX_THREAD_NUM;
    tx_thread_num_ = TX_THREAD_NUM;

    core_id_ = in_core_offset;
    tx_core_id_ = in_core_offset + RX_THREAD_NUM;

    /* initialize random seed: */
    srand(time(NULL));

    radioconfig_ = new RadioConfig(config_);
}

PacketTXRX::PacketTXRX(Config* cfg, int RX_THREAD_NUM, int TX_THREAD_NUM,
    int in_core_offset,
    moodycamel::ConcurrentQueue<Event_data>* in_queue_message,
    moodycamel::ConcurrentQueue<Event_data>* in_queue_task,
    moodycamel::ProducerToken** in_rx_ptoks,
    moodycamel::ProducerToken** in_tx_ptoks)
    : PacketTXRX(cfg, RX_THREAD_NUM, TX_THREAD_NUM, in_core_offset)
{
    message_queue_ = in_queue_message;
    task_queue_ = in_queue_task;
    rx_ptoks_ = in_rx_ptoks;
    tx_ptoks_ = in_tx_ptoks;
}

PacketTXRX::~PacketTXRX()
{
    delete[] socket_;
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
    for (int i = 0; i < tx_thread_num_; i++) {
        pthread_t txrx_thread;
        auto context = new EventHandlerContext<PacketTXRX>;
        context->obj_ptr = this;
        context->id = i;
        if (pthread_create(&txrx_thread, NULL,
                pthread_fun_wrapper<PacketTXRX, &PacketTXRX::loopTXRX_Argos>,
                context)
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

void* PacketTXRX::loopTXRX_Argos(int tid)
{
    pin_to_core_with_offset(ThreadType::kWorkerTXRX, core_id_, tid);
    // printf("Recv thread: thread %d start\n", tid);
    int radio_lo = tid * config_->nRadios / rx_thread_num_;
    int radio_hi = (tid + 1) * config_->nRadios / rx_thread_num_;
    int nradio_cur_thread = radio_hi - radio_lo;
    // printf("receiver thread %d has %d radios\n", tid, nradio_cur_thread);
    // get pointer of message queue

    //// Use mutex to sychronize data receiving across threads
    pthread_mutex_lock(&mutex);
    printf("Thread %d: waiting for release\n", tid);

    pthread_cond_wait(&cond, &mutex);
    pthread_mutex_unlock(&mutex); // unlocking for all other threads

    double* rx_frame_start = (*frame_start_)[tid];
    int rx_offset = 0;

    // downlink socket buffer
    // char *tx_buffer_ptr = tx_buffer_;
    // char *tx_cur_buffer_ptr;
#if DEBUG_DOWNLINK
    size_t txSymsPerFrame = config_->dl_data_symbol_num_perframe;
    std::vector<size_t> txSymbols = config_->DLSymbols[0];
    std::vector<std::complex<int16_t>> zeros(config_->sampsPerSymbol);
#endif

    printf("receiver thread %d has %d radios\n", tid, nradio_cur_thread);

    // to handle second channel at each radio
    // this is assuming buffer_frame_num_ is at least 2
    int prev_frame_id = -1;
    int nChannels = config_->nChannels;

    while (config_->running) {
        // receive data
        for (int radio_id = radio_lo; radio_id < radio_hi;
             radio_id++) { // FIXME: this must be threaded
            while (-1 != dequeue_send_Argos(tid))
                ;
            struct Packet* pkt = recv_enqueue_Argos(tid, radio_id, rx_offset);
            rx_offset = (rx_offset + nChannels) % buffer_frame_num_;
            int frame_id = pkt->frame_id;
#if MEASURE_TIME
            // read information from received packet
            // frame_id = *((int *)cur_ptr_buffer);
            if (frame_id > prev_frame_id) {
                *(rx_frame_start + frame_id) = get_time();
                prev_frame_id = frame_id;
                if (frame_id % 512 == 200) {
                    _mm_prefetch(
                        (char*)(rx_frame_start + frame_id + 512), _MM_HINT_T0);
                }
            }
#endif
#if DEBUG_RECV
            printf("PacketTXRX %d: receive frame_id %d, symbol_id %d, ant_id "
                   "%d, offset %d\n",
                tid, pkt->frame_id, pkt->symbol_id, pkt->ant_id, rx_offset);
#endif
        }
    }
    return 0;
}

struct Packet* PacketTXRX::recv_enqueue_Argos(
    int tid, int radio_id, int rx_offset)
{
    moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];
    char* rx_buffer = (*buffer_)[tid];
    int* rx_buffer_status = (*buffer_status_)[tid];
    int packet_length = config_->packet_length;
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

    // TODO: this is probably a really bad implementation, and needs to be
    // revamped
    long long frameTime;
    while (config_->running
        && radioconfig_->radioRx(radio_id, samp, frameTime) <= 0) {
        // Busy loop
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
        // data records the position of this packet in the rx_buffer & tid of
        // this socket (so that task thread could know which rx_buffer it should
        // visit)
        Event_data rx_message(EventType::kPacketRX,
            generateOffset2d_setbits(tid, rx_offset + ch, 28));

        if (!message_queue_->enqueue(*local_ptok, rx_message)) {
            printf("socket message enqueue failed\n");
            exit(0);
        }
    }
    return pkt[0];
}

int PacketTXRX::dequeue_send_Argos(int tid)
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
    int data_subframe_num_perframe = config_->data_symbol_num_perframe;
    int packet_length = config_->packet_length;
    int offset = task_event.data;
    int ant_id = offset % BS_ANT_NUM;
    int symbol_id = offset / BS_ANT_NUM % data_subframe_num_perframe;
    symbol_id += config_->UE_NUM;
    int frame_id = offset / (BS_ANT_NUM * data_subframe_num_perframe);

#if DEBUG_BS_SENDER
    printf("In TX thread %d: Transmitted frame %d, subframe %d, "
           "ant %d, offset: %d, msg_queue_length: %zu\n",
        tid, frame_id, symbol_id, ant_id, offset,
        message_queue_->size_approx());
#endif

    int socket_subframe_offset = offset
        % (SOCKET_BUFFER_FRAME_NUM * data_subframe_num_perframe * BS_ANT_NUM);
    char* cur_buffer_ptr = tx_buffer_ + socket_subframe_offset * packet_length;
    struct Packet* pkt = (struct Packet*)cur_buffer_ptr;
    char* tx_cur_buffer_ptr = (char*)pkt->data;
    frame_id += TX_FRAME_DELTA;

    void* txbuf[2];
    long long frameTime = ((long long)frame_id << 32) | (symbol_id << 16);
    int nChannels = config_->nChannels;
    int ch = ant_id % nChannels;
#if DEBUG_DOWNLINK
    std::vector<std::complex<int16_t>> zeros(config_->sampsPerSymbol);
    if (ant_id != (int)config_->ref_ant)
        txbuf[ch] = zeros.data();
    else if (config_->getDownlinkPilotId(frame_id, symbol_id) >= 0)
        txbuf[ch] = config_->pilot_ci16.data();
    else
        txbuf[ch] = (void*)config_->dl_IQ_symbol[offset / BS_ANT_NUM
            % data_subframe_num_perframe];
#else
    txbuf[ch] = tx_cur_buffer_ptr + ch * packet_length;
#endif
    int last = config_->isUE ? config_->ULSymbols[0].back()
                             : config_->DLSymbols[0].back();
    int flags = (symbol_id != last) ? 1 // HAS_TIME
                                    : 2; // HAS_TIME & END_BURST, fixme
    radioconfig_->radioTx(ant_id / nChannels, txbuf, flags, frameTime);

    Event_data tx_message(EventType::kPacketTX, offset);
    moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];
    if (!message_queue_->enqueue(*local_ptok, tx_message)) {
        printf("socket message enqueue failed\n");
        exit(0);
    }
    return offset;
}
