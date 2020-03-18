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
    delete config_;
}

std::vector<pthread_t> PacketTXRX::startRecv(Table<char>& in_buffer,
    Table<int>& in_buffer_status, int in_buffer_frame_num,
    long long in_buffer_length, Table<double>& in_frame_start)
{
    buffer_ = &in_buffer; // for save data
    buffer_status_ = &in_buffer_status; // for save status
    frame_start_ = &in_frame_start;

    // check length
    buffer_frame_num_ = in_buffer_frame_num;
    // assert(in_buffer_length == packet_length * buffer_frame_num_); // should
    // be integer
    buffer_length_ = in_buffer_length;
    printf("create RX threads\n");
    // new thread
    // pin_to_core_with_offset(RX, core_id_, 0);

    std::vector<pthread_t> created_threads;

    if (!radioconfig_->radioStart())
        return created_threads;

    for (int i = 0; i < rx_thread_num_; i++) {
        pthread_t recv_thread_;
        // record the thread id
        auto context = new EventHandlerContext<PacketTXRX>;
        context->obj_ptr = this;
        context->id = i;
        // start socket thread
        if (pthread_create(&recv_thread_, NULL,
                pthread_fun_wrapper<PacketTXRX, &PacketTXRX::loopRecv_Argos>,
                context)
            != 0) {
            perror("socket recv thread create failed");
            exit(0);
        }
        created_threads.push_back(recv_thread_);
    }
    sleep(1);
    pthread_cond_broadcast(&cond);
    // sleep(1);
    radioconfig_->go();
    return created_threads;
}

void PacketTXRX::startTX(char* in_buffer, int* in_buffer_status,
    int in_buffer_frame_num, int in_buffer_length)
{
    // check length
    tx_buffer_frame_num_ = in_buffer_frame_num;
    // assert(in_buffer_length == packet_length * buffer_frame_num_); // should
    // be integer
    tx_buffer_length_ = in_buffer_length;
    tx_buffer_ = in_buffer; // for save data
    tx_buffer_status_ = in_buffer_status; // for save status

    printf("create TX or TXRX threads\n");
    // create new threads
    for (int i = 0; i < tx_thread_num_; i++) {
        pthread_t send_thread_;
        auto context = new EventHandlerContext<PacketTXRX>;
        context->obj_ptr = this;
        context->id = i;
        if (pthread_create(&send_thread_, NULL,
                pthread_fun_wrapper<PacketTXRX, &PacketTXRX::loopSend_Argos>,
                context)
            != 0) {
            perror("socket Transmit thread create failed");
            exit(0);
        }
    }
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
            = 1; // has data, after it is read it should be set to 0

        // Push EVENT_RX_ENB event into the queue. data records the position of
        // this packet in the rx_buffer & tid of this socket (so that task
        // thread could know which rx_buffer it should visit)
        Event_data packet_message(EventType::kPacketRX,
            generateOffset2d_setbits(tid, rx_offset + ch, 28));

        if (!message_queue_->enqueue(*local_ptok, packet_message)) {
            printf("socket message enqueue failed\n");
            exit(0);
        }
    }
    return pkt[0];
}

void* PacketTXRX::loopRecv_Argos(int tid)
{
    // printf("Recv thread: thread %d start\n", tid);
    int radio_lo = tid * config_->nRadios / rx_thread_num_;
    int radio_hi = (tid + 1) * config_->nRadios / rx_thread_num_;
    int nradio_cur_thread = radio_hi - radio_lo;
    // printf("receiver thread %d has %d radios\n", tid, nradio_cur_thread);
    // get pointer of message queue
    pin_to_core_with_offset(ThreadType::kWorkerRX, core_id_, tid);

    //// Use mutex to sychronize data receiving across threads
    pthread_mutex_lock(&mutex);
    printf("Thread %d: waiting for release\n", tid);

    pthread_cond_wait(&cond, &mutex);
    pthread_mutex_unlock(&mutex); // unlocking for all other threads

    double* frame_start = (*frame_start_)[tid];

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
    int rx_offset = 0;
    int prev_frame_id = -1;
    int nChannels = config_->nChannels;

    while (config_->running) {
        // receive data
        for (int radio_id = radio_lo; radio_id < radio_hi;
             radio_id++) { // FIXME: this must be threaded
            struct Packet* pkt = recv_enqueue_Argos(tid, radio_id, rx_offset);
            rx_offset = (rx_offset + nChannels) % buffer_frame_num_;
            int frame_id = pkt->frame_id;
#if MEASURE_TIME
            // read information from received packet
            // frame_id = *((int *)cur_ptr_buffer);
            if (frame_id > prev_frame_id) {
                *(frame_start + frame_id) = get_time();
                prev_frame_id = frame_id;
                if (frame_id % 512 == 200) {
                    _mm_prefetch(
                        (char*)(frame_start + frame_id + 512), _MM_HINT_T0);
                    // double temp = frame_start[frame_id+3];
                }
            }
#endif
#if DEBUG_RECV
            printf("PacketTXRX %d: receive frame_id %d, symbol_id %d, ant_id "
                   "%d, offset %d\n",
                tid, pkt->frame_id, pkt->symbol_id, pkt->ant_id, rx_offset);
#endif
#if DEBUG_DOWNLINK && !SEPARATE_TX_RX
            if (rx_symbol_id > 0)
                continue;
            for (size_t sym_id = 0; sym_id < txSymsPerFrame; sym_id++) {
                symbol_id = txSymbols[sym_id];
                int tx_frame_id = frame_id + TX_FRAME_DELTA;
                void* txbuf[2];
                long long frameTime
                    = ((long long)tx_frame_id << 32) | (tx_symbol_id << 16);
                int flags = 1; // HAS_TIME
                if (symbol_id == (int)txSymbols.back())
                    flags = 2; // HAS_TIME & END_BURST, fixme
                if (ant_id != (int)config_->ref_ant)
                    txbuf[0] = zeros.data();
                else if (config_->getDownlinkPilotId(frame_id, symbol_id) >= 0)
                    txbuf[0] = config_->pilot_ci16.data();
                else
                    txbuf[0] = (void*)config_->dl_IQ_symbol[sym_id];
                radioconfig_->radioTx(
                    ant_id / nChannels, txbuf, flags, frameTime);
            }
#endif
        }
    }
    return 0;
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
    int UE_NUM = config_->UE_NUM;
    int data_subframe_num_perframe = config_->data_symbol_num_perframe;
    int packet_length = config_->packet_length;
    int offset = task_event.data;
    int ant_id = offset % BS_ANT_NUM;
    int total_data_subframe_id = offset / BS_ANT_NUM;
    int frame_id = total_data_subframe_id / data_subframe_num_perframe;
    int current_data_subframe_id
        = total_data_subframe_id % data_subframe_num_perframe;
    int symbol_id = current_data_subframe_id + UE_NUM;

    int socket_subframe_offset = offset
        % (SOCKET_BUFFER_FRAME_NUM * data_subframe_num_perframe * BS_ANT_NUM);
    struct Packet* pkt
        = (struct Packet*)&tx_buffer_[socket_subframe_offset * packet_length];
    char* tx_cur_buffer_ptr = (char*)pkt->data;
    frame_id += TX_FRAME_DELTA;

    // symbol_id = task_event.data / config_->getNumAntennas();
    // for (symbol_id = 0; symbol_id < txSymsPerFrame; symbol_id++)
    //{
    UNUSED void* txbuf[2];
    long long frameTime = ((long long)frame_id << 32) | (symbol_id << 16);
#if SEPARATE_TX_RX
    int last = config_->isUE ? config_->ULSymbols[0].back()
                             : config_->DLSymbols[0].back();
    int flags = (symbol_id != last) ? 1 // HAS_TIME
                                    : 2; // HAS_TIME & END_BURST, fixme
#endif
    int nChannels = config_->nChannels;
    int ch = ant_id % nChannels;
#if DEBUG_DOWNLINK
    std::vector<std::complex<int16_t>> zeros(config_->sampsPerSymbol);
    if (ant_id != (int)config_->ref_ant)
        txbuf[ch] = zeros.data();
    else if (config_->getDownlinkPilotId(frame_id, symbol_id) >= 0)
        txbuf[ch] = config_->pilot_ci16.data();
    else
        txbuf[ch] = (void*)config_->dl_IQ_symbol[current_data_subframe_id];
#else
    txbuf[ch] = tx_cur_buffer_ptr + ch * packet_length;
#endif
        // buffer_status[offset+ch] = 0;
#if DEBUG_BS_SENDER
    printf("In TX thread %d: Transmitted frame %d, subframe %d, ant %d, "
           "offset: %d, msg_queue_length: %d\n",
        tid, frame_id, symbol_id, ant_id, offset,
        message_queue_->size_approx());
#endif

    // clock_gettime(CLOCK_MONOTONIC, &tv);
#if SEPARATE_TX_RX
    radioconfig_->radioTx(ant_id / nChannels, txbuf, flags, frameTime);
#endif
    // clock_gettime(CLOCK_MONOTONIC, &tv2);

    Event_data tx_message(EventType::kPacketTX, offset);

    moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];
    if (!message_queue_->enqueue(*local_ptok, tx_message)) {
        printf("socket message enqueue failed\n");
        exit(0);
    }
    return offset;
}

void* PacketTXRX::loopSend_Argos(int tid)
{
    pin_to_core_with_offset(ThreadType::kWorkerTX, tx_core_id_, tid);

    while (config_->running) {
        dequeue_send_Argos(tid);
    }
    return 0;
}
