
#include "config.hpp"
#include "ru.hpp"
//#include "radio_lib.hpp"

RU::RU(int n_rx_thread, int n_tx_thread, Config* cfg)
{
    config_ = cfg;
    radioconfig_ = new ClientRadioConfig(config_);

    thread_num_ = n_rx_thread;
    tx_thread_num_ = n_tx_thread;
    /* initialize random seed: */
    srand(time(NULL));
}

RU::RU(int n_rx_thread, int n_tx_thread, Config* config,
    moodycamel::ConcurrentQueue<Event_data>* in_message_queue,
    moodycamel::ConcurrentQueue<Event_data>* in_task_queue)
    : RU(n_rx_thread, n_tx_thread, config)
{
    message_queue_ = in_message_queue;
    task_queue_ = in_task_queue;
    task_ptok.resize(thread_num_);
    for (int i = 0; i < thread_num_; i++)
        task_ptok[i].reset(new moodycamel::ProducerToken(*task_queue_));
}

RU::~RU()
{
    radioconfig_->radioStop();
    delete radioconfig_;
    delete config_;
}

void RU::startRadios()
{
#ifndef SIM
    radioconfig_->radioStart();
#endif
}

std::vector<pthread_t> RU::startTXRX(Table<char>& in_buffer,
    Table<int>& in_buffer_status, int in_buffer_frame_num, int in_buffer_length,
    int in_core_id)
{
    // check length
    buffer_frame_num_ = in_buffer_frame_num;
    assert(in_buffer_length
        == config_->packet_length * buffer_frame_num_); // should be integer
    buffer_length_ = in_buffer_length;
    buffer_ = &in_buffer; // for save data
    buffer_status_ = &in_buffer_status; // for save status

    core_id_ = in_core_id;

    std::vector<pthread_t> created_threads;
    for (int i = 0; i < thread_num_; i++) {
        pthread_t proc_thread_;
        // record the thread id
        auto* context = new RUContext;
        context->ptr = this;
        context->tid = i;
        // start socket thread
        if (pthread_create(&proc_thread_, NULL, taskThread_launch, context)
            != 0) {
            perror("socket thread create failed");
            exit(0);
        } else {
            printf("RU thread: thread %d created\n", i);
        }
        created_threads.push_back(proc_thread_);
    }

    // give time for all threads to lock
    sleep(1);
    pthread_cond_broadcast(&cond);

    return created_threads;
}

std::vector<pthread_t> RU::startTX(char* in_buffer, char* in_pilot_buffer,
    int* in_buffer_status, int in_buffer_frame_num, int in_buffer_length,
    int in_core_id)
{
    // check length
    tx_buffer_frame_num_ = in_buffer_frame_num;
    tx_buffer_length_ = in_buffer_length;
    tx_buffer_ = in_buffer; // for save data
    pilot_buffer_ = in_pilot_buffer;
    tx_buffer_status_ = in_buffer_status; // for save status

    // create new threads
    std::vector<pthread_t> created_threads;
    tx_core_id_ = in_core_id;
#if SEPARATE_TX_RX_UE
    printf("start Transmit thread\n");
    for (int i = 0; i < tx_thread_num_; i++) {
        auto* context = new RUContext;
        context->ptr = this;
        context->tid = i;

        pthread_t send_thread;
        if (pthread_create(&send_thread, NULL, sendThread_launch, context)
            != 0) {
            perror("socket Transmit thread create failed");
            exit(0);
        }
        created_threads.push_back(send_thread);
    }

#endif
    return created_threads;
}

/*****  transmit threads   *****/

void* RU::sendThread_launch(void* in_context)
{
    RUContext* context = (RUContext*)in_context;
    RU* me = context->ptr;
    int tid = context->tid;
    delete context;
    me->sendThread(tid);
    return 0;
}

void RU::sendThread(int tid)
{
    printf("packet sender thread %d start\n", tid);

#ifdef ENABLE_CPU_ATTACH
    if (pin_to_core(core_id_ + tid) != 0) {
        printf("TX thread: stitch thread %d to core %d failed\n", tid,
            core_id_ + tid);
        exit(0);
    } else {
        printf("TX thread: stitch thread %d to core %d succeeded\n", tid,
            core_id_ + tid);
    }
#endif

    int packet_length = config_->packet_length;
    ClientRadioConfig* radio = radioconfig_;
    packet_length -= offsetof(Packet, data);

    int ret;
    size_t ant_id, symbol_id;
    int offset = 0;
    int frame_id = 0;
    struct timespec tv, tv2;
#if TX_TIME_MEASURE
    double time_avg = 0;
    int time_count = 0;
#endif

    std::vector<size_t>& txSymbols = config_->ULSymbols[0];
    // use token to speed up
    moodycamel::ProducerToken local_ptok(*message_queue_);
    while (config_->running) {

        Event_data task_event;
        // ret = task_queue_.try_dequeue(task_event);
        ret = task_queue_->try_dequeue_from_producer(
            *task_ptok[tid], task_event);
        if (!ret)
            continue;

        // printf("tx queue length: %d\n", task_queue_.size_approx());
        if (task_event.event_type != EventType::kPacketTX) {
            printf("Wrong event type!");
            exit(0);
        }

        ant_id = task_event.data; //% config_->getNumAntennas();
        // frame_id = task_event.more_data;

        // symbol_id = task_event.data / config_->getNumAntennas();
        for (symbol_id = 0; symbol_id < txSymbols.size(); symbol_id++) {
            int tx_frame_id = frame_id;
            size_t tx_symbol_id = txSymbols[symbol_id];
            offset = generateOffset3d(TASK_BUFFER_FRAME_NUM, txSymbols.size(),
                config_->getNumAntennas(), frame_id, symbol_id, ant_id);
            void* txbuf[2];
            long long frameTime
                = ((long long)tx_frame_id << 32) | (tx_symbol_id << 16);
            int flags = 1; // HAS_TIME
            if (tx_symbol_id == txSymbols.back())
                flags = 2; // HAS_TIME & END_BURST, fixme
            for (size_t ch = 0; ch < config_->nChannels; ++ch) {
                txbuf[ch] = tx_buffer_
                    + (offset + ch) * packet_length; //   pilot_buffer_; //
                tx_buffer_status_[offset + ch] = 0;
            }
            clock_gettime(CLOCK_MONOTONIC, &tv);
            radio->radioTx(
                ant_id / config_->nChannels, txbuf, flags, frameTime);
            clock_gettime(CLOCK_MONOTONIC, &tv2);
#if TX_TIME_MEASURE
            double diff = (tv2.tv_sec * 1e9 + tv2.tv_nsec - tv.tv_sec * 1e9
                - tv.tv_nsec);
            time_avg += diff;
            time_count++;
            if (time_count
                == config_->getNumAntennas()
                    * config_->dl_data_symbol_num_perframe) {
                printf("In TX thread %d: Transmitted 100 frames with average "
                       "tx time %f\n",
                    tid, time_avg / time_count / 1e3);
                time_count = 0;
                time_avg = 0;
            }
#endif

#if DEBUG_SEND
            printf(
                "TX thread %d: finished TX for frame %d, symbol %d, ant %d\n",
                tid, frame_id, symbol_id, ant_id);
#endif
        }

        Event_data packet_message(EventType::kPacketTX, offset);
        // packet_message.more_data = frame_id;

        if (config_->running
            && !message_queue_->enqueue(local_ptok, packet_message)) {
            printf("socket message enqueue failed\n");
            exit(0);
        }
    }
}

/*****  Receive threads   *****/

void* RU::taskThread_launch(void* in_context)
{
    RUContext* context = (RUContext*)in_context;
    RU* me = context->ptr;
    int tid = context->tid;
    delete context;
    me->taskThread(tid);
    return 0;
}

void RU::taskThread(int tid)
{
    // if ENABLE_CPU_ATTACH is enabled, attach threads to specific cores
#ifdef ENABLE_CPU_ATTACH
    // printf("Recv thread: pinning thread %d to core %d\n", tid, core_id_ +
    // tid);
    if (pin_to_core(core_id_ + tid) != 0) {
        printf("Recv thread: pinning thread %d to core %d failed\n", tid,
            core_id_ + tid);
        exit(0);
    } else {
        printf("Recv thread: pinning thread %d to core %d succeed\n", tid,
            core_id_ + tid);
    }
#endif

    // Use mutex to sychronize data receiving across threads
    pthread_mutex_lock(&mutex);
    printf("Thread %d: waiting for release\n", tid);

    pthread_cond_wait(&cond, &mutex);
    pthread_mutex_unlock(&mutex); // unlocking for all other threads

    // usleep(10000-tid*2000);
    // use token to speed up
    moodycamel::ProducerToken local_ptok(*message_queue_);
    // moodycamel::ProducerToken local_ctok(task_queue_);
    // moodycamel::ProducerToken *local_ctok = (task_ptok[tid]);

    const std::vector<size_t>& txSymbols = config_->ULSymbols[0];
    int tx_packet_length = config_->packet_length - offsetof(Packet, data);
    int frame_samp_size
        = tx_packet_length * config_->getNumAntennas() * txSymbols.size();

    char* buffer = (*buffer_)[tid];
    int* buffer_status = (*buffer_status_)[tid];

    int num_radios = config_->nRadios;
    int radio_start = tid * num_radios / thread_num_;
    int radio_end = (tid + 1) * num_radios / thread_num_;
    printf("receiver thread %d has radios %d to %d (%d)\n", tid, radio_start,
        radio_end - 1, radio_end - radio_start);
    ClientRadioConfig* radio = radioconfig_;
    int packet_num = 0;
    long long frameTime;

    std::vector<int> all_trigs(radio_end - radio_start, 0);
    struct timespec tv, tv2;
    clock_gettime(CLOCK_MONOTONIC, &tv);

    int maxQueueLength = 0;
    int cursor = 0;
    while (config_->running) {
        clock_gettime(CLOCK_MONOTONIC, &tv2);
        double diff
            = ((tv2.tv_sec - tv.tv_sec) * 1e9 + (tv2.tv_nsec - tv.tv_nsec))
            / 1e9;
        if (diff > 2) {
            for (int it = radio_start; it < radio_end; it++) {
                int total_trigs = radio->triggers(it);
                std::cout << "radio: " << it << ", new triggers: "
                          << total_trigs - all_trigs[it - radio_start]
                          << ", total: " << total_trigs << std::endl;
                all_trigs[it - radio_start] = total_trigs;
            }
            tv = tv2;
        }
        // if buffer is full, exit
        if (buffer_status[cursor] == 1) {
            printf("RX thread %d at cursor %d buffer full\n", tid, cursor);
            // exit(0);
            for (int i = 0; i < buffer_frame_num_; i++)
                printf("%d ", buffer_status[cursor]);
            printf("\n");
            config_->running = false;
            break;
        }
        // receive data
        for (int it = radio_start; it < radio_end; it++) {
            // this is probably a really bad implementation, and needs to be
            // revamped
            struct Packet* pkt[config_->nChannels];
            void* samp[config_->nChannels];
            for (size_t ch = 0; ch < config_->nChannels; ++ch) {
                pkt[ch] = (struct Packet*)&buffer[(cursor + ch)
                    * config_->packet_length];
                samp[ch] = pkt[ch]->data;
            }
            while (config_->running
                && radio->radioRx(it, samp, frameTime)
                    < (int)config_->sampsPerSymbol)
                ;
            int frame_id = (int)(frameTime >> 32);
            int symbol_id = (int)((frameTime >> 16) & 0xFFFF);
            int ant_id = it * config_->nChannels;
#if DEBUG_RECV
            printf("receive thread %d: frame_id %d, symbol_id %d, ant_id %d "
                   "frametime %llx\n",
                tid, frame_id, symbol_id, ant_id, frameTime);
#endif
            for (size_t ch = 0; ch < config_->nChannels; ++ch) {
                new (pkt[ch])
                    Packet(frame_id, symbol_id, 0 /* cell_id */, ant_id + ch);
                // buffer_status[cursor + ch]
                //    = 1; // has data, after it is read it should be set to 0

                // Push EVENT_RX_ENB event into the queue. data records the
                // position of this packet in the buffer & tid of this socket
                // (so that task thread could know which buffer
                Event_data packet_message(
                    EventType::kPacketRX, cursor + tid * buffer_frame_num_);

                if (!message_queue_->enqueue(local_ptok, packet_message)) {
                    printf("socket message enqueue failed\n");
                    exit(0);
                }
                cursor++;
                cursor %= buffer_frame_num_;
            }

#if !SEPARATE_TX_RX_UE
            if (txSymbols.size() > 0
                && config_->getDlSFIndex(frame_id, symbol_id) == 0) {
                for (size_t tx_symbol_id = 0; tx_symbol_id < txSymbols.size();
                     tx_symbol_id++) {
                    int tx_frame_id = frame_id + TX_FRAME_DELTA;
                    size_t tx_symbol = txSymbols[tx_symbol_id];
                    int tx_ant_offset
                        = tx_symbol_id * config_->getNumAntennas() + ant_id;
                    void* txbuf[2];
#if DEBUG_UPLINK
                    for (size_t ch = 0; ch < config_->nChannels; ++ch)
                        txbuf[ch]
                            = (void*)config_->ul_IQ_symbol[tx_ant_offset + ch];
#else
                    int tx_frame_offset = tx_frame_id % TASK_BUFFER_FRAME_NUM;
                    int tx_offset = tx_frame_offset * frame_samp_size
                        + tx_packet_length * (tx_ant_offset);
                    char* cur_buffer_ = (tx_buffer_ + tx_offset
                        + tx_packet_length);
                    struct Packet* pkt = (struct Packet*)cur_buffer_;
                    char* tx_cur_buffer_ = (char*)pkt->data;
                    for (size_t ch = 0; ch < config_->nChannels; ++ch)
                        txbuf[ch] = (void*)tx_cur_buffer_;
#endif
                    long long frameTime = ((long long)tx_frame_id << 32)
                        | ((long long)tx_symbol << 16);
                    int flags = (tx_symbol == txSymbols.back())
                        ? 2
                        : // HAS_TIME & END_BURST
                        1; // HAS_TIME
                    radio->radioTx(it, txbuf, flags, frameTime);
                }
            }
#endif
            // printf("enqueue offset %d\n", offset);
            int cur_queue_len = message_queue_->size_approx();
            maxQueueLength = maxQueueLength > cur_queue_len ? maxQueueLength
                                                            : cur_queue_len;

            packet_num++;
        }
    }
}
