
#include "config.hpp"
#include "ru.hpp"
//#include "radio_lib.hpp"

RU::RU(Config* cfg, int n_threads, int in_core_id)
{
    config_ = cfg;
    radioconfig_ = new ClientRadioConfig(config_);

    thread_num_ = n_threads;
    core_id_ = in_core_id;

    /* initialize random seed: */
    srand(time(NULL));
}

RU::RU(Config* config, int n_threads, int in_core_id,
    moodycamel::ConcurrentQueue<Event_data>* in_message_queue,
    moodycamel::ConcurrentQueue<Event_data>* in_task_queue,
    moodycamel::ProducerToken** in_rx_ptoks,
    moodycamel::ProducerToken** in_tx_ptoks)
    : RU(config, n_threads, in_core_id)
{
    message_queue_ = in_message_queue;
    task_queue_ = in_task_queue;
    rx_ptoks_ = in_rx_ptoks;
    tx_ptoks_ = in_tx_ptoks;
}

RU::~RU()
{
    radioconfig_->radioStop();
    delete radioconfig_;
    delete config_;
}

bool RU::startTXRX(Table<char>& in_buffer, Table<int>& in_buffer_status,
    int in_buffer_frame_num, int in_buffer_length, char* in_tx_buffer,
    int* in_tx_buffer_status, int in_tx_buffer_frame_num,
    int in_tx_buffer_length)
{
    buffer_frame_num_ = in_buffer_frame_num;
    assert(in_buffer_length
        == config_->packet_length * buffer_frame_num_); // should be integer
    buffer_length_ = in_buffer_length;
    buffer_ = &in_buffer; // for save data
    buffer_status_ = &in_buffer_status; // for save status

    tx_buffer_frame_num_ = in_tx_buffer_frame_num;
    tx_buffer_length_ = in_tx_buffer_length;
    tx_buffer_ = in_tx_buffer;
    tx_buffer_status_ = in_tx_buffer_status;

    if (!radioconfig_->radioStart())
        return false;

    for (int i = 0; i < thread_num_; i++) {
        pthread_t txrx_thread;
        // record the thread id
        auto context = new EventHandlerContext<RU>;
        context->obj_ptr = this;
        context->id = i;
        // start socket thread
        if (config_->hw_framer) {
            if (pthread_create(&txrx_thread, NULL,
                    pthread_fun_wrapper<RU, &RU::loopTXRX>, context)
                != 0) {
                perror("socket thread create failed");
                exit(0);
            }
        } else {
            if (pthread_create(&txrx_thread, NULL,
                    pthread_fun_wrapper<RU, &RU::loopSYNC_TXRX>, context)
                != 0) {
                perror("socket thread create failed");
                exit(0);
            }
        }
    }

    // give time for all threads to lock
    sleep(1);
    pthread_cond_broadcast(&cond);

    return true;
}

int RU::dequeue_send(int tid)
{
    auto& c = config_;
    auto& radio = radioconfig_;
    int packet_length = c->packet_length;

    Event_data event;
    if (!task_queue_->try_dequeue_from_producer(*tx_ptoks_[tid], event))
        return -1;

    assert(event.event_type == EventType::kPacketTX);

    size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
    size_t ue_id = gen_tag_t(event.tags[0]).ant_id;

    for (size_t symbol_id = 0; symbol_id < c->ul_data_symbol_num_perframe;
         symbol_id++) {
        size_t tx_frame_id = frame_id + TX_FRAME_DELTA;
        size_t tx_symbol_id = c->ULSymbols[0][symbol_id];
        size_t ant_id = ue_id * c->nChannels;
        size_t offset = (c->get_total_data_symbol_idx_ul(frame_id, symbol_id)
                            * c->UE_ANT_NUM)
            + ant_id;

        void* txbuf[2];
        for (size_t ch = 0; ch < c->nChannels; ++ch) {
            struct Packet* pkt
                = (struct Packet*)(tx_buffer_ + (offset + ch) * packet_length);
            txbuf[ch] = (void*)pkt->data;
            tx_buffer_status_[offset + ch] = 0;
        }
        long long frameTime
            = ((long long)tx_frame_id << 32) | (tx_symbol_id << 16);
        int flags = 1; // HAS_TIME
        if (tx_symbol_id == c->ULSymbols[0].back())
            flags = 2; // HAS_TIME & END_BURST, fixme
        radio->radioTx(ue_id, txbuf, c->sampsPerSymbol, flags, frameTime);
    }

    rt_assert(message_queue_->enqueue(*rx_ptoks_[tid],
                  Event_data(EventType::kPacketTX, event.tags[0])),
        "Socket message enqueue failed\n");
    return event.tags[0];
}

void* RU::loopTXRX(int tid)
{
    // if ENABLE_CPU_ATTACH is enabled, attach threads to specific cores
    pin_to_core_with_offset(ThreadType::kWorkerTXRX, core_id_, tid);
    auto& c = config_;
    int num_radios = c->nRadios;
    int radio_lo = tid * num_radios / thread_num_;
    int radio_hi = (tid + 1) * num_radios / thread_num_;
    printf("receiver thread %d has radios %d to %d (%d)\n", tid, radio_lo,
        radio_hi - 1, radio_hi - radio_lo);

    // Use mutex to sychronize data receiving across threads
    pthread_mutex_lock(&mutex);
    printf("Thread %d: waiting for release\n", tid);

    pthread_cond_wait(&cond, &mutex);
    pthread_mutex_unlock(&mutex); // unlocking for all other threads

    // usleep(10000-tid*2000);
    // use token to speed up
    moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];

    char* buffer = (*buffer_)[tid];
    int* buffer_status = (*buffer_status_)[tid];

    ClientRadioConfig* radio = radioconfig_;
    long long frameTime;

    std::vector<int> all_trigs(radio_hi - radio_lo, 0);
    struct timespec tv, tv2;
    clock_gettime(CLOCK_MONOTONIC, &tv);

    int cursor = 0;
    int radio_id = radio_lo;
    while (c->running) {
        clock_gettime(CLOCK_MONOTONIC, &tv2);
        double diff
            = ((tv2.tv_sec - tv.tv_sec) * 1e9 + (tv2.tv_nsec - tv.tv_nsec))
            / 1e9;
        if (diff > 2) {
            for (int it = radio_lo; it < radio_hi; it++) {
                int total_trigs = radio->triggers(it);
                std::cout << "radio: " << it << ", new triggers: "
                          << total_trigs - all_trigs[it - radio_lo]
                          << ", total: " << total_trigs << std::endl;
                all_trigs[it - radio_lo] = total_trigs;
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
            c->running = false;
            break;
        }
        // transmit data
        if (-1 != dequeue_send(tid))
            continue;
        struct Packet* pkt[c->nChannels];
        void* samp[c->nChannels];
        for (size_t ch = 0; ch < c->nChannels; ++ch) {
            pkt[ch] = (struct Packet*)&buffer[(cursor + ch)
                * config_->packet_length];
            samp[ch] = pkt[ch]->data;
        }
        while (c->running
            && radio->radioRx(radio_id, samp, c->sampsPerSymbol, frameTime)
                < (int)c->sampsPerSymbol)
            ;
        int frame_id = (int)(frameTime >> 32);
        int symbol_id = (int)((frameTime >> 16) & 0xFFFF);
        int ant_id = radio_id * c->nChannels;
#if DEBUG_RECV
        printf("receive thread %d: frame_id %d, symbol_id %d, radio_id %d "
               "frametime %llx\n",
            tid, frame_id, symbol_id, radio_id, frameTime);
#endif
        for (size_t ch = 0; ch < c->nChannels; ++ch) {
            new (pkt[ch])
                Packet(frame_id, symbol_id, 0 /* cell_id */, ant_id + ch);

            Event_data rx_message(
                EventType::kPacketRX, rx_tag_t(tid, cursor + ch)._tag);

            if (!message_queue_->enqueue(*local_ptok, rx_message)) {
                printf("socket message enqueue failed\n");
                exit(0);
            }
            cursor++;
            cursor %= buffer_frame_num_;
        }

        if (++radio_id == radio_hi)
            radio_id = radio_lo;
    }
    return 0;
}

void* RU::loopSYNC_TXRX(int tid)
{
    // FIXME: This only works when there is 1 radio per thread.
    // if ENABLE_CPU_ATTACH is enabled, attach threads to specific cores
    pin_to_core_with_offset(ThreadType::kWorkerTXRX, core_id_, tid);
    auto& c = config_;
    int num_radios = c->nRadios;
    int radio_lo = tid * num_radios / thread_num_;
    int radio_hi = (tid + 1) * num_radios / thread_num_;
    printf("receiver thread %d has radios %d to %d (%d)\n", tid, radio_lo,
        radio_hi - 1, radio_hi - radio_lo);

    // Use mutex to sychronize data receiving across threads
    pthread_mutex_lock(&mutex);
    printf("Thread %d: waiting for release\n", tid);

    pthread_cond_wait(&cond, &mutex);
    pthread_mutex_unlock(&mutex); // unlocking for all other threads

    // usleep(10000-tid*2000);
    // use token to speed up
    moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];

    char* buffer = (*buffer_)[tid];
    int* buffer_status = (*buffer_status_)[tid];

    ClientRadioConfig* radio = radioconfig_;

    int num_samps = c->sampsPerSymbol;
    int frm_num_samps = num_samps * c->symbol_num_perframe;
    std::vector<std::complex<int16_t>> frmbuff0(frm_num_samps, 0);
    std::vector<std::complex<int16_t>> frmbuff1(frm_num_samps, 0);
    std::vector<void*> frmrxbuff(2);
    frmrxbuff[0] = frmbuff0.data();

    long long rxTime(0);
    long long txTime(0);
    int radio_id = radio_lo;
    int sync_index(-1);
    int rx_offset = 0;
    while (c->running && sync_index < 0) {
        int r
            = radio->radioRx(radio_id, frmrxbuff.data(), frm_num_samps, rxTime);

        if (r != frm_num_samps) {
            std::cerr << "BAD SYNC Receive(" << r << "/" << frm_num_samps
                      << ") at Time " << rxTime << std::endl;
            continue;
        }

        // convert data to complex float for sync detection
        std::vector<std::complex<float>> syncbuff;
        for (int i = 0; i < frm_num_samps; i++)
            syncbuff.push_back(std::complex<float>(
                frmbuff0[i].real() / 32768.0, frmbuff0[i].imag() / 32768.0));
        sync_index = CommsLib::find_beacon_avx(syncbuff, c->gold_cf32);
        if (sync_index < 0)
            continue;
        std::cout << "Beacon detected at Time " << rxTime
                  << ", sync_index: " << sync_index << std::endl;
        rx_offset = sync_index - config_->beacon_len - c->prefix;
    }

    // Read rx_offset to align with the begining of a frame
    radio->radioRx(radio_id, frmrxbuff.data(), rx_offset, rxTime);

    long long time0(0);
    int frame_id = 0;

    int cursor = 0;
    while (c->running) {

        // recv corresponding to sf = 0 (Beacon)
        radio->radioRx(radio_id, frmrxbuff.data(), num_samps, rxTime);
        if (frame_id == 0) {
            time0 = rxTime;
        }
        // transmit data
        Event_data event;
        while (task_queue_->try_dequeue_from_producer(*tx_ptoks_[tid], event)) {

            assert(event.event_type == EventType::kPacketTX);

            size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
            size_t ue_id = gen_tag_t(event.tags[0]).ant_id;

            for (size_t symbol_id = 0;
                 symbol_id < c->ul_data_symbol_num_perframe; symbol_id++) {
                size_t tx_frame_id = frame_id + TX_FRAME_DELTA;
                size_t tx_symbol_id = c->ULSymbols[0][symbol_id];

                size_t ant_id = ue_id * c->nChannels;
                size_t offset
                    = (c->get_total_data_symbol_idx_ul(frame_id, symbol_id)
                          * c->UE_ANT_NUM)
                    + ant_id;

                void* txbuf[2];
                for (size_t ch = 0; ch < c->nChannels; ++ch) {
                    struct Packet* pkt = (struct Packet*)(tx_buffer_
                        + (offset + ch) * c->packet_length);
                    txbuf[ch] = (void*)pkt->data;
                    tx_buffer_status_[offset + ch] = 0;
                }
                txTime = time0 + tx_frame_id * frm_num_samps
                    + tx_symbol_id * num_samps - 100;
                int flags = 1; // HAS_TIME
                if (tx_symbol_id == c->ULSymbols[0].back())
                    flags = 2; // HAS_TIME & END_BURST, fixme
                radio->radioTx(ue_id, txbuf, num_samps, flags, txTime);
            }
            rt_assert(message_queue_->enqueue(*rx_ptoks_[tid],
                          Event_data(EventType::kPacketTX, event.tags[0])),
                "Socket message enqueue failed\n");
        }

        // receive the remaining of the frame
        for (size_t sf = 1; sf < c->symbol_num_perframe; sf++) {
            if (c->frames[0].at(sf) != 'R')
                radio->radioRx(radio_id, frmrxbuff.data(), num_samps, rxTime);
            else {
                // if buffer is full, exit
                if (buffer_status[cursor] == 1) {
                    printf(
                        "RX thread %d at cursor %d buffer full\n", tid, cursor);
                    // exit(0);
                    for (int i = 0; i < buffer_frame_num_; i++)
                        printf("%d ", buffer_status[cursor]);
                    printf("\n");
                    c->running = false;
                    break;
                }

                struct Packet* pkt[c->nChannels];
                void* samp[c->nChannels];
                for (size_t ch = 0; ch < c->nChannels; ++ch) {
                    pkt[ch] = (struct Packet*)&buffer[(cursor + ch)
                        * config_->packet_length];
                    samp[ch] = pkt[ch]->data;
                }
                int r = radio->radioRx(radio_id, samp, num_samps, rxTime);
                if (r < num_samps)
                    std::cerr << "BAD Receive(" << r << "/" << num_samps
                              << ") at Time " << rxTime << std::endl;
                int ant_id = radio_id * c->nChannels;
                for (size_t ch = 0; ch < c->nChannels; ++ch) {
                    new (pkt[ch])
                        Packet(frame_id, sf, 0 /* cell_id */, ant_id + ch);

                    Event_data rx_message(
                        EventType::kPacketRX, rx_tag_t(tid, cursor + ch)._tag);

                    if (!message_queue_->enqueue(*local_ptok, rx_message)) {
                        printf("socket message enqueue failed\n");
                        exit(0);
                    }
                    cursor++;
                    cursor %= buffer_frame_num_;
                }
            }
        }
        frame_id++;
        if (++radio_id == radio_hi)
            radio_id = radio_lo;
    }
    return 0;
}
