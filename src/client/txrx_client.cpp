#include "txrx_client.hpp"
#include "config.hpp"

RadioTXRX::RadioTXRX(Config* cfg, int n_threads, int in_core_id)
    : config_(cfg)
    , thread_num_(n_threads)
    , core_id_(in_core_id)
{
    if (!kUseArgos && !kUseUHD) {
        socket_.resize(config_->nRadios);
        servaddr_.resize(config_->nRadios);
    } else {
        radioconfig_ = new ClientRadioConfig(config_);
    }

    /* initialize random seed: */
    srand(time(NULL));
}

RadioTXRX::RadioTXRX(Config* config, int n_threads, int in_core_id,
    moodycamel::ConcurrentQueue<Event_data>* in_message_queue,
    moodycamel::ConcurrentQueue<Event_data>* in_task_queue,
    moodycamel::ProducerToken** in_rx_ptoks,
    moodycamel::ProducerToken** in_tx_ptoks)
    : RadioTXRX(config, n_threads, in_core_id)
{
    message_queue_ = in_message_queue;
    task_queue_ = in_task_queue;
    rx_ptoks_ = in_rx_ptoks;
    tx_ptoks_ = in_tx_ptoks;
}

RadioTXRX::~RadioTXRX()
{
    if (kUseArgos || kUseUHD) {
        radioconfig_->radioStop();
        delete radioconfig_;
    }
    delete config_;
}

bool RadioTXRX::startTXRX(Table<char>& in_buffer, Table<int>& in_buffer_status,
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

    if (kUseArgos || kUseUHD)
        if (!radioconfig_->radioStart())
            return false;

    for (int i = 0; i < thread_num_; i++) {
        pthread_t txrx_thread;
        // record the thread id
        auto context = new EventHandlerContext<RadioTXRX>;
        context->obj_ptr = this;
        context->id = i;
        // start socket thread
        if (kUseArgos && config_->hw_framer) {
            if (pthread_create(&txrx_thread, NULL,
                    pthread_fun_wrapper<RadioTXRX,
                        &RadioTXRX::loop_tx_rx_argos>,
                    context)
                != 0) {
                perror("socket thread create failed");
                exit(0);
            }
        } else if (kUseArgos || kUseUHD) {
            if (pthread_create(&txrx_thread, NULL,
                    pthread_fun_wrapper<RadioTXRX,
                        &RadioTXRX::loop_tx_rx_argos_sync>,
                    context)
                != 0) {
                perror("socket thread create failed");
                exit(0);
            }
        } else {
            if (pthread_create(&txrx_thread, NULL,
                    pthread_fun_wrapper<RadioTXRX, &RadioTXRX::loop_tx_rx>,
                    context)
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

struct Packet* RadioTXRX::recv_enqueue(int tid, int radio_id, int rx_offset)
{
    moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];
    char* rx_buffer = (*buffer_)[tid];
    int* rx_buffer_status = (*buffer_status_)[tid];
    int packet_length = config_->packet_length;

    // if rx_buffer is full, exit
    if (rx_buffer_status[rx_offset] == 1) {
        printf(
            "Receive thread %d rx_buffer full, offset: %d\n", tid, rx_offset);
        config_->running = false;
        return (NULL);
    }
    struct Packet* pkt = (struct Packet*)&rx_buffer[rx_offset * packet_length];
    if (-1 == recv(socket_[radio_id], (char*)pkt, packet_length, 0)) {
        if (errno != EAGAIN && config_->running) {
            perror("recv failed");
            exit(0);
        }
        return (NULL);
    }

    // get the position in rx_buffer
    // move ptr & set status to full
    rx_buffer_status[rx_offset] = 1;

    // Push kPacketRX event into the queue.
    Event_data rx_message(EventType::kPacketRX, rx_tag_t(tid, rx_offset)._tag);
    if (!message_queue_->enqueue(*local_ptok, rx_message)) {
        printf("socket message enqueue failed\n");
        exit(0);
    }
    return pkt;
}

int RadioTXRX::dequeue_send(int tid)
{
    auto& c = config_;
    Event_data event;
    if (!task_queue_->try_dequeue_from_producer(*tx_ptoks_[tid], event))
        return -1;

    // printf("tx queue length: %d\n", task_queue_->size_approx());
    size_t ant_id = gen_tag_t(event.tags[0]).ant_id;
    size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
    if (event.event_type == EventType::kPacketTX) {
        for (size_t symbol_id = 0; symbol_id < c->ul_data_symbol_num_perframe;
             symbol_id++) {

            size_t offset
                = (c->get_total_data_symbol_idx_ul(frame_id, symbol_id)
                      * c->UE_ANT_NUM)
                + ant_id;

            if (kDebugPrintInTask) {
                printf(
                    "In TX thread %d: Transmitted frame %zu, data symbol %zu, "
                    "ant %zu, tag %zu, offset: %zu, msg_queue_length: %zu\n",
                    tid, frame_id, c->ULSymbols[0][symbol_id], ant_id,
                    gen_tag_t(event.tags[0])._tag, offset,
                    message_queue_->size_approx());
            }

            auto* pkt
                = (struct Packet*)(tx_buffer_ + offset * c->packet_length);
            new (pkt) Packet(
                frame_id, c->ULSymbols[0][symbol_id], 0 /* cell_id */, ant_id);

            // Send data (one OFDM symbol)
            ssize_t ret = sendto(socket_[ant_id], (char*)pkt, c->packet_length,
                0, (struct sockaddr*)&servaddr_[ant_id],
                sizeof(servaddr_[ant_id]));
            rt_assert(ret > 0, "sendto() failed");
        }
        rt_assert(message_queue_->enqueue(*rx_ptoks_[tid],
                      Event_data(EventType::kPacketTX, event.tags[0])),
            "Socket message enqueue failed\n");
    } else if (event.event_type == EventType::kPacketPilotTX) {
        std::vector<char> zeros(c->packet_length, 0);
        std::vector<char> pilot(c->packet_length, 0);
        memcpy(&pilot[Packet::kOffsetOfData], c->pilot_ci16.data(),
            c->packet_length - Packet::kOffsetOfData);
        for (size_t symbol_idx = 0; symbol_idx < c->pilot_symbol_num_perframe;
             symbol_idx++) {
            if (kDebugPrintInTask) {
                printf("In TX thread %d: Transmitted pilot in frame %zu, "
                       "symbol %zu, "
                       "ant %zu\n",
                    tid, frame_id, c->pilotSymbols[0][symbol_idx], ant_id);
            }

            auto* pkt = (symbol_idx == ant_id) ? (struct Packet*)pilot.data()
                                               : (struct Packet*)zeros.data();
            new (pkt) Packet(frame_id, c->pilotSymbols[0][symbol_idx],
                0 /* cell_id */, ant_id);
            // Send pilots
            ssize_t ret = sendto(socket_[ant_id], (char*)pkt, c->packet_length,
                0, (struct sockaddr*)&servaddr_[ant_id],
                sizeof(servaddr_[ant_id]));
            rt_assert(ret > 0, "sendto() failed");
        }
        rt_assert(message_queue_->enqueue(*rx_ptoks_[tid],
                      Event_data(EventType::kPacketPilotTX, event.tags[0])),
            "Socket message enqueue failed\n");
    } else {
        printf("Wrong event type in tx queue!");
        exit(0);
    }
    return event.tags[0];
}

void* RadioTXRX::loop_tx_rx(int tid)
{
    pin_to_core_with_offset(ThreadType::kWorkerTXRX, core_id_, tid);
    size_t rx_offset = 0;
    int radio_lo = tid * config_->nRadios / thread_num_;
    int radio_hi = (tid + 1) * config_->nRadios / thread_num_;
    printf("Receiver thread %d has %d radios\n", tid, radio_hi - radio_lo);

    int sock_buf_size = 1024 * 1024 * 64 * 8 - 1;
    for (int radio_id = radio_lo; radio_id < radio_hi; ++radio_id) {
        int local_port_id = config_->ue_server_port + radio_id;
        socket_[radio_id]
            = setup_socket_ipv4(local_port_id, true, sock_buf_size);
        setup_sockaddr_remote_ipv4(&servaddr_[radio_id],
            config_->ue_rru_port + radio_id, config_->bs_rru_addr.c_str());
        printf("TXRX thread %d: set up UDP socket server listening to port %d"
               " with remote address %s:%d \n",
            tid, local_port_id, config_->bs_rru_addr.c_str(),
            config_->ue_rru_port + radio_id);
        fcntl(socket_[radio_id], F_SETFL, O_NONBLOCK);
    }

    int radio_id = radio_lo;
    while (config_->running) {
        if (-1 != dequeue_send(tid))
            continue;
        // receive data
        struct Packet* pkt = recv_enqueue(tid, radio_id, rx_offset);
        if (pkt == NULL)
            continue;
        rx_offset = (rx_offset + 1) % buffer_frame_num_;

        if (++radio_id == radio_hi)
            radio_id = radio_lo;
    }
    return 0;
}

// dequeue_send_sdr
int RadioTXRX::dequeue_send_argos(int tid, long long time0)
{
    auto& c = config_;
    auto& radio = radioconfig_;
    int packet_length = c->packet_length;
    int num_samps = c->sampsPerSymbol;
    int frm_num_samps = num_samps * c->symbol_num_perframe;

    // For UHD devices, first pilot should not be with the END_BURST flag
    // 1: HAS_TIME, 2: HAS_TIME | END_BURST
    int flags_tx_pilot = (kUseUHD && c->nChannels == 2) ? 1 : 2;

    Event_data event;
    if (!task_queue_->try_dequeue_from_producer(*tx_ptoks_[tid], event))
        return -1;

    assert(event.event_type == EventType::kPacketTX);

    size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
    size_t ue_id = gen_tag_t(event.tags[0]).ant_id;
    size_t tx_frame_id = frame_id + TX_FRAME_DELTA;
    size_t ant_id = ue_id * c->nChannels;
    long long txTime(0);
    int r;

    // Transmit pilot
    if (!c->hw_framer) {
        size_t pilot_symbol_id = c->pilotSymbols[0][ant_id];

        txTime = time0 + tx_frame_id * frm_num_samps
            + pilot_symbol_id * num_samps - c->cl_tx_advance;
        r = radio->radioTx(
            ue_id, pilot_buff0.data(), num_samps, flags_tx_pilot, txTime);
        if (r < num_samps)
            std::cout << "BAD Write: (PILOT)" << r << "/" << num_samps
                      << std::endl;
        if (c->nChannels == 2) {
            pilot_symbol_id = c->pilotSymbols[0][ant_id + 1];
            txTime = time0 + tx_frame_id * frm_num_samps
                + pilot_symbol_id * num_samps - c->cl_tx_advance;
            r = radio->radioTx(ue_id, pilot_buff1.data(), num_samps, 2, txTime);
            if (r < num_samps)
                std::cout << "BAD Write (PILOT): " << r << "/" << num_samps
                          << std::endl;
        }
    }

    // Transmit data
    for (size_t symbol_id = 0; symbol_id < c->ul_data_symbol_num_perframe;
         symbol_id++) {
        size_t tx_symbol_id = c->ULSymbols[0][symbol_id];
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
        txTime = c->hw_framer
            ? ((long long)tx_frame_id << 32) | (tx_symbol_id << 16)
            : time0 + tx_frame_id * frm_num_samps + tx_symbol_id * num_samps
                - c->cl_tx_advance;
        int flags_tx_symbol = 1; // HAS_TIME
        if (tx_symbol_id == c->ULSymbols[0].back())
            flags_tx_symbol = 2; // HAS_TIME & END_BURST, fixme
        r = radio->radioTx(ue_id, txbuf, num_samps, flags_tx_symbol, txTime);
        if (r < num_samps)
            std::cout << "BAD Write (UL): " << r << "/" << num_samps
                      << std::endl;
    }

    rt_assert(message_queue_->enqueue(*rx_ptoks_[tid],
                  Event_data(EventType::kPacketTX, event.tags[0])),
        "Socket message enqueue failed\n");
    return event.tags[0];
}

struct Packet* RadioTXRX::recv_enqueue_argos(int tid, size_t radio_id,
    size_t& frame_id, size_t& symbol_id, size_t rx_offset)
{
    auto& c = config_;
    moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];
    char* rx_buffer = (*buffer_)[tid];
    int* rx_buffer_status = (*buffer_status_)[tid];
    int num_samps = c->sampsPerSymbol;
    int packet_length = c->packet_length;
    ClientRadioConfig* radio = radioconfig_;

    // if buffer is full, exit
    if (rx_buffer_status[rx_offset] == 1) {
        printf("RX thread %d at rx_offset %zu buffer full\n", tid, rx_offset);
        c->running = false;
        return NULL;
    }

    long long rxTime(0);
    struct Packet* pkt[c->nChannels];
    void* samp[c->nChannels];
    for (size_t ch = 0; ch < c->nChannels; ++ch) {
        pkt[ch]
            = (struct Packet*)&rx_buffer[((rx_offset + ch) % buffer_frame_num_)
                * packet_length];
        samp[ch] = pkt[ch]->data;
    }
    int r = radio->radioRx(radio_id, samp, num_samps, rxTime);
    if (r < num_samps)
        std::cerr << "BAD Receive(" << r << "/" << num_samps << ") at Time "
                  << rxTime << std::endl;
    if (r < 0) {
        std::cerr << "Receive error! Stopping... " << std::endl;
        c->running = false;
        return NULL;
    }
    if (c->hw_framer) {
        frame_id = (size_t)(rxTime >> 32);
        symbol_id = (size_t)((rxTime >> 16) & 0xFFFF);
    } else {
        assert(c->hw_framer
            || (((rxTime - time0) / (num_samps * c->symbol_num_perframe))
                   == frame_id));
    }
    if (kDebugPrintInTask) {
        printf("downlink receive: thread %d, frame_id %zu, symbol_id "
               "%zu, radio_id %zu "
               "rxtime %llx\n",
            tid, frame_id, symbol_id, radio_id, rxTime);
    }
    size_t ant_id = radio_id * c->nChannels;
    for (size_t ch = 0; ch < c->nChannels; ++ch) {
        new (pkt[ch]) Packet(frame_id, symbol_id, 0 /* cell_id */, ant_id + ch);

        // get the position in rx_buffer
        // move ptr & set status to full
        rx_buffer_status[rx_offset + ch] = 1;

        Event_data rx_message(
            EventType::kPacketRX, rx_tag_t(tid, rx_offset + ch)._tag);

        rt_assert(message_queue_->enqueue(*local_ptok, rx_message),
            "socket message enqueue failed");
    }

    // bool downlink = true;
    // cfo_correction(downlink);

    return pkt[0];
}

void* RadioTXRX::loop_tx_rx_argos(int tid)
{
    pin_to_core_with_offset(ThreadType::kWorkerTXRX, core_id_, tid);
    auto& c = config_;
    size_t num_radios = c->nRadios;
    size_t radio_lo = tid * num_radios / thread_num_;
    size_t radio_hi = (tid + 1) * num_radios / thread_num_;
    printf("receiver thread %d has radios %zu to %zu (%zu)\n", tid, radio_lo,
        radio_hi - 1, radio_hi - radio_lo);

    // Use mutex to sychronize data receiving across threads
    pthread_mutex_lock(&mutex);
    printf("Thread %d: waiting for release\n", tid);

    pthread_cond_wait(&cond, &mutex);
    pthread_mutex_unlock(&mutex); // unlocking for all other threads

    ClientRadioConfig* radio = radioconfig_;

    std::vector<int> all_trigs(radio_hi - radio_lo, 0);
    struct timespec tv, tv2;
    clock_gettime(CLOCK_MONOTONIC, &tv);

    size_t rx_offset = 0;
    size_t frame_id(0);
    size_t symbol_id(0);
    size_t radio_id = radio_lo;
    while (c->running) {
        clock_gettime(CLOCK_MONOTONIC, &tv2);
        double diff
            = ((tv2.tv_sec - tv.tv_sec) * 1e9 + (tv2.tv_nsec - tv.tv_nsec))
            / 1e9;
        if (diff > 2) {
            for (size_t it = radio_lo; it < radio_hi; it++) {
                int total_trigs = radio->triggers(it);
                std::cout << "radio: " << it << ", new triggers: "
                          << total_trigs - all_trigs[it - radio_lo]
                          << ", total: " << total_trigs << std::endl;
                all_trigs[it - radio_lo] = total_trigs;
            }
            tv = tv2;
        }
        // transmit data
        if (-1 != dequeue_send_argos(tid, 0))
            continue;

        struct Packet* pkt
            = recv_enqueue_argos(tid, radio_id, frame_id, symbol_id, rx_offset);
        if (pkt == NULL)
            continue;

        rx_offset += c->nChannels;
        rx_offset %= buffer_frame_num_;
        if (++radio_id == radio_hi)
            radio_id = radio_lo;
    }
    return 0;
}

void* RadioTXRX::loop_tx_rx_argos_sync(int tid)
{
    // FIXME: This only works when there is 1 radio per thread.
    pin_to_core_with_offset(ThreadType::kWorkerTXRX, core_id_, tid);
    auto& c = config_;

    // Use mutex to sychronize data receiving across threads
    pthread_mutex_lock(&mutex);
    printf("Thread %d: waiting for release\n", tid);

    pthread_cond_wait(&cond, &mutex);
    pthread_mutex_unlock(&mutex); // unlocking for all other threads

    ClientRadioConfig* radio = radioconfig_;

    int num_samps = c->sampsPerSymbol;
    int frm_num_samps = num_samps * c->symbol_num_perframe;
    std::vector<std::complex<int16_t>> frm_buff0(frm_num_samps, 0);
    std::vector<std::complex<int16_t>> frm_buff1(frm_num_samps, 0);
    std::vector<void*> frm_rx_buff(2);
    frm_rx_buff[0] = frm_buff0.data();

    pilot_buff0.resize(2);
    pilot_buff1.resize(2);
    Table<std::complex<int16_t>> zeros;
    zeros.calloc(2, c->sampsPerSymbol, 64);
    //pilot_buff0[0] = c->pilot_ci16.data();
    if (c->nChannels == 2) {
        pilot_buff0[1] = zeros[0];
        pilot_buff1[0] = zeros[1];
        //pilot_buff1[1] = c->pilot_ci16.data();
        frm_rx_buff[1] = frm_buff1.data();
    }

    long long rxTime(0);
    int radio_id = tid;
    int sync_index(-1);
    int rx_offset(0);
    size_t cursor(0);
    std::stringstream sout;

    // Keep receiving one frame of data until a beacon is found
    // Perform initial beacon detection every kBeaconDetectInterval frames
    while (c->running && sync_index < 0) {
        int r;
        for (size_t find_beacon_retry = 0;
             find_beacon_retry < kBeaconDetectInterval; find_beacon_retry++) {
            r = radio->radioRx(
                radio_id, frm_rx_buff.data(), frm_num_samps, rxTime);

            if (r != frm_num_samps) {
                std::cerr << "BAD SYNC Receive(" << r << "/" << frm_num_samps
                          << ") at Time " << rxTime << std::endl;
                continue;
            }
        }

        // convert data to complex float for sync detection
        std::vector<std::complex<float>> sync_buff(frm_num_samps, 0);
        for (int i = 0; i < frm_num_samps; i++)
            sync_buff[i] = (std::complex<float>(
                frm_buff0[i].real() / 32768.0, frm_buff0[i].imag() / 32768.0));
        sync_index = CommsLib::find_beacon_avx(sync_buff, c->gold_cf32);
        if (sync_index < 0)
            continue;
        sout << "Client " << radio_id << ": Beacon detected at Time " << rxTime
             << ", sync_index: " << sync_index << std::endl;
        std::cout << sout.str();
        sout.str(std::string()); // clear stringstream after print
        rx_offset = sync_index - c->beacon_len - c->ofdm_tx_zero_prefix_;

        // Carrier Frequency Offset estimation and correction
        cfo_estimation(sync_index, sync_buff);
    }

    // CFO "pre-distortion" on uplink pilots
    std::vector<std::complex<int16_t>> pilot_ci16_local;
    pilot_ci16_local.resize(c->sampsPerSymbol, 0);
    bool is_downlink = false;
    cfo_correction(is_downlink, c->pilot_ifft, c->OFDM_CA_NUM);
    CommsLib::ifft2tx(c->pilot_ifft,
        (std::complex<int16_t>*)pilot_ci16_local.data(), c->OFDM_CA_NUM,
        c->ofdm_tx_zero_prefix_, c->CP_LEN, c->scale);

    pilot_buff0[0] = pilot_ci16_local.data();
    if (c->nChannels == 2) {
        pilot_buff1[1] = pilot_ci16_local.data();
    }

    // Read rx_offset to align with the begining of a frame
    radio->radioRx(radio_id, frm_rx_buff.data(), rx_offset, rxTime);

    long long time0(0);
    size_t frame_id(0);

    bool resync = false;
    int resync_retry_cnt(0);
    int resync_retry_max(100);
    rx_offset = 0;
    while (c->running) {

        // recv corresponding to symbol_id = 0 (Beacon)
        int r = radio->radioRx(
            radio_id, frm_rx_buff.data(), num_samps + rx_offset, rxTime);
        if (r != num_samps + rx_offset) {
            std::cerr << "BAD Beacon Receive(" << r << "/" << num_samps
                      << ") at Time " << rxTime << std::endl;
        }
        if (r < 0) {
            std::cerr << "Receive error! Stopping... " << std::endl;
            c->running = false;
            break;
        }
        if (frame_id == 0) {
            time0 = rxTime;
        }

        // resync every X=1000 frames:
        // TODO: X should be a function of sample rate and max CFO
        if (frame_id / 1000 > 0 && frame_id % 1000 == 0) {
            resync = true;
        }
        rx_offset = 0;
        if (resync) {
            // convert data to complex float for sync detection
            std::vector<std::complex<float>> sync_buff;
            for (int i = 0; i < num_samps; i++)
                sync_buff.push_back(
                    std::complex<float>(frm_buff0[i].real() / 32768.0,
                        frm_buff0[i].imag() / 32768.0));
            sync_index = CommsLib::find_beacon_avx(sync_buff, c->gold_cf32);
            if (sync_index >= 0) {
                rx_offset
                    = sync_index - c->beacon_len - c->ofdm_tx_zero_prefix_;
                time0 += rx_offset;
                resync = false;
                resync_retry_cnt = 0;
                sout << "Client " << radio_id << ": Re-syncing with offset "
                     << rx_offset << " after " << resync_retry_cnt + 1
                     << " tries\n";
                std::cout << sout.str();
                sout.str(std::string()); // clear stringstream after print
            } else
                resync_retry_cnt++;
        }
        if (resync && resync_retry_cnt > resync_retry_max) {
            sout << "Client " << radio_id << ": Exceeded resync retry limit ("
                 << resync_retry_max << "). Stopping..." << std::endl;
            std::cerr << sout.str();
            sout.str(std::string()); // clear stringstream after print
            c->running = false;
            break;
        }

        // schedule transmit pilots and symbols
        while (-1 != dequeue_send_argos(tid, time0))
            ;

        // receive the remaining of the frame
        for (size_t symbol_id = 1; symbol_id < c->symbol_num_perframe;
             symbol_id++) {
            if (!config_->isPilot(frame_id, symbol_id)
                && !(config_->isDownlink(frame_id, symbol_id))) {
                radio->radioRx(radio_id, frm_rx_buff.data(), num_samps, rxTime);
                if (r < num_samps) {
                    std::cerr << "BAD Receive(" << r << "/" << num_samps
                              << ") at Time " << rxTime << std::endl;
                }
                if (r < 0) {
                    std::cerr << "Receive error! Stopping... " << std::endl;
                    c->running = false;
                    break;
                }
                if (kDebugPrintInTask) {
                    printf(
                        "idle receive: thread %d, frame_id %zu, symbol_id %zu, "
                        "radio_id %d "
                        "rxtime %llx\n",
                        tid, frame_id, symbol_id, radio_id, rxTime);
                }
            } else {
                struct Packet* pkt = recv_enqueue_argos(
                    tid, radio_id, frame_id, symbol_id, cursor);
                if (pkt == NULL)
                    break;

                cursor += c->nChannels;
                cursor %= buffer_frame_num_;
            }
        }
        frame_id++;
    }
    return 0;
}

void RadioTXRX::cfo_estimation(
    const int sync_index, const std::vector<std::complex<float>>& beacon_buff)
{

    auto& c = config_;

    // Compute phase error across same-sample index from two consecutive training sequences
    size_t cfo_start_idx
        = sync_index - (c->beacon_longsym_len * c->beacon_longsym_reps);
    std::vector<float> phase_vec(c->beacon_longsym_len, 0);
    std::vector<float> phase_uwrap(c->beacon_longsym_len, 0);

    /*  SEE IF WE CAN MAKE IT FASTER...
    std::vector<std::complex<float>> s1(beacon_buff.begin() + cfo_start_idx,
        beacon_buff.begin() + cfo_start_idx + c->beacon_longsym_len - 1);
    std::vector<std::complex<float>> s2(
        beacon_buff.begin() + cfo_start_idx + c->beacon_longsym_len,
        beacon_buff.begin() + cfo_start_idx + c->beacon_longsym_len
            + c->beacon_longsym_len - 1);
     */

    for (size_t i = 0; i < c->beacon_longsym_len; i++) {
        std::complex<float> s1 = beacon_buff[cfo_start_idx + i];
        std::complex<float> s2
            = beacon_buff[cfo_start_idx + c->beacon_longsym_len + i];
        std::complex<float> s12 = s1 * std::conj(s2);
        phase_vec[i] = std::atan2(s12.imag(), s12.real());

        // Unwrap phase
        if (i == 0) {
            phase_uwrap[0] = phase_vec[0];
        } else {
            float diff = phase_vec[i] - phase_vec[i - 1];
            if (diff > M_PI)
                diff = diff - 2 * M_PI;
            else if (diff < -M_PI)
                diff = diff + 2 * M_PI;

            phase_uwrap[i] = phase_vec[i - 1] + diff;
        }
    }

    float average = accumulate(phase_uwrap.begin(), phase_uwrap.end(), 0.0)
        / phase_uwrap.size();
    cfo_ = average / (2 * M_PI * c->beacon_longsym_len);
    double cfo_est_khz = cfo_ * c->rate * 1e-3;
    printf("XXXXX  CFO Estimate: %f kHz  XXXXX", cfo_est_khz);
}

void RadioTXRX::cfo_correction(
    bool is_downlink, complex_float* samps, size_t len)
{
    const std::complex<double> i(0.0, 1.0);
    std::complex<float> samps_cf;

    double cfo = is_downlink ? cfo_ : -1 * cfo_;

    for (size_t idx = 0; idx < len; idx++) {
        double tmp = 2 * M_PI * cfo * idx;
        std::complex<double> cfo_tmp = exp(tmp * -i);
        samps_cf = std::complex<float>(samps[idx].re, samps[idx].im)
            * (std::complex<float>)cfo_tmp;
        samps[idx].re = samps_cf.real();
        samps[idx].im = samps_cf.imag();
    }
}
