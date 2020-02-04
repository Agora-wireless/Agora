
#include "ru.hpp"
#include "config.hpp"
#ifndef SIM
#include "radio_lib.hpp"
#endif

RU::RU(int n_rx_thread, int n_tx_thread, Config* cfg)
{
    config_ = cfg;
#ifdef SIM
    rx_socket_ = new int[n_rx_thread];

    for (int i = 0; i < n_rx_thread; i++) {
        /* Create sockets at different port numbers */
        servaddr_[i].sin_family = AF_INET;
        servaddr_[i].sin_port = htons(config_->rx_port + i);
        servaddr_[i].sin_addr.s_addr = INADDR_ANY;
        memset(servaddr_[i].sin_zero, 0, sizeof(servaddr_[i].sin_zero));

        /* RX UDP socket */
        if ((rx_socket_[i] = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
            printf("cannot create socket %d\n", i);
            exit(0);
        }

        /* use SO_REUSEPORT option, so that multiple sockets could receive packets simultaneously, though the load is not balance */
        int optval = 1;
        setsockopt(rx_socket_[i], SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));

        int sock_buf_size = 1024 * 1024 * 64 * 8;
        if (setsockopt(rx_socket_[i], SOL_SOCKET, SO_RCVBUF, (void*)&sock_buf_size, sizeof(sock_buf_size)) < 0) {
            printf("Error setting buffer size to %d\n", sock_buf_size);
        }

        if (bind(rx_socket_[i], (struct sockaddr*)&servaddr_[i], sizeof(servaddr_[i])) != 0) {
            printf("rx socket %d bind failed\n", i);
            exit(0);
        } else
            printf("rx socket %d bind to port %d successful\n", i, config_->rx_port + i);
    }

    tx_socket_ = new int[n_tx_thread];

    for (int i = 0; i < n_tx_thread; i++) {

        cliaddr_[i].sin_family = AF_INET;
        cliaddr_[i].sin_port = htons(config_->tx_port + i);
        cliaddr_[i].sin_addr.s_addr = inet_addr(config_->tx_addr.c_str());
        //cliaddr_.sin_addr.s_addr = htons(INADDR_ANY);
        memset(cliaddr_[i].sin_zero, 0, sizeof(cliaddr_[i].sin_zero));
        if ((tx_socket_[i] = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
            printf("cannot create socket %d\n", i);
            exit(0);
        }
    }
#else
    radioconfig_ = new ClientRadioConfig(config_);

#endif

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
#ifdef SIM
    delete[] rx_socket_;
    delete[] tx_socket_;
#else
    radioconfig_->radioStop();
    delete radioconfig_;
#endif
    delete config_;
}

void RU::startRadios()
{
#ifndef SIM
    radioconfig_->radioStart();
#endif
}

std::vector<pthread_t> RU::startProc(Table<char>& in_buffer, Table<int>& in_buffer_status, int in_buffer_frame_num, int in_buffer_length, int in_core_id)
{
    // check length
    buffer_frame_num_ = in_buffer_frame_num;
    assert(in_buffer_length == config_->packet_length * buffer_frame_num_); // should be integer
    buffer_length_ = in_buffer_length;
    buffer_ = &in_buffer; // for save data
    buffer_status_ = &in_buffer_status; // for save status

    core_id_ = in_core_id;

    std::vector<pthread_t> created_threads;
    for (int i = 0; i < thread_num_; i++) {
        pthread_t proc_thread_;
        // record the thread id
        RUContext* context = new RUContext;
        context->ptr = this;
        context->tid = i;
        // start socket thread
        if (pthread_create(&proc_thread_, NULL, taskThread_launch, context) != 0) {
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

std::vector<pthread_t> RU::startTX(char* in_buffer, char* in_pilot_buffer, int* in_buffer_status, int in_buffer_frame_num, int in_buffer_length, int in_core_id)
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
#ifdef SEPARATE_TX_THREAD
    printf("start Transmit thread\n");
    for (int i = 0; i < tx_thread_num_; i++) {
        pthread_t send_thread_;
        RUContext* context = new RUContext;
        context->ptr = this;
        context->tid = i;
        if (pthread_create(&send_thread_, NULL, sendThread_launch, context) != 0) {
            perror("socket Transmit thread create failed");
            exit(0);
        }
        created_threads.push_back(send_thread_);
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
        printf("TX thread: stitch thread %d to core %d failed\n", tid, core_id_ + tid);
        exit(0);
    } else {
        printf("TX thread: stitch thread %d to core %d succeeded\n", tid, core_id_ + tid);
    }
#endif

    int packet_length = config_->packet_length;
#ifndef SIM
    ClientRadioConfig* radio = radioconfig_;
    packet_length -= offsetof(Packet, data);
#endif

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
        //ret = task_queue_.try_dequeue(task_event);
        ret = task_queue_->try_dequeue_from_producer(*task_ptok[tid], task_event);
        if (!ret)
            continue;

        // printf("tx queue length: %d\n", task_queue_.size_approx());
        if (task_event.event_type != TASK_SEND) {
            printf("Wrong event type!");
            exit(0);
        }

        ant_id = task_event.data; //% config_->getNumAntennas();
        //frame_id = task_event.more_data;

#ifdef SIM
        // first sending pilots in sim mode
        //for (int p_id = 0; p_id < cfg->pilotSymsPerFrame; p_id++)
        {
            struct Packet* pkt = (struct Packet*)pilot_buffer_;
            new (pkt) Packet(frame_id, config_->pilotSymbols[0][ant_id], 0, ant_id);
            //ru_->send((void *)ul_pilot_aligned, cfg->getTxPackageLength(), frame_id, cfg->pilotSymbols[0][p_id], p_id);
            if (sendto(tx_socket_[tid], (char*)obj_ptr->pilot_buffer_, packet_length, 0, (struct sockaddr*)&obj_ptr->cliaddr_[tid], sizeof(obj_ptr->cliaddr_[tid])) < 0) {
                perror("loopSend: socket sendto failed");
                exit(0);
            }
        }
#if DEBUG_SEND
        printf("TX thread %d: finished TX pilot for frame %d at symbol %d on ant %d\n", tid, frame_id, config_->pilotSymbols[0][ant_id], ant_id);
#endif
    }
    for (symbol_id = 0; symbol_id < txSymbols.size(); symbol_id++) {
        //for (ant_id = 0; ant_id < config_->getNumAntennas(); ant_id++)
        {
            offset = generateOffset3d(TASK_BUFFER_FRAME_NUM, txSymbols.size(), config_->getNumAntennas(), frame_id, symbol_id, ant_id);
            // send data (one OFDM symbol)
            struct Packet* pkt = (struct Packet*)(tx_buffer_ + offset * packet_length);
            new (pkt) Packet(frame_id, txSymbols[symbol_id], cell_id, ant_id);

            if (sendto(tx_socket_[tid], (char*)pkt, packet_length, 0, (struct sockaddr*)&cliaddr_[tid], sizeof(cliaddr_[tid])) < 0) {
                perror("loopSend: socket sendto failed");
                exit(0);
            }
        }
#if DEBUG_SEND
        printf("TX thread %d: finished TX for frame %d, symbol %d, ant %d\n", tid, frame_id, txSymbols[symbol_id], ant_id);
#endif
    }
#else
        //symbol_id = task_event.data / config_->getNumAntennas();
        for (symbol_id = 0; symbol_id < txSymbols.size(); symbol_id++) {
            int tx_frame_id = frame_id;
            size_t tx_symbol_id = txSymbols[symbol_id];
            offset = generateOffset3d(TASK_BUFFER_FRAME_NUM, txSymbols.size(), config_->getNumAntennas(), frame_id, symbol_id, ant_id);
            void* txbuf[2];
            long long frameTime = ((long long)tx_frame_id << 32) | (tx_symbol_id << 16);
            int flags = 1; // HAS_TIME
            if (tx_symbol_id == txSymbols.back())
                flags = 2; // HAS_TIME & END_BURST, fixme
            for (size_t ch = 0; ch < config_->nChannels; ++ch) {
                txbuf[ch] = tx_buffer_ + (offset + ch) * packet_length; //   pilot_buffer_; //
                tx_buffer_status_[offset + ch] = 0;
            }
#if DEBUG_SEND
            printf("RU: transmit tx_frame_id %d, tx_symbol_id %d, cell_id %d, ant_id %d\n", frame_id, symbol_id, cell_id, ant_id);
            printf("transmit samples: %f %f %f %f %f %f %f %f ...\n", *((RadioBufRealType*)txbuf[0] + 2 * config_->prefix + 9),
                *((RadioBufRealType*)txbuf[0] + 2 * config_->prefix + 10),
                *((RadioBufRealType*)txbuf[0] + 2 * config_->prefix + 11),
                *((RadioBufRealType*)txbuf[0] + 2 * config_->prefix + 12),
                *((RadioBufRealType*)txbuf[0] + 2 * config_->prefix + 13),
                *((RadioBufRealType*)txbuf[0] + 2 * config_->prefix + 14),
                *((RadioBufRealType*)txbuf[0] + 2 * config_->prefix + 15),
                *((RadioBufRealType*)txbuf[0] + 2 * config_->prefix + 16));
#endif
            clock_gettime(CLOCK_MONOTONIC, &tv);
            radio->radioTx(ant_id / config_->nChannels, txbuf, flags, frameTime);
            clock_gettime(CLOCK_MONOTONIC, &tv2);
#if TX_TIME_MEASURE
            double diff = (tv2.tv_sec * 1e9 + tv2.tv_nsec - tv.tv_sec * 1e9 - tv.tv_nsec);
            time_avg += diff;
            time_count++;
            if (time_count == config_->getNumAntennas() * config_->dl_data_symbol_num_perframe) {
                printf("In TX thread %d: Transmitted 100 frames with average tx time %f\n", tid, time_avg / time_count / 1e3);
                time_count = 0;
                time_avg = 0;
            }
#endif

#if DEBUG_SEND
            printf("TX thread %d: finished TX for frame %d, symbol %d, ant %d\n", tid, frame_id, symbol_id, ant_id);
#endif
        }
#endif

    Event_data packet_message;
    packet_message.event_type = EVENT_PACKET_SENT;
    packet_message.data = offset;
    //packet_message.more_data = frame_id;
    if (config_->running && !message_queue_->enqueue(local_ptok, packet_message)) {
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
    // printf("Recv thread: pinning thread %d to core %d\n", tid, core_id_ + tid);
    if (pin_to_core(core_id_ + tid) != 0) {
        printf("Recv thread: pinning thread %d to core %d failed\n", tid, core_id_ + tid);
        exit(0);
    } else {
        printf("Recv thread: pinning thread %d to core %d succeed\n", tid, core_id_ + tid);
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
    //moodycamel::ProducerToken local_ctok(task_queue_);
    //moodycamel::ProducerToken *local_ctok = (task_ptok[tid]);

    const std::vector<size_t>& txSymbols = config_->ULSymbols[0];
    int n_ant = config_->getNumAntennas();

    char* buffer = (*buffer_)[tid];
    int* buffer_status = (*buffer_status_)[tid];

#ifndef SIM
    int num_radios = config_->nRadios;
    int radio_start = tid * num_radios / thread_num_;
    int radio_end = (tid + 1) * num_radios / thread_num_;
    printf("receiver thread %d has %d radios\n", tid, radio_end - radio_start);
    ClientRadioConfig* radio = radioconfig_;
#else
    // loop recv
    socklen_t addrlen = sizeof(servaddr_[tid]);
#endif
    int packet_num = 0;
    long long frameTime;

    int all_trigs = 0;
    struct timespec tv, tv2;
    clock_gettime(CLOCK_MONOTONIC, &tv);

    int maxQueueLength = 0;
    int cursor = 0;
    while (config_->running) {
        clock_gettime(CLOCK_MONOTONIC, &tv2);
        double diff = ((tv2.tv_sec - tv.tv_sec) * 1e9 + (tv2.tv_nsec - tv.tv_nsec)) / 1e9;
        if (diff > 2) {
            int total_trigs = radio->triggers(tid);
            std::cout << "new triggers: " << total_trigs - all_trigs << ", total: " << total_trigs << std::endl;
            all_trigs = total_trigs;
            tv = tv2;
        }
        // if buffer is full, exit
        if (buffer_status[cursor] == 1) {
            printf("RX thread %d at cursor %d buffer full\n", tid, cursor);
            //exit(0);
            for (size_t a = 0; a < buffer_frame_num_; a++) {
		printf("%d ", buffer_status[a]);
            }
            printf("");
            config_->running = false;
            break;
        }
        // receive data
#ifdef SIM
        struct Packet* pkt = (struct Packet*)&buffer[cursor * config_->packet_length];
        if (recvfrom(rx_socket_[tid], (char*)pkt, (size_t)config_->packet_length, 0,
                (struct sockaddr*)&servaddr_[tid], &addrlen)
            < 0) {
            perror("recv failed");
            exit(0);
        }
        // read information from received packet
        int frame_id = pkt->frame_id;
        int symbol_id = pkt->symbol_id;
        //int cell_id = pkt->cell_id;
        int ant_id = pkt->ant_id;

#if DEBUG_RECV
        printf("RU: receive frame_id %d, symbol_id %d, cell_id %d, ant_id %d\n",
            frame_id, symbol_id, cell_id, ant_id);
        //printf("receive samples: %f %f %f %f %f %f %f %f ...\n",
        //        *((RadioBufRealType *)&pkt->data[1],
        //        *((RadioBufRealType *)&pkt->data[2],
        //        *((RadioBufRealType *)&pkt->data[3],
        //        *((RadioBufRealType *)&pkt->data[4],
        //        *((RadioBufRealType *)&pkt->data[5],
        //        *((RadioBufRealType *)&pkt->data[6],
        //        *((RadioBufRealType *)&pkt->data[7],
        //        *((RadioBufRealType *)&pkt->data[8]);
#endif
        // move ptr & set status to full
        buffer_status[cursor] = 1; // has data, after doing fft, it is set to 0
        cursor++;
        cursor %= buffer_frame_num_;

        // push EVENT_RX_ENB event into the queue
        Event_data packet_message;
        packet_message.event_type = EVENT_RX_SYMBOL;
        // data records the position of this packet in the buffer & tid of this socket (so that task thread could know which buffer it should visit)
        packet_message.data = cursor + tid * buffer_frame_num_;
        if (!message_queue_->enqueue(local_ptok, packet_message)) {
            printf("socket message enqueue failed\n");
            exit(0);
        }

        if (txSymbols.size() > 0 && config_->getDlSFIndex(frame_id, symbol_id) == 0) {
            // notify TXthread to start transmitting frame_id+offset
            Event_data do_tx_task;
            do_tx_task.event_type = TASK_SEND;
            do_tx_task.data = ant_id;
            do_tx_task.more_data = frame_id + TX_FRAME_DELTA;
            if (!task_queue_->enqueue(*task_ptok[tid], do_tx_task)) {
                printf("task enqueue failed\n");
                exit(0);
            }
        }

        //printf("enqueue offset %d\n", offset);
        int cur_queue_len = message_queue_->size_approx();
        maxQueueLength = maxQueueLength > cur_queue_len ? maxQueueLength : cur_queue_len;

        packet_num++;
#else
        for (int it = radio_start; it < radio_end; it++) // FIXME: this must be threaded
        {
            // this is probably a really bad implementation, and needs to be revamped
            struct Packet* pkt[config_->nChannels];
            void* samp[config_->nChannels];
            for (size_t ch = 0; ch < config_->nChannels; ++ch) {
                pkt[ch] = (struct Packet*)&buffer[(cursor + ch) * config_->packet_length];
                samp[ch] = pkt[ch]->data;
            }
            while (config_->running && radio->radioRx(it, samp, frameTime) < config_->sampsPerSymbol)
                ;
            int frame_id = (int)(frameTime >> 32);
            int symbol_id = (int)((frameTime >> 16) & 0xFFFF);
            int ant_id = it * config_->nChannels;
#if DEBUG_RECV
            printf("receive thread %d: frame_id %d, symbol_id %d, ant_id %d frametime %llx\n",
                tid, frame_id, symbol_id, ant_id, frameTime);
#endif
            for (size_t ch = 0; ch < config_->nChannels; ++ch) {
                new (pkt[ch]) Packet(frame_id, symbol_id, 0 /* cell_id */, ant_id + ch);
                buffer_status[cursor + ch] = 1; // has data, after it is read it should be set to 0
                // push EVENT_RX_ENB event into the queue
                Event_data packet_message;
                packet_message.event_type = EVENT_PACKET_RECEIVED;
                // data records the position of this packet in the buffer & tid of this socket (so that task thread could know which buffer it should visit)
                packet_message.data = cursor + tid * buffer_frame_num_;
                if (!message_queue_->enqueue(local_ptok, packet_message)) {
                    printf("socket message enqueue failed\n");
                    exit(0);
                }
                cursor++;
                cursor %= buffer_frame_num_;
            }

            // notify TXthread to start transmitting frame_id+offset
            if (txSymbols.size() > 0 && config_->getDlSFIndex(frame_id, symbol_id) == 0) {
                //#ifdef SEPARATE_TX_THREAD
                //                Event_data do_tx_task;
                //                do_tx_task.event_type = TASK_packet_SENT;
                //                do_tx_task.data = ant_id; //tx_symbol_id * config_->getNumAntennas() + ant_id;
                //                do_tx_task.more_data = frame_id + TX_FRAME_DELTA;
                //                if ( !task_queue_.enqueue(*task_ptok[tid], do_tx_task)) {
                //                    printf("task enqueue failed\n");
                //                    exit(0);
                //                }
                //#else
                for (size_t tx_symbol_id = 0; tx_symbol_id < txSymbols.size(); tx_symbol_id++) {
                    int tx_frame_id = frame_id + TX_FRAME_DELTA;
                    int tx_frame_offset = tx_frame_id % TASK_BUFFER_FRAME_NUM;
                    size_t tx_symbol = txSymbols[tx_symbol_id];
                    //int tx_offset = generateOffset3d(TASK_BUFFER_FRAME_NUM, txSymbols.size(), config_->getNumAntennas(), tx_frame_id, tx_symbol_id, ant_id);
                    int tx_packet_length = config_->packet_length - offsetof(Packet, data);
                    int frame_samp_size = tx_packet_length * n_ant * txSymbols.size();
                    int tx_offset = tx_frame_offset * frame_samp_size + tx_packet_length * (n_ant * tx_symbol_id + ant_id);
                    void* txbuf[2];
                    long long frameTime = ((long long)tx_frame_id << 32) | (tx_symbol << 16);
                    int flags = 1; // HAS_TIME
                    if (tx_symbol == txSymbols.back())
                        flags = 2; // HAS_TIME & END_BURST
                    for (size_t ch = 0; ch < config_->nChannels; ++ch)
                        txbuf[ch] = (void*)(tx_buffer_ + tx_offset + ch * tx_packet_length);
#if DEBUG_SEND
                    int start_ind = 2 * config_->prefix;
                    printf("transmit samples: %d %d %d %d %d %d %d %d ...\n\n",
                        *((short*)txbuf[0] + start_ind + 0),
                        *((short*)txbuf[0] + start_ind + 1),
                        *((short*)txbuf[0] + start_ind + 2),
                        *((short*)txbuf[0] + start_ind + 3),
                        *((short*)txbuf[0] + start_ind + 4),
                        *((short*)txbuf[0] + start_ind + 5),
                        *((short*)txbuf[0] + start_ind + 6),
                        *((short*)txbuf[0] + start_ind + 7));
#endif
                    radio->radioTx(it, txbuf, flags, frameTime);
                }
                //#endif
            }
            //stats

            //printf("enqueue offset %d\n", offset);
            int cur_queue_len = message_queue_->size_approx();
            maxQueueLength = maxQueueLength > cur_queue_len ? maxQueueLength : cur_queue_len;

            packet_num++;
        }
#endif
    }
}
