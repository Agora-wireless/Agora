
#include "ru.hpp"
#include "config.hpp"

RU::RU(int n_rx_thread, int n_tx_thread, Config* cfg)
{
    config_ = cfg;
    rx_socket_ = new int[n_rx_thread];

    for (int i = 0; i < n_rx_thread; i++) {
        /* Create sockets at different port numbers */
        servaddr_[i].sin_family = AF_INET;
        servaddr_[i].sin_port = htons(config_->bs_port + i);
        servaddr_[i].sin_addr.s_addr = INADDR_ANY;
        memset(servaddr_[i].sin_zero, 0, sizeof(servaddr_[i].sin_zero));

        /* RX UDP socket */
        if ((rx_socket_[i] = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
            printf("cannot create socket %d\n", i);
            exit(0);
        }

        /* use SO_REUSEPORT option, so that multiple sockets could receive
         * packets simultaneously, though the load is not balance */
        int optval = 1;
        setsockopt(
            rx_socket_[i], SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));

        int sock_buf_size = 1024 * 1024 * 64 * 8;
        if (setsockopt(rx_socket_[i], SOL_SOCKET, SO_RCVBUF,
                (void*)&sock_buf_size, sizeof(sock_buf_size))
            < 0) {
            printf("Error setting buffer size to %d\n", sock_buf_size);
        }

        if (bind(rx_socket_[i], (struct sockaddr*)&servaddr_[i],
                sizeof(servaddr_[i]))
            != 0) {
            printf("rx socket %d bind failed\n", i);
            exit(0);
        } else
            printf("rx socket %d bind to port %d successful\n", i,
                config_->bs_port + i);
    }

    tx_socket_ = new int[n_tx_thread];

    for (int i = 0; i < n_tx_thread; i++) {

        cliaddr_[i].sin_family = AF_INET;
        cliaddr_[i].sin_port = htons(config_->ue_tx_port + i);
        cliaddr_[i].sin_addr.s_addr = inet_addr(config_->client_addr.c_str());
        // cliaddr_.sin_addr.s_addr = htons(INADDR_ANY);
        memset(cliaddr_[i].sin_zero, 0, sizeof(cliaddr_[i].sin_zero));
        if ((tx_socket_[i] = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
            printf("cannot create socket %d\n", i);
            exit(0);
        }
    }

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
    delete[] rx_socket_;
    delete[] tx_socket_;
    delete config_;
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
#if SEPARATE_TX_RX_CLIENT
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

        // first sending pilots in sim mode
        // for (int p_id = 0; p_id < cfg->pilotSymsPerFrame; p_id++)
        {
            auto* pkt = (struct Packet*)pilot_buffer_;
            new (pkt)
                Packet(frame_id, config_->pilotSymbols[0][ant_id], 0, ant_id);
            // ru_->send((void *)ul_pilot_aligned, cfg->getTxPackageLength(),
            // frame_id, cfg->pilotSymbols[0][p_id], p_id);
            if (sendto(tx_socket_[tid], (char*)pilot_buffer_, packet_length, 0,
                    (struct sockaddr*)&cliaddr_[tid], sizeof(cliaddr_[tid]))
                < 0) {
                perror("loopSend: socket sendto failed");
                exit(0);
            }
        }
        for (symbol_id = 0; symbol_id < txSymbols.size(); symbol_id++) {
            offset = generateOffset3d(TASK_BUFFER_FRAME_NUM, txSymbols.size(),
                config_->getNumAntennas(), frame_id, symbol_id, ant_id);
            // send data (one OFDM symbol)
            auto* pkt = (struct Packet*)(tx_buffer_ + offset * packet_length);
            new (pkt) Packet(frame_id, txSymbols[symbol_id], cell_id, ant_id);

            if (sendto(tx_socket_[tid], (char*)pkt, packet_length, 0,
                    (struct sockaddr*)&cliaddr_[tid], sizeof(cliaddr_[tid]))
                < 0) {
                perror("loopSend: socket sendto failed");
                exit(0);
            }
        }
#if DEBUG_SEND
        printf("TX thread %d: finished TX pilot for frame %d at symbol %d on "
               "ant %d\n",
            tid, frame_id, config_->pilotSymbols[0][ant_id], ant_id);
#endif
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
    int n_ant = config_->getNumAntennas();

    char* buffer = (*buffer_)[tid];
    int* buffer_status = (*buffer_status_)[tid];

    // loop recv
    socklen_t addrlen = sizeof(servaddr_[tid]);
    int packet_num = 0;
    long long frameTime;

    int all_trigs = 0;
    struct timespec tv, tv2;

    int maxQueueLength = 0;
    int cursor = 0;
    while (config_->running) {
        // if buffer is full, exit
        if (buffer_status[cursor] == 1) {
            printf("RX thread %d at cursor %d buffer full\n", tid, cursor);
            // exit(0);
            config_->running = false;
            break;
        }
        // receive data
        struct Packet* pkt
            = (struct Packet*)&buffer[cursor * config_->packet_length];
        if (recvfrom(rx_socket_[tid], (char*)pkt,
                (size_t)config_->packet_length, 0,
                (struct sockaddr*)&servaddr_[tid], &addrlen)
            < 0) {
            perror("recv failed");
            exit(0);
        }
        // read information from received packet
        int frame_id = pkt->frame_id;
        int symbol_id = pkt->symbol_id;
        // int cell_id = pkt->cell_id;
        int ant_id = pkt->ant_id;

        // move ptr & set status to full
        buffer_status[cursor] = 1; // has data, after doing fft, it is set to 0
        cursor++;
        cursor %= buffer_frame_num_;

        // Push EVENT_RX_ENB event into the queue. data records the position of
        // this packet in the buffer & tid of this socket (so that task thread
        // could know which buffer it should visit)
        Event_data packet_message(
            EventType::kRXSymbol, cursor + tid * buffer_frame_num_);

        if (!message_queue_->enqueue(local_ptok, packet_message)) {
            printf("socket message enqueue failed\n");
            exit(0);
        }

#if SEPARATE_TX_RX_CLIENT
        if (txSymbols.size() > 0
            && config_->get_dl_symbol_idx(frame_id, symbol_id) == 0) {
            // notify TXthread to start transmitting frame_id+offset
            Event_data do_tx_task(EventType::kPacketTX, ant_id);
            do_tx_task.more_data = frame_id + TX_FRAME_DELTA;

            if (!task_queue_->enqueue(*task_ptok[tid], do_tx_task)) {
                printf("task enqueue failed\n");
                exit(0);
            }
        }
#endif
        // printf("enqueue offset %d\n", offset);
        int cur_queue_len = message_queue_->size_approx();
        maxQueueLength
            = maxQueueLength > cur_queue_len ? maxQueueLength : cur_queue_len;

        packet_num++;
    }
}
