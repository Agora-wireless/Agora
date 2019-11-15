
#include "ru.hpp"

RU::RU(int n_rx_thread, int n_tx_thread, Config *cfg)
{
    config_ = cfg;
#ifdef SIM
    rx_socket_ = new int[n_rx_thread]; 
    
    for(int i = 0; i < n_rx_thread; i++) {
        /* Create sockets at different port numbers */
        servaddr_[i].sin_family = AF_INET;
        servaddr_[i].sin_port = htons(config_->rx_port+i);
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

        int sock_buf_size = 1024*1024*64*8;
        if (setsockopt(rx_socket_[i], SOL_SOCKET, SO_RCVBUF, (void*)&sock_buf_size, sizeof(sock_buf_size))<0)
        {
            printf("Error setting buffer size to %d\n", sock_buf_size);
        }

        if(bind(rx_socket_[i], (struct sockaddr *) &servaddr_[i], sizeof(servaddr_[i])) != 0)
        {
            printf("rx socket %d bind failed\n", i);
            exit(0);
        }
        else
            printf("rx socket %d bind to port %d successful\n", i, config_->rx_port+i);
    }


    tx_socket_ = new int[n_tx_thread]; 
    
    for(int i = 0; i < n_tx_thread; i++) {

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
    radioconfig_ = new RadioConfig(config_);

#endif    

    thread_num_ = n_rx_thread;
    tx_thread_num_ = n_tx_thread;
    /* initialize random seed: */
    srand (time(NULL));
    context = new RUContext[thread_num_];

}

RU::RU(int n_rx_thread, int n_tx_thread, Config *config, moodycamel::ConcurrentQueue<Event_data> * in_queue, moodycamel::ConcurrentQueue<Event_data> * in_queue_task):
RU(n_rx_thread,n_tx_thread, config)
{
    message_queue_ = in_queue;
    task_queue_ = in_queue_task;
    task_ptok.resize(thread_num_);
    for (int i = 0; i < thread_num_; i++)
        task_ptok[i].reset(new moodycamel::ProducerToken(*task_queue_));

}

RU::~RU()
{
    delete[] context;
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

std::vector<pthread_t> RU::startProc(void** in_buffer, int** in_buffer_status, int in_buffer_frame_num, int in_buffer_length, int in_core_id)
{
    // check length
    buffer_frame_num_ = in_buffer_frame_num;
    assert(in_buffer_length == config_->packet_length * buffer_frame_num_); // should be integer
    buffer_length_ = in_buffer_length;
    buffer_ = in_buffer;  // for save data
    buffer_status_ = in_buffer_status; // for save status

    core_id_ = in_core_id;

#ifndef SIM
    int nradio_per_thread = config_->nRadios/thread_num_;
    int rem_thread_nradio = config_->nRadios%thread_num_;
#endif
     
    std::vector<pthread_t> created_threads;
    for(int i = 0; i < thread_num_; i++) {
        pthread_t proc_thread_;
        // record the thread id 
        context[i].ptr = this;
        context[i].tid = i;
#ifndef SIM
        context[i].radios = (i < rem_thread_nradio) ? nradio_per_thread + 1 : nradio_per_thread;
#endif
        // start socket thread
        if(pthread_create( &proc_thread_, NULL, RU::loopProc, (void *)(&context[i])) != 0) {
            perror("socket thread create failed");
            exit(0);
        }
        else {
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
    tx_buffer_ = in_buffer;  // for save data
    pilot_buffer_ = in_pilot_buffer;
    tx_buffer_status_ = in_buffer_status; // for save status

    // create new threads
    std::vector<pthread_t> created_threads;
    tx_core_id_ = in_core_id;
#ifdef SEPARATE_TX_THREAD
    printf("start Transmit thread\n");
    for (int i = 0; i < tx_thread_num_; i++) {
        pthread_t send_thread_;
        
        context[i].ptr = this;
        context[i].tid = i;

        if (pthread_create( &send_thread_, NULL, RU::loopSend, (void *)(&context[i])) != 0) {
            perror("socket Transmit thread create failed");
            exit(0);
        }
        created_threads.push_back(send_thread_);
    }
    
#endif
    return created_threads;
}


/*****  transmit threads   *****/

void* RU::loopSend(void *in_context)
{
    RU* obj_ptr = ((RUContext *)in_context)->ptr;
    int tid = ((RUContext *)in_context)->tid;
    printf("packet sender thread %d start\n", tid);

    moodycamel::ConcurrentQueue<Event_data> *task_queue_ = obj_ptr->task_queue_;
    // get pointer to message queue
    moodycamel::ConcurrentQueue<Event_data> *message_queue_ = obj_ptr->message_queue_;
    int core_id = obj_ptr->tx_core_id_;

#ifdef ENABLE_CPU_ATTACH
    if(pin_to_core(core_id + tid) != 0) {
        printf("TX thread: stitch thread %d to core %d failed\n", tid, core_id+ tid);
        exit(0);
    }
    else {
        printf("TX thread: stitch thread %d to core %d succeeded\n", tid, core_id + tid);
    }
#endif

    // downlink socket buffer
    char *buffer = obj_ptr->tx_buffer_;
    //int buffer_frame_num = obj_ptr->tx_buffer_frame_num_;
    //int buffer_length = obj_ptr->tx_buffer_length_;
    int* buffer_status = obj_ptr->tx_buffer_status_;

    Config *cfg = obj_ptr->config_;
#ifdef SIM
    int* tx_socket_ = obj_ptr->tx_socket_;
    char *cur_ptr_buffer;
#else 
    RadioConfig *radio = obj_ptr->radioconfig_;
#endif
    int packet_length = cfg->packet_length;
#ifndef SIM
    packet_length -= cfg->packet_header_offset;
#endif

    int ret;
    int ant_id, symbol_id;
    int offset = 0;
    int frame_id = 0;
    struct timespec tv, tv2;
#if TX_TIME_MEASURE
    double time_avg = 0;
    int time_count = 0;
#endif

    int txSymsPerFrame = 0;
    std::vector<size_t> txSymbols;
    if (cfg->isUE)
    {
        txSymsPerFrame = cfg->ulSymsPerFrame;
        txSymbols = cfg->ULSymbols[0];
    }
    else
    {
        txSymsPerFrame = cfg->dlSymsPerFrame;
        txSymbols = cfg->DLSymbols[0];
    }

    // use token to speed up
    moodycamel::ProducerToken local_ptok(*message_queue_);
    while(cfg->running) {
    
        Event_data task_event;
        //ret = task_queue_->try_dequeue(task_event); 
        ret = task_queue_->try_dequeue_from_producer(*obj_ptr->task_ptok[tid], task_event); 
        if(!ret)
            continue;

        // printf("tx queue length: %d\n", task_queue_->size_approx());
        if (task_event.event_type!=TASK_SEND) {
            printf("Wrong event type!");
            exit(0);
        }

        ant_id = task_event.data; //% cfg->getNumAntennas();
        //frame_id = task_event.more_data; 

#ifdef SIM
        if(cfg->isUE)
        {
            // first sending pilots in sim mode
            //for (int p_id = 0; p_id < cfg->pilotSymsPerFrame; p_id++)
            {
                int* tx_buffer_hdr = (int*)obj_ptr->pilot_buffer_; //.data();
                tx_buffer_hdr[0] = frame_id;    
                tx_buffer_hdr[1] = cfg->pilotSymbols[0][ant_id];  
                tx_buffer_hdr[2] = 0; //cell_id
                tx_buffer_hdr[3] = ant_id; // rsvd  
                //ru_->send((void *)ul_pilot_aligned, cfg->getTxPackageLength(), frame_id, cfg->pilotSymbols[0][p_id], p_id);
                if (sendto(tx_socket_[tid], (char *)obj_ptr->pilot_buffer_, packet_length, 0, (struct sockaddr *)&obj_ptr->cliaddr_[tid], sizeof(obj_ptr->cliaddr_[tid])) < 0) {
                    perror("loopSend: socket sendto failed");
                    exit(0);
                }
            }
#if DEBUG_SEND
            printf("TX thread %d: finished TX pilot for frame %d at symbol %d on ant %d\n", tid, frame_id, cfg->pilotSymbols[0][ant_id], ant_id);
#endif
        }
        for (symbol_id = 0; symbol_id < txSymsPerFrame; symbol_id++)
        {
            //for (ant_id = 0; ant_id < cfg->getNumAntennas(); ant_id++) 
            {
                offset = generateOffset3d(TASK_BUFFER_FRAME_NUM, txSymsPerFrame, cfg->getNumAntennas(), frame_id, symbol_id, ant_id);
                cur_ptr_buffer = buffer + offset * packet_length;  
                // send data (one OFDM symbol)
		struct Packet *pkt = (struct Packet *)cur_ptr_buffer;
		new (cur_ptr_buffer) Packet(frame_id, txSymbols[symbol_id], cell_id, ant_id);

                if (sendto(tx_socket_[tid], (char *)cur_ptr_buffer, packet_length, 0, (struct sockaddr *)&obj_ptr->cliaddr_[tid], sizeof(obj_ptr->cliaddr_[tid])) < 0) {
                    perror("loopSend: socket sendto failed");
                    exit(0);
                }
            }
#if DEBUG_SEND
            printf("TX thread %d: finished TX for frame %d, symbol %d, ant %d\n", tid, frame_id, txSymbols[symbol_id], ant_id);
#endif
        }
#else
        //symbol_id = task_event.data / cfg->getNumAntennas();
        for (symbol_id = 0; symbol_id < txSymsPerFrame; symbol_id++)
        {
            int tx_frame_id = frame_id;
            size_t tx_symbol_id = txSymbols[symbol_id];
            offset = generateOffset3d(TASK_BUFFER_FRAME_NUM, txSymsPerFrame, cfg->getNumAntennas(), frame_id, symbol_id, ant_id);
            void* txbuf[2];
            long long frameTime = ((long long)tx_frame_id << 32) | (tx_symbol_id << 16);
            int flags = 1; // HAS_TIME
            if (tx_symbol_id == txSymbols.back()) flags = 2; // HAS_TIME & END_BURST, fixme
            txbuf[0] = buffer + offset * packet_length; //   obj_ptr->pilot_buffer_; //
            buffer_status[offset] = 0;
#if DEBUG_SEND
        printf("RU: transmit tx_frame_id %d, tx_symbol_id %d, cell_id %d, ant_id %d\n", frame_id, symbol_id, cell_id, ant_id);
        printf("transmit samples: %f %f %f %f %f %f %f %f ...\n",*((RadioBufRealType *)txbuf[0]+2*cfg->prefix+9), 
                           *((RadioBufRealType *)txbuf[0]+2*cfg->prefix+10),
                           *((RadioBufRealType *)txbuf[0]+2*cfg->prefix+11),
                           *((RadioBufRealType *)txbuf[0]+2*cfg->prefix+12),
                           *((RadioBufRealType *)txbuf[0]+2*cfg->prefix+13),
                           *((RadioBufRealType *)txbuf[0]+2*cfg->prefix+14),
                           *((RadioBufRealType *)txbuf[0]+2*cfg->prefix+15),
                           *((RadioBufRealType *)txbuf[0]+2*cfg->prefix+16));
#endif 
            if (cfg->nChannels == 2)
            {
                txbuf[1] = buffer + (offset + 1) * packet_length;
                buffer_status[offset+1] = 0;
            }
            clock_gettime(CLOCK_MONOTONIC, &tv);
            radio->radioTx(ant_id/cfg->nChannels, txbuf, flags, frameTime);
            clock_gettime(CLOCK_MONOTONIC, &tv2);
#if TX_TIME_MEASURE
            double diff = (tv2.tv_sec * 1e9 + tv2.tv_nsec - tv.tv_sec * 1e9 - tv.tv_nsec);
            time_avg += diff;
            time_count++;
            if (time_count == cfg->getNumAntennas()*cfg->dlSymsPerFrame)
            {
                printf("In TX thread %d: Transmitted 100 frames with average tx time %f\n", tid, time_avg/time_count/1e3);
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
        if ( cfg->running && !message_queue_->enqueue(local_ptok, packet_message ) ) {
            printf("socket message enqueue failed\n");
            exit(0);
        }
    }
    return 0; 
}



/*****  Receive threads   *****/

void* RU::loopProc(void *in_context)
{
    // get the pointer of class & tid
    RU* obj_ptr = ((RUContext *)in_context)->ptr;
    int tid = ((RUContext *)in_context)->tid;
    Config *cfg = obj_ptr->config_;
    // get pointer of message queue
    moodycamel::ConcurrentQueue<Event_data> *message_queue_ = obj_ptr->message_queue_;
#ifdef SIM
    moodycamel::ConcurrentQueue<Event_data> *task_queue_ = obj_ptr->task_queue_;
#else
    int nradio_cur_thread = ((RUContext *)in_context)->radios;
    printf("receiver thread %d has %d radios\n", tid, nradio_cur_thread);
#endif
    int core_id = obj_ptr->core_id_;
    // if ENABLE_CPU_ATTACH is enabled, attach threads to specific cores
#ifdef ENABLE_CPU_ATTACH
    // printf("Recv thread: pinning thread %d to core %d\n", tid, core_id + tid);
    if(pin_to_core(core_id + tid) != 0)
    {
        printf("Recv thread: pinning thread %d to core %d failed\n", tid, core_id + tid);
        exit(0);
    }
    else {
        printf("Recv thread: pinning thread %d to core %d succeed\n", tid, core_id + tid);
    }
#endif

    // Use mutex to sychronize data receiving across threads
    pthread_mutex_lock(&obj_ptr->mutex);
    printf("Thread %d: waiting for release\n", tid);

    pthread_cond_wait(&obj_ptr->cond, &obj_ptr->mutex);
    pthread_mutex_unlock(&obj_ptr->mutex); // unlocking for all other threads

    // usleep(10000-tid*2000);
    // use token to speed up
    moodycamel::ProducerToken local_ptok(*message_queue_);
    //moodycamel::ProducerToken local_ctok(*task_queue_);
    //moodycamel::ProducerToken *local_ctok = (obj_ptr->task_ptok[tid]);

    int packet_length = cfg->packet_length;
    int packet_header_offset = cfg->packet_header_offset;
    int tx_packet_length = packet_length - packet_header_offset;
    char* buffer = (char*)obj_ptr->buffer_[tid];
    int* buffer_status = obj_ptr->buffer_status_[tid];
    int buffer_length = obj_ptr->buffer_length_;
    int buffer_frame_num = obj_ptr->buffer_frame_num_;

    char *tx_buffer = obj_ptr->tx_buffer_;

    int txSymsPerFrame = 0;
    std::vector<size_t> txSymbols;
    if (cfg->isUE)
    {
        txSymsPerFrame = cfg->ulSymsPerFrame;
        txSymbols = cfg->ULSymbols[0];
    }
    else
    {
        txSymsPerFrame = cfg->dlSymsPerFrame;
        txSymbols = cfg->DLSymbols[0];
    }
    int n_ant = cfg->getNumAntennas();

    char* cur_ptr_buffer = buffer;
    int* cur_ptr_buffer_status = buffer_status;
#ifndef SIM
    int nradio_per_thread = cfg->nRadios/obj_ptr->thread_num_;
    int rem_thread_nradio = cfg->nRadios%obj_ptr->thread_num_;//obj_ptr->thread_num_*(cfg->nRadios/obj_ptr->thread_num_);
    printf("receiver thread %d has %d radios\n", tid, nradio_cur_thread);
    RadioConfig *radio = obj_ptr->radioconfig_;

    // to handle second channel at each radio
    // this is assuming buffer_frame_num is at least 2 
    char* cur_ptr_buffer2;
    int* cur_ptr_buffer_status2;
    char* buffer2 = (char*)obj_ptr->buffer_[tid] + packet_length;
    int* buffer_status2 = obj_ptr->buffer_status_[tid] + 1;
    cur_ptr_buffer_status2 = buffer_status2;
    if (cfg->nChannels == 2) {
        cur_ptr_buffer2 = buffer2;
    }
    else {
        cur_ptr_buffer2 = (char*)calloc(packet_length, sizeof(char)); 
    }
#else
    // loop recv
    socklen_t addrlen = sizeof(obj_ptr->servaddr_[tid]);
#endif
    int offset = 0;
    int packet_num = 0;
    long long frameTime;

    int maxQueueLength = 0;
    while(cfg->running)
    {
        // if buffer is full, exit
        if(cur_ptr_buffer_status[0] == 1) {
            printf("RX thread %d buffer full\n", tid);
            //exit(0);
            cfg->running = false;
            break;
        }
        // receive data
#ifdef SIM
        int recvlen = -1;
        if ((recvlen = recvfrom(obj_ptr->rx_socket_[tid], cur_ptr_buffer, (size_t) packet_length, 0, (struct sockaddr *) &obj_ptr->servaddr_[tid], &addrlen)) < 0) {
            perror("recv failed");
            exit(0);
        }
        // read information from received packet
    struct Packet *pkt = (struct Packet *)cur_ptr_buffer;

    int frame_id = pkt->frame_id;
    int symbol_id = pkt->symbol_id;
    //int cell_id = pkt->cell_id;
    int ant_id = pkt->ant_id;

#if DEBUG_RECV
        printf("RU: receive frame_id %d, symbol_id %d, cell_id %d, ant_id %d\n", frame_id, symbol_id, cell_id, ant_id);
        //printf("receive samples: %f %f %f %f %f %f %f %f ...\n",*((RadioBufRealType *)cur_ptr_buffer+9), 
        //                   *((RadioBufRealType *)cur_ptr_buffer+10),
        //                   *((RadioBufRealType *)cur_ptr_buffer+11),
        //                   *((RadioBufRealType *)cur_ptr_buffer+12),
        //                   *((RadioBufRealType *)cur_ptr_buffer+13),
        //                   *((RadioBufRealType *)cur_ptr_buffer+14),
        //                   *((RadioBufRealType *)cur_ptr_buffer+15),
        //                   *((RadioBufRealType *)cur_ptr_buffer+16));
#endif 
        // get the position in buffer
        offset = cur_ptr_buffer_status - buffer_status;
        // move ptr & set status to full
        cur_ptr_buffer_status[0] = 1; // has data, after doing fft, it is set to 0
        cur_ptr_buffer_status = buffer_status + (cur_ptr_buffer_status - buffer_status + 1) % buffer_frame_num;
        cur_ptr_buffer = buffer + (cur_ptr_buffer - buffer + packet_length) % buffer_length;
        // push EVENT_RX_ENB event into the queue
        Event_data packet_message;
        packet_message.event_type = EVENT_RX_SYMBOL;
        // data records the position of this packet in the buffer & tid of this socket (so that task thread could know which buffer it should visit) 
        packet_message.data = offset + tid * buffer_frame_num;
        if ( !message_queue_->enqueue(local_ptok, packet_message ) ) {
            printf("socket message enqueue failed\n");
            exit(0);
        }

        if (txSymsPerFrame > 0 and ((cfg->isUE and cfg->getDlSFIndex(frame_id, symbol_id) == 0) || (!cfg->isUE and cfg->getPilotSFIndex(frame_id, symbol_id) == 0)))
        {
            // notify TXthread to start transmitting frame_id+offset
            Event_data do_tx_task;
            do_tx_task.event_type = TASK_SEND;
            do_tx_task.data = ant_id;
            do_tx_task.more_data = cfg->isUE ? frame_id + TX_FRAME_DELTA : frame_id;
            if ( !task_queue_->enqueue(*obj_ptr->task_ptok[tid], do_tx_task)) {
                printf("task enqueue failed\n");
                exit(0);
            }
        }

        //printf("enqueue offset %d\n", offset);
        int cur_queue_len = message_queue_->size_approx();
        maxQueueLength = maxQueueLength > cur_queue_len ? maxQueueLength : cur_queue_len;

        packet_num++;
#else
        for (int it = 0 ; it < nradio_cur_thread; it++) // FIXME: this must be threaded
        {
            //int rid = tid * obj_ptr->radios_per_thread + it;
            int rid = (tid < rem_thread_nradio) ? tid * (nradio_per_thread + 1) + it : tid * (nradio_per_thread) + rem_thread_nradio + it ;
            // this is probably a really bad implementation, and needs to be revamped
            char* samp1 = (cur_ptr_buffer +  packet_header_offset*sizeof(int));
            char* samp2 = (cur_ptr_buffer2 + packet_header_offset*sizeof(int));
            void *samp[2] = {(void*)samp1, (void*)samp2};
            while (cfg->running && radio->radioRx(rid, samp, frameTime) <= 0);
            frame_id = (int)(frameTime>>32);
            symbol_id = (int)((frameTime>>16)&0xFFFF);
            ant_id = rid * cfg->nChannels;
	    struct Packet *pkt = (struct Packet *)cur_ptr_buffer;
	    new (pkt) Packet(frame_id, symbol_id, 0 /* cell_id */, ant_id);
            if (cfg->nChannels == 2)
            {
	        struct Packet *pkt2 = (struct Packet *)cur_ptr_buffer2;
	        new (pkt2) Packet(frame_id, symbol_id, 0 /* cell_id */, ant_id + 1);
            }
#if DEBUG_RECV
            printf("receive thread %d: frame_id %d, symbol_id %d, cell_id %d, ant_id %d frametime %llx\n", tid, frame_id, symbol_id, cell_id, ant_id, frameTime);
//            printf("receive samples: %d %d %d %d %d %d %d %d ...\n",*((RadioBufElemeType *)cur_ptr_buffer+9), 
//                               *((RadioBufElemeType *)cur_ptr_buffer+10),
//                                                           *((RadioBufElemeType *)cur_ptr_buffer+11),
//                                                           *((RadioBufElemeType *)cur_ptr_buffer+12),
//                                                           *((RadioBufElemeType *)cur_ptr_buffer+13),
//                                                           *((RadioBufElemeType *)cur_ptr_buffer+14),
//                                                           *((RadioBufElemeType *)cur_ptr_buffer+15),
//                                                           *((RadioBufElemeType *)cur_ptr_buffer+16)); 
#endif             
            // get the position in buffer
            offset = cur_ptr_buffer_status - buffer_status;
            // move ptr & set status to full
            cur_ptr_buffer_status[0] = 1; // has data, after it is read it should be set to 0
            cur_ptr_buffer_status = buffer_status + (cur_ptr_buffer_status - buffer_status + cfg->nChannels) % buffer_frame_num;
            cur_ptr_buffer = buffer + (cur_ptr_buffer - buffer + packet_length * cfg->nChannels) % buffer_length;
            // push EVENT_RX_ENB event into the queue
            Event_data packet_message;
            packet_message.event_type = EVENT_PACKET_RECEIVED;
            // data records the position of this packet in the buffer & tid of this socket (so that task thread could know which buffer it should visit) 
            packet_message.data = offset + tid * buffer_frame_num; // Note: offset < buffer_frame_num 
            if (!message_queue_->enqueue(local_ptok, packet_message ) ) {
                printf("socket message enqueue failed\n");
                exit(0);
            }
            if (cfg->nChannels == 2)
            {
                offset = cur_ptr_buffer_status2 - buffer_status; // offset is absolute 
                cur_ptr_buffer_status2[0] = 1; // has data, after doing fft, it is set to 0
                cur_ptr_buffer_status2 = buffer_status2 + (cur_ptr_buffer_status2 - buffer_status2 + cfg->nChannels) % buffer_frame_num;
                cur_ptr_buffer2 = buffer2 + (cur_ptr_buffer2 - buffer2 + packet_length * cfg->nChannels) % buffer_length;
                // push EVENT_RX_ENB event into the queue
                Event_data packet_message2;
                packet_message2.event_type = EVENT_PACKET_RECEIVED;
                // data records the position of this packet in the buffer & tid of this socket (so that task thread could know which buffer it should visit) 
                packet_message2.data = offset + tid * buffer_frame_num;
                if (!message_queue_->enqueue(local_ptok, packet_message2 ) ) {
                    printf("socket message enqueue failed\n");
                    exit(0);
                }
            }

            // notify TXthread to start transmitting frame_id+offset
            if (txSymsPerFrame > 0 && ((cfg->isUE && cfg->getDlSFIndex(frame_id, symbol_id) == 0) || (!cfg->isUE && cfg->getPilotSFIndex(frame_id, symbol_id) == 0)))
            {
//#ifdef SEPARATE_TX_THREAD
//                Event_data do_tx_task;
//                do_tx_task.event_type = TASK_packet_SENT;
//                do_tx_task.data = ant_id; //tx_symbol_id * cfg->getNumAntennas() + ant_id;
//                do_tx_task.more_data = frame_id + TX_FRAME_DELTA;
//                if ( !task_queue_->enqueue(*obj_ptr->task_ptok[tid], do_tx_task)) {
//                    printf("task enqueue failed\n");
//                    exit(0);
//                }
//#else
                for (int tx_symbol_id = 0; tx_symbol_id < txSymsPerFrame; tx_symbol_id++)
                {
                    int tx_frame_id = frame_id + TX_FRAME_DELTA;
                    int tx_frame_offset = tx_frame_id % TASK_BUFFER_FRAME_NUM; 
                    size_t tx_symbol = txSymbols[tx_symbol_id];
                    //int tx_offset = generateOffset3d(TASK_BUFFER_FRAME_NUM, txSymsPerFrame, cfg->getNumAntennas(), tx_frame_id, tx_symbol_id, ant_id);
                    int frame_samp_size = (tx_packet_length * n_ant * txSymsPerFrame);
                    int tx_offset = tx_frame_offset * frame_samp_size + tx_packet_length * (n_ant * tx_symbol_id + ant_id);
                    void* txbuf[2];
                    long long frameTime = ((long long)tx_frame_id << 32) | (tx_symbol << 16);
                    int flags = 1; // HAS_TIME
                    if (tx_symbol == txSymbols.back()) flags = 2; // HAS_TIME & END_BURST
                    txbuf[0] = (void*)(tx_buffer + tx_offset); 
                    if (cfg->nChannels == 2)
                    {
                        txbuf[1] = (void*)(tx_buffer + tx_offset + tx_packet_length);
                    }
#if DEBUG_SEND
                    int start_ind = 2 * cfg->prefix;
                    printf("transmit samples: %d %d %d %d %d %d %d %d ...\n\n",
                                              *((short *)txbuf[0]+start_ind+0),
                                              *((short *)txbuf[0]+start_ind+1),
                                              *((short *)txbuf[0]+start_ind+2),
                                              *((short *)txbuf[0]+start_ind+3),
                                              *((short *)txbuf[0]+start_ind+4),
                                              *((short *)txbuf[0]+start_ind+5),
                                              *((short *)txbuf[0]+start_ind+6), 
                                              *((short *)txbuf[0]+start_ind+7)); 
#endif
                    radio->radioTx(rid, txbuf, flags, frameTime);
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
    return 0;
}

