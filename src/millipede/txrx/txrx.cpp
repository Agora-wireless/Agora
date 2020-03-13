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

    if (config_->downlink_mode)
        return created_threads;

    for (int i = 0; i < rx_thread_num_; i++) {
        pthread_t recv_thread_;
        // record the thread id
        auto context = new EventHandlerContext<PacketTXRX>;
        context->obj_ptr = this;
        context->id = i;
        // start socket thread
        if (pthread_create(&recv_thread_, NULL,
                pthread_fun_wrapper<PacketTXRX, &PacketTXRX::loopRecv>, context)
            != 0) {
            perror("socket recv thread create failed");
            exit(0);
        }
        created_threads.push_back(recv_thread_);
    }
    return created_threads;
}

std::vector<pthread_t> PacketTXRX::startTX(char* in_buffer,
    int* in_buffer_status, int in_buffer_frame_num, int in_buffer_length)
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
    std::vector<pthread_t> created_threads;

    for (int i = 0; i < tx_thread_num_; i++) {
        pthread_t send_thread_;
        auto context = new EventHandlerContext<PacketTXRX>;
        context->obj_ptr = this;
        context->id = i;
        if (pthread_create(&send_thread_, NULL,
                pthread_fun_wrapper<PacketTXRX, &PacketTXRX::loopTXRX>, context)
            != 0) {
            perror("socket Transmit thread create failed");
            exit(0);
        }

        created_threads.push_back(send_thread_);
    }

    return created_threads;
}

void* PacketTXRX::loopTXRX(int tid)
{
    pin_to_core_with_offset(Worker_TXRX, core_id_, tid);
    int BS_ANT_NUM = config_->BS_ANT_NUM;
    int pilot_subframe_num_perframe = config_->pilot_symbol_num_perframe;
    int ul_data_subframe_num_perframe = config_->ul_data_symbol_num_perframe;
    int dl_data_subframe_num_perframe = config_->dl_data_symbol_num_perframe;
    int downlink_mode = config_->downlink_mode;
    int sock_buf_size = 1024 * 1024 * 64 * 8 - 1;
    int local_port_id = config_->bs_port + tid;
    int remote_port_id = config_->ue_rx_port + tid;
#if USE_IPV4
    int socket_local = setup_socket_ipv4(local_port_id, true, sock_buf_size);
    struct sockaddr_in remote_addr;
    setup_sockaddr_remote_ipv4(
        &remote_addr, remote_port_id, config_->tx_addr.c_str());
    // struct sockaddr_in local_addr;
    // setup_sockaddr_local_ipv4(&local_addr, local_port_id);
#else
    int socket_local = setup_socket_ipv6(local_port_id, true, sock_buf_size);
    struct sockaddr_in6 remote_addr;
    setup_sockaddr_remote_ipv6(
        &remote_addr, remote_port_id, config_->tx_addr.c_str());
    // struct sockaddr_in6 local_addr;
    // setup_sockaddr_local_ipv6(&local_addr, local_port_id);
#endif

    // RX  pointers
    int rx_buffer_frame_num = buffer_frame_num_;
    double* rx_frame_start = (*frame_start_)[tid];
    int rx_offset = 0;
    int frame_id;

#if 0
    // walk through all the pages
    double temp;
    for (int i = 0; i < 20; i++) {
        temp = rx_frame_start[i * 512];
    }
#endif

    // TX pointers
    // float *tx_data_buffer = tx_data_buffer_;
    // buffer_frame_num: subframe_num_perframe * BS_ANT_NUM *
    // SOCKET_BUFFER_FRAME_NUM float *tx_cur_ptr_data;

    int max_subframe_id
        = downlink_mode
        ? pilot_subframe_num_perframe
        : (pilot_subframe_num_perframe + ul_data_subframe_num_perframe);
    int max_rx_packet_num_per_frame
        = max_subframe_id * BS_ANT_NUM / rx_thread_num_;
    int max_tx_packet_num_per_frame
        = dl_data_subframe_num_perframe * BS_ANT_NUM / tx_thread_num_;
    printf("Maximum RX pkts: %d, TX pkts: %d\n", max_rx_packet_num_per_frame,
        max_tx_packet_num_per_frame);
    int prev_frame_id = -1;
    bool do_tx = false;
    enum { NUM_COUNTERS = 10000 };
    int rx_pkts_in_frame_count[NUM_COUNTERS] = { 0 };
    // int last_finished_frame_id = 0;

    int tx_pkts_in_frame_count[NUM_COUNTERS] = { 0 };
    // int last_finished_tx_frame_id = 0;

    // double start_time= get_time();

    while (true) {
        if (!do_tx) {
            struct Packet* pkt = recv_enqueue(tid, socket_local, rx_offset);
            frame_id = pkt->frame_id;

#if MEASURE_TIME
#if DEBUG_RECV
            int symbol_id = pkt->symbol_id;
            int ant_id = pkt->ant_id;
            printf("RX thread %d received frame %d subframe %d, ant %d "
                   "offset %d\n",
                tid, frame_id, symbol_id, ant_id, rx_offset);
#endif
            if (frame_id > prev_frame_id) {
                *(rx_frame_start + frame_id) = get_time();
                prev_frame_id = frame_id;
                if (frame_id % 512 == 200) {
                    _mm_prefetch(
                        (char*)(rx_frame_start + frame_id + 512), _MM_HINT_T0);
                }
            }
#endif

            frame_id %= NUM_COUNTERS;
            if (downlink_mode
                && ++rx_pkts_in_frame_count[frame_id]
                    == max_rx_packet_num_per_frame) {
                do_tx = true;
                rx_pkts_in_frame_count[frame_id] = 0;
                // printf("In TXRX thread %d: RX finished frame %d, current
                // frame %d\n", tid, last_finished_frame_id, prev_frame_id);
                // last_finished_frame_id++;
            }
        } else {
            int offset = dequeue_send(tid, socket_local, &remote_addr);
            if (offset == -1)
                continue;
            int frame_id_in_buffer = offset / BS_ANT_NUM
                / config_->data_symbol_num_perframe % SOCKET_BUFFER_FRAME_NUM;
            assert(SOCKET_BUFFER_FRAME_NUM < NUM_COUNTERS);
            tx_pkts_in_frame_count[frame_id_in_buffer]++;

            if (tx_pkts_in_frame_count[frame_id_in_buffer]
                == max_tx_packet_num_per_frame) {
                do_tx = false;
                tx_pkts_in_frame_count[frame_id_in_buffer] = 0;
                // printf("In TXRX thread %d: TX finished frame %d, current
                // frame %d\n", tid, last_finished_tx_frame_id,
                // prev_frame_id); prev_frame_id = tx_frame_id;
                // last_finished_tx_frame_id = (last_finished_tx_frame_id +
                // 1) % TASK_BUFFER_FRAME_NUM;
            }
        }
        rx_offset++;
        if (rx_offset == rx_buffer_frame_num)
            rx_offset = 0;
    }
    return 0;
}

struct Packet* PacketTXRX::recv_enqueue(
    int tid, int socket_local, int rx_offset)
{
    moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];
    char* rx_buffer = (*buffer_)[tid];
    int* rx_buffer_status = (*buffer_status_)[tid];
    int packet_length = config_->packet_length;

    // if rx_buffer is full, exit
    if (rx_buffer_status[rx_offset] == 1) {
        printf(
            "Receive thread %d rx_buffer full, offset: %d\n", tid, rx_offset);
        exit(0);
    }
    struct Packet* pkt = (struct Packet*)&rx_buffer[rx_offset * packet_length];
    int recvlen = recv(socket_local, (char*)pkt, packet_length, 0);
    if (recvlen < 0) {
        perror("recv failed");
        exit(0);
    }

    // get the position in rx_buffer
    // move ptr & set status to full
    rx_buffer_status[rx_offset]
        = 1; // has data, after doing fft, it is set to 0
    // push EVENT_PACKET_RECEIVED event into the queue
    Event_data rx_message;
    rx_message.event_type = EVENT_PACKET_RECEIVED;
    // data records the position of this packet in the rx_buffer & tid of this
    // socket (so that task thread could know which rx_buffer it should visit)
    rx_message.data = generateOffset2d_setbits(tid, rx_offset, 28);
    // rx_message.data = rx_offset + tid * rx_buffer_frame_num;
    // if ( !message_queue_->enqueue(rx_message ) ) {
    if (!message_queue_->enqueue(*local_ptok, rx_message)) {
        printf("socket message enqueue failed\n");
        exit(0);
    }
    return pkt;
}

void* PacketTXRX::loopRecv(int tid)
{
    pin_to_core_with_offset(Worker_RX, core_id_, tid);
    int sock_buf_size = 1024 * 1024 * 64 * 8 - 1;
    int local_port_id = config_->bs_port + tid;
#if USE_IPV4
    int socket_local = setup_socket_ipv4(local_port_id, true, sock_buf_size);
    // struct sockaddr_in local_addr;
    // setup_sockaddr_local_ipv4(&local_addr, local_port_id);
#else
    int socket_local = setup_socket_ipv6(local_port_id, true, sock_buf_size);
    // struct sockaddr_in6 local_addr;
    // setup_sockaddr_local_ipv6(&local_addr, local_port_id);
#endif

    // use token to speed up
    // moodycamel::ProducerToken local_ptok(*message_queue_);
    // moodycamel::ProducerToken *local_ptok = new
    // moodycamel::ProducerToken(*message_queue_);
    int rx_buffer_frame_num = buffer_frame_num_;
    double* rx_frame_start = (*frame_start_)[tid];

    // walk through all the pages
#if 0
    double temp;
    for (int i = 0; i < 20; i++) {
        temp = rx_frame_start[i * 512];
    }
#endif

    // loop recv
    // socklen_t addrlen = sizeof(obj_ptr->servaddr_[tid]);
    int rx_offset = 0;
    int prev_frame_id = -1;
    // double start_time= get_time();

    // printf("Rx thread %d: on core %d\n", tid, sched_getcpu());

    while (true) {
        struct Packet* pkt = recv_enqueue(tid, socket_local, rx_offset);
#if MEASURE_TIME
        // read information from received packet
        int frame_id = pkt->frame_id;
#if DEBUG_RECV
        int symbol_id = pkt->symbol_id;
        int ant_id = pkt->ant_id;
        printf("RX thread %d received frame %d subframe %d, ant %d\n", tid,
            frame_id, symbol_id, ant_id);
#endif
        if (frame_id > prev_frame_id) {
            *(rx_frame_start + frame_id) = get_time();
            prev_frame_id = frame_id;
            if (frame_id % 512 == 200) {
                _mm_prefetch(
                    (char*)(rx_frame_start + frame_id + 512), _MM_HINT_T0);
                // double temp = rx_frame_start[frame_id+3];
            }
        }
#endif
        rx_offset++;
        if (rx_offset == rx_buffer_frame_num)
            rx_offset = 0;
    }
    return 0;
}

int PacketTXRX::dequeue_send(int tid, int socket_local, sockaddr_t* remote_addr)
{
    Event_data task_event;
    if (!task_queue_->try_dequeue_from_producer(*tx_ptoks_[tid], task_event))
        return -1;

    // printf("tx queue length: %d\n", task_queue_->size_approx());
    if (task_event.event_type != TASK_SEND) {
        printf("Wrong event type!");
        exit(0);
    }

    int BS_ANT_NUM = config_->BS_ANT_NUM;
    int pilot_subframe_num_perframe = config_->pilot_symbol_num_perframe;
    int data_subframe_num_perframe = config_->data_symbol_num_perframe;
    int packet_length = config_->packet_length;
    int offset = task_event.data;
    int ant_id = offset % BS_ANT_NUM;
    int total_data_subframe_id = offset / BS_ANT_NUM;
    int frame_id = total_data_subframe_id / data_subframe_num_perframe;
    int current_data_subframe_id
        = total_data_subframe_id % data_subframe_num_perframe;
    int symbol_id = current_data_subframe_id + pilot_subframe_num_perframe;

#if DEBUG_BS_SENDER
    printf("In TX thread %d: Transmitted frame %d, subframe %d, "
           "ant %d, offset: %d, msg_queue_length: %zu\n",
        tid, frame_id, symbol_id, ant_id, offset,
        message_queue_->size_approx());
#endif

    char* cur_buffer_ptr = tx_buffer_
        + (current_data_subframe_id * BS_ANT_NUM + ant_id) * packet_length;
    // cur_ptr_data = (dl_data_buffer + 2 * data_subframe_offset * OFDM_CA_NUM *
    // BS_ANT_NUM);
    struct Packet* pkt = (struct Packet*)cur_buffer_ptr;
    new (pkt) Packet(frame_id, symbol_id, 0 /* cell_id */, ant_id);

    // send data (one OFDM symbol)
    if (sendto(socket_local, (char*)cur_buffer_ptr, packet_length, 0,
            (struct sockaddr*)remote_addr, sizeof(*remote_addr))
        < 0) {
        perror("socket sendto failed");
        exit(0);
    }
    Event_data tx_message;
    tx_message.event_type = EVENT_PACKET_SENT;
    tx_message.data = offset;
    moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];
    if (!message_queue_->enqueue(*local_ptok, tx_message)) {
        printf("socket message enqueue failed\n");
        exit(0);
    }
    return offset;
}
