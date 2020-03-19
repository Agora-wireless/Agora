/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */

#include "receiver.hpp"

Receiver::Receiver(Config* cfg, int RX_THREAD_NUM, int in_core_offset)
{
    config_ = cfg;
    rx_thread_num_ = RX_THREAD_NUM;

    core_id_ = in_core_offset;
    packet_length = cfg->packet_length;
}

Receiver::Receiver(Config* cfg, int RX_THREAD_NUM, int in_core_offset,
    moodycamel::ConcurrentQueue<Event_data>* in_queue_message,
    moodycamel::ProducerToken** in_rx_ptoks)
    : Receiver(cfg, RX_THREAD_NUM, in_core_offset)
{
    message_queue_ = in_queue_message;
    rx_ptoks_ = in_rx_ptoks;
}

Receiver::~Receiver() { delete config_; }

std::vector<pthread_t> Receiver::startRecv(Table<char>& in_buffer,
    Table<int>& in_buffer_status, int in_buffer_frame_num,
    long long in_buffer_length, Table<double>& in_frame_start)
{
    buffer_frame_num_ = in_buffer_frame_num;
    buffer_length_ = in_buffer_length;
    buffer_ = &in_buffer;
    buffer_status_ = &in_buffer_status;
    frame_start_ = &in_frame_start;

    printf("start Recv thread\n");
    std::vector<pthread_t> created_threads;

#if USE_DPDK
    unsigned int nb_lcores = rte_lcore_count();
    printf("Number of DPDK cores: %d\n", nb_lcores);
    unsigned int lcore_id;
    int worker_id = 0;
    // Launch specific task to cores
    RTE_LCORE_FOREACH_SLAVE(lcore_id)
    {
        // launch communication and task thread onto specific core
        if (worker_id < rx_thread_num_) {
            auto context = new EventHandlerContext<Receiver>;
            context->obj_ptr = this;
            context->id = worker_id;
            rte_eal_remote_launch(
                (lcore_function_t*)loopRecv_DPDK, context, lcore_id);
            printf("RX: launched thread %d on core %d\n", worker_id, lcore_id);
        }
        worker_id++;
    }
#else
    for (int i = 0; i < rx_thread_num_; i++) {
        pthread_t recv_thread_;
        auto context = new EventHandlerContext<Receiver>;
        context->obj_ptr = this;
        context->id = i;
        if (pthread_create(&recv_thread_, NULL,
                pthread_fun_wrapper<Receiver, &Receiver::loopRecv>, context)
            != 0) {
            perror("socket recv thread create failed");
            exit(0);
        }
        created_threads.push_back(recv_thread_);
    }
#endif
    return created_threads;
}

void* Receiver::loopRecv(int tid)
{
    int core_offset = core_id_ + rx_thread_num_ + 2;
    pin_to_core_with_offset(ThreadType::kWorkerRX, core_offset, tid);

    int sock_buf_size = 1024 * 1024 * 64 * 8 - 1;
#if USE_IPV4
    struct sockaddr_in remote_addr;
    int socket_local
        = setup_socket_ipv4(config_->ue_rx_port + tid, true, sock_buf_size);
    setup_sockaddr_remote_ipv4(
        &remote_addr, config_->bs_port + tid, config_->rx_addr.c_str());
#else
    int socket_local
        = setup_socket_ipv6(config_->ue_rx_port + tid, true, sock_buf_size);
    setup_sockaddr_remote_ipv6(
        &remote_addr, config_->bs_port + tid, "fe80::f436:d735:b04a:864a");
#endif

    /* use token to speed up */
    // moodycamel::ProducerToken local_ptok(*message_queue_);
    // moodycamel::ProducerToken *local_ptok = new
    // moodycamel::ProducerToken(*message_queue_);
    moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];

    char* buffer_ptr = (*buffer_)[tid];
    int* buffer_status_ptr = (*buffer_status_)[tid];
    long long buffer_length = buffer_length_;
    int buffer_frame_num = buffer_frame_num_;
    double* frame_start = (*frame_start_)[tid];

    // // walk through all the pages
    // double temp;
    // for (int i = 0; i < 20; i++) {
    //     temp = frame_start[i * 512];
    // }

    char* cur_buffer_ptr = buffer_ptr;
    int* cur_buffer_status_ptr = buffer_status_ptr;
    // loop recv
    socklen_t addrlen = sizeof(remote_addr);
    int offset = 0;
    int prev_frame_id = -1;
    while (true) {
        /* if buffer is full, exit */
        if (cur_buffer_status_ptr[0] == 1) {
            printf("Receive thread %d buffer full, offset: %d\n", tid, offset);
            exit(0);
        }
        int recvlen = -1;
        // if ((recvlen = recv(socket_local, (char*)cur_buffer_ptr,
        // packet_length, 0))<0) {
        if ((recvlen = recvfrom(socket_local, (char*)cur_buffer_ptr,
                 packet_length, 0, (struct sockaddr*)&remote_addr, &addrlen))
            < 0) {
            perror("recv failed");
            exit(0);
        }

#if MEASURE_TIME
        // read information from received packet
        struct Packet* pkt = (struct Packet*)cur_buffer_ptr;
        int frame_id = pkt->frame_id;
#if DEBUG_SENDER
        printf("RX thread %d received frame %d symbol %d, ant %d\n ", tid,
            frame_id, pkt->symbol_id, pkt->ant_id);
#endif
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
        /* get the position in buffer */
        offset = cur_buffer_status_ptr - buffer_status_ptr;
        cur_buffer_status_ptr[0] = 1;
        cur_buffer_status_ptr
            = buffer_status_ptr + (offset + 1) % buffer_frame_num;
        cur_buffer_ptr = buffer_ptr
            + (cur_buffer_ptr - buffer_ptr + packet_length) % buffer_length;

        /* Push packet received event into the queue. data records the position
         * of this packet in the buffer & tid of this socket (so that task
         * thread could know which buffer it should visit) */
        Event_data packet_message(
            EventType::kPacketRX, generateOffset2d_setbits(tid, offset, 28));

        if (!message_queue_->enqueue(*local_ptok, packet_message)) {
            printf("socket message enqueue failed\n");
            exit(0);
        }
    }
}
