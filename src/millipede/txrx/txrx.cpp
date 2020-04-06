/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */

#include "txrx.hpp"

PacketTXRX::PacketTXRX(Config* cfg, int COMM_THREAD_NUM, int in_core_offset)
{
    config_ = cfg;
    comm_thread_num_ = COMM_THREAD_NUM;

    core_id_ = in_core_offset;
    tx_core_id_ = in_core_offset + COMM_THREAD_NUM;

    /* initialize random seed: */
    srand(time(NULL));

    socket_ = new int[config_->nRadios];
#if USE_IPV4
    servaddr_ = new struct sockaddr_in[config_->nRadios];
#else
    servaddr_ = new struct sockaddr_in6[config_->nRadios];
#endif
}

PacketTXRX::PacketTXRX(Config* cfg, int COMM_THREAD_NUM, int in_core_offset,
    moodycamel::ConcurrentQueue<Event_data>* in_queue_message,
    moodycamel::ConcurrentQueue<Event_data>* in_queue_task,
    moodycamel::ProducerToken** in_rx_ptoks,
    moodycamel::ProducerToken** in_tx_ptoks)
    : PacketTXRX(cfg, COMM_THREAD_NUM, in_core_offset)
{
    message_queue_ = in_queue_message;
    task_queue_ = in_queue_task;
    rx_ptoks_ = in_rx_ptoks;
    tx_ptoks_ = in_tx_ptoks;
}

PacketTXRX::~PacketTXRX()
{
    delete[] socket_;
    delete[] servaddr_;
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

    printf("create TXRX threads\n");
    for (int i = 0; i < comm_thread_num_; i++) {
        pthread_t txrx_thread;
        auto context = new EventHandlerContext<PacketTXRX>;
        context->obj_ptr = this;
        context->id = i;
        if (pthread_create(&txrx_thread, NULL,
                pthread_fun_wrapper<PacketTXRX, &PacketTXRX::loopTXRX>, context)
            != 0) {
            perror("socket communication thread create failed");
            exit(0);
        }
    }
    return true;
}

void* PacketTXRX::loopTXRX(int tid)
{
    pin_to_core_with_offset(ThreadType::kWorkerTXRX, core_id_, tid);
    double* rx_frame_start = (*frame_start_)[tid];
    int rx_offset = 0;
    int radio_lo = tid * config_->nRadios / comm_thread_num_;
    int radio_hi = (tid + 1) * config_->nRadios / comm_thread_num_;
    printf("receiver thread %d has %d radios\n", tid, radio_hi - radio_lo);

    int sock_buf_size = 1024 * 1024 * 64 * 8 - 1;
    for (int radio_id = radio_lo; radio_id < radio_hi; ++radio_id) {
        int local_port_id = config_->bs_port + radio_id;
#if USE_IPV4
        socket_[radio_id]
            = setup_socket_ipv4(local_port_id, true, sock_buf_size);
        setup_sockaddr_remote_ipv4(&servaddr_[radio_id],
            config_->ue_rx_port + radio_id, config_->tx_addr.c_str());
#else
        socket_[radio_id]
            = setup_socket_ipv6(local_port_id, true, sock_buf_size);
        setup_sockaddr_remote_ipv6(&servaddr_[radio_id],
            config_->ue_rx_port + radio_id, config_->tx_addr.c_str());
#endif
        // fcntl(socket_[radio_id], F_SETFL, O_NONBLOCK);
    }

#if MEASURE_TIME
    int prev_frame_id = -1;
#endif
    int radio_id = radio_lo;
    while (config_->running) {
        // if (-1 != dequeue_send(tid))
        //     continue;
        // receive data
        struct Packet* pkt = recv_enqueue(tid, radio_id, rx_offset);
        if (pkt == NULL)
            continue;
        rx_offset = (rx_offset + 1) % buffer_frame_num_;
#if MEASURE_TIME
        int frame_id = pkt->frame_id;
        if (frame_id > prev_frame_id) {
            *(rx_frame_start + frame_id) = get_time();
            prev_frame_id = frame_id;
            if (frame_id % 512 == 200) {
                _mm_prefetch(
                    (char*)(rx_frame_start + frame_id + 512), _MM_HINT_T0);
            }
        }
#endif

        if (++radio_id == radio_hi)
            radio_id = radio_lo;
    }
    return 0;
}

struct Packet* PacketTXRX::recv_enqueue(int tid, int radio_id, int rx_offset)
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
    rx_buffer_status[rx_offset]
        = 1; // has data, after it is read, it is set to 0

    // Push kPacketRX event into the queue.
    Event_data rx_message(EventType::kPacketRX, rx_tag_t(tid, rx_offset)._tag);
    if (!message_queue_->enqueue(*local_ptok, rx_message)) {
        printf("socket message enqueue failed\n");
        exit(0);
    }
    return pkt;
}

int PacketTXRX::dequeue_send(int tid)
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
    symbol_id += config_->pilot_symbol_num_perframe;
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
    new (pkt) Packet(frame_id, symbol_id, 0 /* cell_id */, ant_id);

    // send data (one OFDM symbol)
    if (sendto(socket_[ant_id % config_->socket_thread_num],
            (char*)cur_buffer_ptr, packet_length, 0,
            (struct sockaddr*)&servaddr_[tid], sizeof(servaddr_[tid]))
        < 0) {
        perror("socket sendto failed");
        exit(0);
    }

    Event_data tx_message(EventType::kPacketTX, offset);
    moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];
    if (!message_queue_->enqueue(*local_ptok, tx_message)) {
        printf("socket message enqueue failed\n");
        exit(0);
    }
    return offset;
}
