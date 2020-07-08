/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */

#include "txrx_mac.hpp"

PacketTXRX::PacketTXRX(Config* cfg, int COMM_THREAD_NUM, int in_core_offset)
{
    config_ = cfg;
    comm_thread_num_ = COMM_THREAD_NUM;

    core_id_ = in_core_offset;
    tx_core_id_ = in_core_offset + COMM_THREAD_NUM;

    mac_running = false;

    /* initialize random seed: */
    srand(time(NULL));

    socket_ = new int[config_->UE_ANT_NUM];
#if USE_IPV4
    servaddr_ = new struct sockaddr_in[config_->UE_ANT_NUM];
#else
    servaddr_ = new struct sockaddr_in6[config_->UE_ANT_NUM];
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

void PacketTXRX::wakeup_mac()
{
    mac_running = true;
}

bool PacketTXRX::is_mac_running()
{
    return mac_running;
}

bool PacketTXRX::startTXRX(Table<char>& in_buffer, Table<int>& in_buffer_status,
    int in_buffer_frame_num, long long in_buffer_length, char* in_tx_buffer,
    int* in_tx_buffer_status, int in_tx_buffer_frame_num,
    int in_tx_buffer_length)
{
    buffer_ = &in_buffer; // for save data
    buffer_status_ = &in_buffer_status; // for save status

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
    pin_to_core_with_offset(ThreadType::kWorkerMacTXRX, core_id_, tid);
    std::vector<int> rx_offset(config_->UE_ANT_NUM, 0);
    int radio_lo = tid * config_->UE_ANT_NUM / comm_thread_num_;
    int radio_hi = (tid + 1) * config_->UE_ANT_NUM / comm_thread_num_;
    printf("MAC TXRX thread %d has %d radios\n", tid, radio_hi - radio_lo);

    int sock_buf_size = 1024 * 1024 * 64 * 8 - 1;
    for (int radio_id = radio_lo; radio_id < radio_hi; ++radio_id) {
        int local_port_id = config_->mac_rx_port + radio_id;
#if USE_IPV4
        socket_[radio_id]
            = setup_socket_ipv4(local_port_id, true, sock_buf_size);
        setup_sockaddr_remote_ipv4(&servaddr_[radio_id],
            config_->ue_tx_port + radio_id, config_->tx_addr.c_str());
#else
        socket_[radio_id]
            = setup_socket_ipv6(local_port_id, true, sock_buf_size);
        setup_sockaddr_remote_ipv6(&servaddr_[radio_id],
            config_->ue_tx_port + radio_id, config_->tx_addr.c_str());
#endif
        fcntl(socket_[radio_id], F_SETFL, O_NONBLOCK);
        printf("Set up UDP socket server listening to port %d"
               " with remote address %s:%d  \n",
            local_port_id, config_->tx_addr.c_str(),
            config_->ue_tx_port + radio_id);
    }

    while(config_->running && !is_mac_running());

    // send start notification to mac
    char* start_msg[1024];
    ssize_t ret = sendto(socket_[0],
        start_msg, 1024, 0, (struct sockaddr*)&servaddr_[0],
        sizeof(servaddr_[0]));
    rt_assert(ret > 0, "sendto() failed");
    std::cout << "Waking up MAC.." << std::endl;

    int radio_id = radio_lo;
    while (config_->running) {
        //if (-1 != dequeue_send(tid))
        //    continue;
        // receive data
        struct MacPacket* pkt
            = recv_enqueue(tid, radio_id, rx_offset[radio_id]);
        if (pkt == NULL)
            continue;
        rx_offset[radio_id] = (rx_offset[radio_id] + 1) % buffer_frame_num_;

        if (++radio_id == radio_hi)
            radio_id = radio_lo;
    }
    return 0;
}

struct MacPacket* PacketTXRX::recv_enqueue(
    UNUSED int tid, int radio_id, int rx_offset)
{
    moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];
    char* rx_buffer = (*buffer_)[radio_id];
    int* rx_buffer_status = (*buffer_status_)[radio_id];
    int packet_length = config_->mac_packet_length;

    // if rx_buffer is full, exit
    if (rx_buffer_status[rx_offset] == 1) {
        printf(
            "Receive thread %d rx_buffer full, offset: %d\n", tid, rx_offset);
        //config_->running = false;
        //return (NULL);
    }
    struct MacPacket* pkt
        = (struct MacPacket*)&rx_buffer[rx_offset * packet_length];
    // if (-1 == recv(socket_[radio_id], pkt->data, packet_length, 0)) {

    socklen_t addrlen = sizeof(servaddr_[radio_id]);
    int ret = recvfrom(socket_[radio_id], (char*)pkt, packet_length, 0,
        (struct sockaddr*)&servaddr_[radio_id], &addrlen);
    if (ret == -1) {
        if (errno != EAGAIN && config_->running) {
            perror("recv failed");
            exit(0);
        }
        return (NULL);
    }
    // printf("received data %d\n", ret);
    // printf("IP address is: %s\n", inet_ntoa(servaddr_[radio_id].sin_addr));
    // printf("port is: %d\n", (int)ntohs(servaddr_[radio_id].sin_port));

    // printf("In MAC TXRX thread %d: received frame %d, ue %d\n", tid,
    //     pkt->frame_id, pkt->ue_id);

    // get the position in rx_buffer
    // move ptr & set status to full
    rx_buffer_status[rx_offset] = 1;

    Event_data rx_message(
        EventType::kPacketFromMac, rx_tag_t(radio_id, rx_offset)._tag);
    if (!message_queue_->enqueue(*local_ptok, rx_message)) {
        printf("MAC socket message enqueue failed\n");
        exit(0);
    }
    return pkt;
}

int PacketTXRX::dequeue_send(int tid)
{
    auto& c = config_;
    Event_data event;
    if (!task_queue_->try_dequeue_from_producer(*tx_ptoks_[tid], event))
        return -1;

    // printf("tx queue length: %d\n", task_queue_->size_approx());
    assert(event.event_type == EventType::kPacketTX);

    size_t ant_id = gen_tag_t(event.tags[0]).ant_id;
    size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
    size_t data_symbol_idx = gen_tag_t(event.tags[0]).symbol_id;

    size_t offset = (c->get_total_data_symbol_idx(frame_id, data_symbol_idx)
                        * c->BS_ANT_NUM)
        + ant_id;

    if (kDebugPrintInTask) {
        printf("In TX thread %d: Transmitted frame %zu, symbol %zu, "
               "ant %zu, tag %zu, offset: %zu, msg_queue_length: %zu\n",
            tid, frame_id, data_symbol_idx, ant_id,
            gen_tag_t(event.tags[0])._tag, offset,
            message_queue_->size_approx());
    }

    size_t socket_symbol_offset = offset
        % (SOCKET_BUFFER_FRAME_NUM * c->data_symbol_num_perframe
              * c->BS_ANT_NUM);
    char* cur_buffer_ptr = tx_buffer_ + socket_symbol_offset * c->packet_length;
    auto* pkt = (Packet*)cur_buffer_ptr;
    new (pkt) Packet(frame_id, data_symbol_idx, 0 /* cell_id */, ant_id);

    // Send data (one OFDM symbol)
    ssize_t ret = sendto(socket_[ant_id % config_->socket_thread_num],
        cur_buffer_ptr, c->packet_length, 0, (struct sockaddr*)&servaddr_[tid],
        sizeof(servaddr_[tid]));
    rt_assert(ret > 0, "sendto() failed");

    rt_assert(message_queue_->enqueue(*rx_ptoks_[tid],
                  Event_data(EventType::kPacketTX, event.tags[0])),
        "Socket message enqueue failed\n");
    return event.tags[0];
}
