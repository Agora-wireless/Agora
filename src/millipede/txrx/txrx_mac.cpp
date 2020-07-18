/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */

#include "txrx_mac.hpp"

MacPacketTXRX::MacPacketTXRX(Config* cfg, size_t core_offset)
    : cfg(cfg)
    , core_offset(core_offset)
    , mac_thread_num(cfg->mac_socket_thread_num)
{

    socket_ = new int[cfg->UE_NUM];
    rx_buffer_.calloc(mac_thread_num, cfg->OFDM_DATA_NUM + 64, 64);
    tx_buffer_.calloc(mac_thread_num, cfg->OFDM_DATA_NUM + 64, 64);

#if USE_IPV4
    servaddr_ = new struct sockaddr_in[cfg->UE_NUM];
#else
    servaddr_ = new struct sockaddr_in6[cfg->UE_NUM];
#endif
}

MacPacketTXRX::MacPacketTXRX(Config* cfg, size_t core_offset,
    moodycamel::ConcurrentQueue<Event_data>* queue_message,
    moodycamel::ConcurrentQueue<Event_data>* queue_task,
    moodycamel::ProducerToken** rx_ptoks, moodycamel::ProducerToken** tx_ptoks)
    : MacPacketTXRX(cfg, core_offset)
{
    message_queue_ = queue_message;
    task_queue_ = queue_task;
    rx_ptoks_ = rx_ptoks;
    tx_ptoks_ = tx_ptoks;
}

MacPacketTXRX::~MacPacketTXRX()
{
    delete[] socket_;
    delete[] servaddr_;
}

bool MacPacketTXRX::startTXRX(Table<int8_t>& dl_bits_buffer,
    Table<int>& dl_bits_buffer_status, size_t packet_num_in_buffer,
    Table<uint8_t>& ul_bits_buffer)
{
    dl_bits_buffer_ = &dl_bits_buffer;
    dl_bits_buffer_status_ = &dl_bits_buffer_status;

    packet_num_in_buffer_ = packet_num_in_buffer;
    ul_bits_buffer_ = &ul_bits_buffer;

    printf("create %zu MAC TXRX threads\n", mac_thread_num);
    for (size_t i = 0; i < mac_thread_num; i++) {
        pthread_t txrx_thread;
        auto context = new EventHandlerContext<MacPacketTXRX>;
        context->obj_ptr = this;
        context->id = i;
        if (pthread_create(&txrx_thread, NULL,
                pthread_fun_wrapper<MacPacketTXRX, &MacPacketTXRX::loopTXRX>,
                context)
            != 0) {
            perror("socket communication thread create failed");
            exit(0);
        }
    }
    return true;
}

void* MacPacketTXRX::loopTXRX(int tid)
{
    pin_to_core_with_offset(ThreadType::kWorkerMacTXRX, core_offset, tid);
    // int rx_offset = 0;
    int radio_lo = tid * cfg->UE_NUM / mac_thread_num;
    int radio_hi = (tid + 1) * cfg->UE_NUM / mac_thread_num;
    printf("MAC receiver thread %d has %d radios\n", tid, radio_hi - radio_lo);

    int sock_buf_size = 1024 * 1024 * 64 * 8 - 1;
    for (int radio_id = radio_lo; radio_id < radio_hi; ++radio_id) {
        int local_port_id = cfg->mac_tx_port + radio_id;
#if USE_IPV4
        socket_[radio_id]
            = setup_socket_ipv4(local_port_id, true, sock_buf_size);
        setup_sockaddr_remote_ipv4(&servaddr_[radio_id],
            cfg->mac_rx_port + radio_id, cfg->tx_addr_to_mac.c_str());
        printf(
            "MAC TXRX thread %d: set up UDP socket server listening to port %d"
            " with remote address %s:%d  \n",
            tid, local_port_id, cfg->tx_addr_to_mac.c_str(),
            cfg->mac_rx_port + tid);
#else
        socket_[radio_id]
            = setup_socket_ipv6(local_port_id, true, sock_buf_size);
        setup_sockaddr_remote_ipv6(&servaddr_[radio_id],
            cfg->mac_rx_port + radio_id, cfg->tx_addr_to_mac.c_str());
#endif
        fcntl(socket_[radio_id], F_SETFL, O_NONBLOCK);
    }

    // int prev_frame_id = -1;
    int radio_id = radio_lo;
    while (cfg->running) {
        if (-1 != dequeue_send(tid))
            continue;
        // receive data
        //struct MacPacket* pkt = recv_enqueue(tid, radio_id);
        //if (pkt == NULL)
        //    continue;
        // rx_offset = (rx_offset + 1) % packet_num_in_buffer_;

        if (++radio_id == radio_hi)
            radio_id = radio_lo;
    }
    return 0;
}

struct MacPacket* MacPacketTXRX::recv_enqueue(int tid, int radio_id)
{
    moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];
    char* rx_buffer = rx_buffer_[tid];
    // int* dl_bits_buffer_status = *dl_bits_buffer_status_;
    int packet_length = kUseLDPC ? (bits_to_bytes(cfg->LDPC_config.cbLen)
                                       * cfg->LDPC_config.nblocksInSymbol)
                                 : bits_to_bytes(cfg->OFDM_DATA_NUM);
    packet_length += MacPacket::kOffsetOfData;

    struct MacPacket* pkt = (struct MacPacket*)rx_buffer;
    // int ret = recv(socket_[radio_id], (char*)pkt, packet_length, 0);
    socklen_t addrlen = sizeof(servaddr_[radio_id]);
    int ret = recvfrom(socket_[radio_id], (char*)pkt, packet_length, 0,
        (struct sockaddr*)&servaddr_[radio_id], &addrlen);
    if (ret == -1) {
        if (errno != EAGAIN && cfg->running) {
            perror("recv failed");
            exit(0);
        }
        return (NULL);
    }
    // printf("received data %d\n", ret);
    // printf("IP address is: %s\n", inet_ntoa(servaddr_[radio_id].sin_addr));
    // printf("port is: %d\n", (int)ntohs(servaddr_[radio_id].sin_port));

    size_t total_symbol_idx
        = cfg->get_total_data_symbol_idx_dl(pkt->frame_id, pkt->symbol_id);
    int rx_offset
        = total_symbol_idx * (kUseLDPC ? cfg->LDPC_config.nblocksInSymbol : 1);
    if ((*dl_bits_buffer_status_)[pkt->ue_id][rx_offset] == 1) {
        printf("MAC Receive thread %d dl_bits_buffer full, offset: %d\n", tid,
            rx_offset);
        cfg->running = false;
        return (NULL);
    } else {
        for (int i = 0; i < (kUseLDPC ? cfg->LDPC_config.nblocksInSymbol : 1);
             i++)
            (*dl_bits_buffer_status_)[pkt->ue_id][rx_offset + i] = 1;
        memcpy(&(*dl_bits_buffer_)[total_symbol_idx]
                                  [pkt->ue_id * cfg->OFDM_DATA_NUM],
            pkt->data, packet_length);
    }

    // Push kPacketRX event into the queue.
    Event_data rx_message(EventType::kPacketFromMac,
        gen_tag_t::frm_sym_ue(pkt->frame_id, pkt->symbol_id, pkt->ue_id)._tag);
    if (!message_queue_->enqueue(*local_ptok, rx_message)) {
        printf("socket message enqueue failed\n");
        exit(0);
    }
    return pkt;
}

int MacPacketTXRX::dequeue_send(int tid)
{
    Event_data event;
    if (!task_queue_->try_dequeue_from_producer(*tx_ptoks_[tid], event))
        return -1;
    // printf("mac queue length: %zu\n", task_queue_->size_approx());
    assert(event.event_type == EventType::kPacketToMac);

    size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
    size_t symbol_id = gen_tag_t(event.tags[0]).symbol_id;
    size_t ue_id = gen_tag_t(event.tags[0]).ue_id;

    int packet_length = cfg->data_bytes_num_persymbol;

    size_t cbLenBytes = (cfg->LDPC_config.cbLen + 7) >> 3;
    size_t data_offset = cbLenBytes * cfg->LDPC_config.nblocksInSymbol * ue_id;
    if (!kUseLDPC)
        data_offset = cfg->OFDM_DATA_NUM * ue_id;

    size_t total_symbol_idx
        = cfg->get_total_data_symbol_idx_ul(frame_id, symbol_id);
    uint8_t* ul_data_ptr
        = &(*ul_bits_buffer_)[total_symbol_idx][data_offset];
    auto* pkt = (MacPacket*)tx_buffer_[tid];
    new (pkt) MacPacket(frame_id,
                        symbol_id,
                        0 /* cell_id */, 
                        ue_id,
                        0 /* valid tun */,
                        0 /* CRC */,
                        0 /* datalen */);
    pkt->frame_id = frame_id;
    pkt->symbol_id = symbol_id;
    pkt->ue_id = ue_id;
    if (!kUseLDPC)
        adapt_bits_from_mod((int8_t*)ul_data_ptr, (int8_t*)pkt->data,
            cfg->OFDM_DATA_NUM, cfg->mod_type);
    else
        memcpy(pkt->data, ul_data_ptr, packet_length);

    int mac_packet_length = packet_length + MacPacket::kOffsetOfData;

    // Send data (one OFDM symbol)
    size_t ret
        = sendto(socket_[ue_id % cfg->UE_NUM], (char*)pkt, mac_packet_length, 0,
            (struct sockaddr*)&servaddr_[tid], sizeof(servaddr_[tid]));
    rt_assert(ret > 0, "sendto() failed");

    if (kDebugPrintInTask) {
        printf("In MAC TX thread %d: Transmitted frame %zu, symbol %zu, "
               "ue %zu, tag %s, msg_queue_length: %zu\n",
            tid, frame_id, symbol_id, ue_id,
            gen_tag_t(event.tags[0]).to_string().c_str(),
            message_queue_->size_approx());
    }

    rt_assert(message_queue_->enqueue(*rx_ptoks_[tid],
                  Event_data(EventType::kPacketToMac, event.tags[0])),
        "Socket message enqueue failed\n");
    return event.tags[0];
}
