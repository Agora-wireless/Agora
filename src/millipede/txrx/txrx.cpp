/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 * Implementation of PacketTXRX initialization functions, and datapath functions
 * for communicating with simulators.
 */

#include "txrx.hpp"

PacketTXRX::PacketTXRX(Config* cfg, size_t core_offset, RxStatus* rx_status,
    DemulStatus* demul_status, DecodeStatus* decode_status)
    : cfg(cfg)
    , core_offset(core_offset)
    , socket_thread_num(cfg->socket_thread_num)
    , rx_status_(rx_status)
    , demul_status_(demul_status)
    , decode_status_(decode_status)
{
    if (!kUseArgos) {
        socket_.resize(cfg->nRadios);
        servaddr_.resize(cfg->nRadios);
        if (cfg->disable_master) {
            millipede_addrs_.resize(cfg->server_addr_list.size());
        }
    } else {
        radioconfig_ = new RadioConfig(cfg);
    }
    demod_symbol_to_send = cfg->pilot_symbol_num_perframe;
}

PacketTXRX::PacketTXRX(Config* cfg, size_t core_offset,
    moodycamel::ConcurrentQueue<Event_data>* queue_message,
    moodycamel::ConcurrentQueue<Event_data>* queue_task,
    moodycamel::ProducerToken** rx_ptoks, moodycamel::ProducerToken** tx_ptoks)
    : PacketTXRX(cfg, core_offset)
{
    message_queue_ = queue_message;
    task_queue_ = queue_task;
    rx_ptoks_ = rx_ptoks;
    tx_ptoks_ = tx_ptoks;
}

PacketTXRX::~PacketTXRX()
{
    if (kUseArgos) {
        radioconfig_->radioStop();
        delete radioconfig_;
    }
}

bool PacketTXRX::startTXRX(Table<char>& buffer, Table<int>& buffer_status,
    size_t packet_num_in_buffer, Table<size_t>& frame_start, char* tx_buffer,
    Table<int8_t>* demod_soft_buffer,
    Table<int8_t>* demod_soft_buffer_to_decode)
{
    buffer_ = &buffer;
    buffer_status_ = &buffer_status;
    frame_start_ = &frame_start;

    packet_num_in_buffer_ = packet_num_in_buffer;
    tx_buffer_ = tx_buffer;

    demod_soft_buffer_ = demod_soft_buffer;
    demod_soft_buffer_to_decode_ = demod_soft_buffer_to_decode;

    if (kUseArgos) {
        if (!radioconfig_->radioStart()) {
            fprintf(stderr, "Failed to start radio\n");
            return false;
        }
    }

    printf("Creating %zu TX/RX threads\n", socket_thread_num);
    for (size_t i = 0; i < socket_thread_num; i++) {
        pthread_t txrx_thread;
        auto context = new EventHandlerContext<PacketTXRX>;
        context->obj_ptr = this;
        context->id = i;

        if (kUseArgos) {
            int ret = pthread_create(&txrx_thread, NULL,
                pthread_fun_wrapper<PacketTXRX, &PacketTXRX::loop_tx_rx_argos>,
                context);
            rt_assert(ret == 0, "Failed to create threads");
        } else {
            int ret = pthread_create(&txrx_thread, NULL,
                pthread_fun_wrapper<PacketTXRX, &PacketTXRX::loop_tx_rx>,
                context);
            rt_assert(ret == 0, "Failed to create threads");
        }
    }

    if (kUseArgos)
        radioconfig_->go();
    return true;
}

void* PacketTXRX::loop_tx_rx(int tid)
{
    pin_to_core_with_offset(ThreadType::kWorkerTXRX, core_offset, tid);
    size_t* rx_frame_start = (*frame_start_)[tid];
    size_t rx_offset = 0;
    int radio_lo = tid * cfg->nRadios / socket_thread_num;
    int radio_hi = (tid + 1) * cfg->nRadios / socket_thread_num;
    printf("Receiver thread %d has %d radios\n", tid, radio_hi - radio_lo);

    int sock_buf_size = 1024 * 1024 * 64 * 8 - 1;
    for (int radio_id = radio_lo; radio_id < radio_hi; ++radio_id) {
        int local_port_id = cfg->bs_port + radio_id;
        socket_[radio_id]
            = setup_socket_ipv4(local_port_id, true, sock_buf_size);
        setup_sockaddr_remote_ipv4(&servaddr_[radio_id],
            cfg->ue_rx_port + radio_id, cfg->sender_addr.c_str());
        printf("TXRX thread %d: set up UDP socket server listening to port %d"
               " with remote address %s:%d \n",
            tid, local_port_id, cfg->sender_addr.c_str(),
            cfg->ue_rx_port + radio_id);
        fcntl(socket_[radio_id], F_SETFL, O_NONBLOCK);
    }

    if (cfg->disable_master && tid == 0) {
        for (size_t i = 0; i < cfg->server_addr_list.size(); i++) {
            setup_sockaddr_remote_ipv4(&millipede_addrs_[i], cfg->bs_port,
                cfg->server_addr_list[i].c_str());
        }
    }

    int prev_frame_id = -1;
    int radio_id = radio_lo;
    while (cfg->running) {
        Packet* pkt;
        if (cfg->disable_master) {
            if (-1 != poll_send(tid))
                continue;
            pkt = recv_relocate(tid, radio_id, rx_offset);
        } else {
            if (-1 != dequeue_send(tid))
                continue;
            pkt = recv_enqueue(tid, radio_id, rx_offset);
        }
        // receive data
        if (pkt == NULL)
            continue;
        rx_offset = (rx_offset + 1) % packet_num_in_buffer_;

        if (kIsWorkerTimingEnabled) {
            int frame_id = pkt->frame_id;
            if (frame_id > prev_frame_id) {
                rx_frame_start[frame_id % kNumStatsFrames] = rdtsc();
                prev_frame_id = frame_id;
            }
        }

        if (++radio_id == radio_hi)
            radio_id = radio_lo;
    }
    return 0;
}

struct Packet* PacketTXRX::recv_relocate(int tid, int radio_id, int rx_offset)
{
    // TODO [junzhi]: Can we avoid malloc
    char* buf = reinterpret_cast<char*>(malloc(cfg->packet_length));
    auto* pkt = reinterpret_cast<Packet*>(buf);
    // if (-1
    //     == recv(socket_[radio_id], (char*)pkt,
    //            Packet::kOffsetOfData
    //                + cfg->get_ofdm_control_num() * 2 * sizeof(unsigned short),
    //            0)) {
    if (-1 == recv(socket_[radio_id], (char*)pkt, cfg->packet_length, 0)) {
        if (errno != EAGAIN && cfg->running) {
            perror("recv failed");
            exit(0);
        }
        free(buf);
        return (NULL);
    }

    if (pkt->packet_type == Packet::PacketType::kRRU) {
        char* rx_buffer = (*buffer_)[pkt->ant_id];
        int* rx_buffer_status = (*buffer_status_)[pkt->ant_id];
        int packet_length = cfg->packet_length;
        size_t frame_id = pkt->frame_id;
        size_t symbol_id = pkt->symbol_id;
        size_t rx_offset_
            = (frame_id % SOCKET_BUFFER_FRAME_NUM) * cfg->symbol_num_perframe
            + symbol_id;

        size_t sc_offset = Packet::kOffsetOfData
            + 2 * sizeof(unsigned short)
                * (cfg->OFDM_DATA_START
                      + cfg->server_addr_idx * cfg->get_ofdm_control_num());
        memcpy(
            &rx_buffer[rx_offset_ * packet_length], buf, Packet::kOffsetOfData);
        memcpy(&rx_buffer[rx_offset_ * packet_length + sc_offset],
            buf + Packet::kOffsetOfData,
            cfg->get_ofdm_control_num() * 2 * sizeof(unsigned short));
        free(buf);

        // get the position in rx_buffer
        // move ptr & set status to full
        // rx_buffer_status[rx_offset] = 1;

        if (!rx_status_->add_new_packet(frame_id, symbol_id)) {
            cfg->running = false;
        }
    } else if (pkt->packet_type == Packet::PacketType::kDemod) {
        size_t frame_id = pkt->frame_id;
        size_t symbol_id = pkt->symbol_id;
        size_t ue_id = pkt->ue_id;
        size_t server_id = pkt->server_id;
        size_t sc_id = server_id * cfg->get_ofdm_control_num();
        int8_t* demod_ptr = cfg->get_demod_buf(
            *demod_soft_buffer_to_decode_, frame_id, symbol_id, ue_id, sc_id);
        memcpy(
            demod_ptr, pkt->data, cfg->get_ofdm_control_num() * cfg->mod_type);
        decode_status_->receive_demod_data(ue_id, frame_id, symbol_id);
    }
    return pkt;
}

struct Packet* PacketTXRX::recv_enqueue(int tid, int radio_id, int rx_offset)
{
    moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];
    char* rx_buffer = (*buffer_)[tid];
    int* rx_buffer_status = (*buffer_status_)[tid];
    int packet_length = cfg->packet_length;

    // if rx_buffer is full, exit
    if (rx_buffer_status[rx_offset] == 1) {
        printf(
            "Receive thread %d rx_buffer full, offset: %d\n", tid, rx_offset);
        cfg->running = false;
        return (NULL);
    }
    struct Packet* pkt = (struct Packet*)&rx_buffer[rx_offset * packet_length];
    if (-1 == recv(socket_[radio_id], (char*)pkt, packet_length, 0)) {
        if (errno != EAGAIN && cfg->running) {
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

int PacketTXRX::poll_send(int tid)
{
    if (tid == 0
        && demul_status_->ready_to_decode(
               demod_frame_to_send, demod_symbol_to_send)) {
        for (size_t ue_id = 0; ue_id < cfg->UE_NUM; ue_id++) {
            int8_t* demod_ptr
                = cfg->get_demod_buf(*demod_soft_buffer_, demod_frame_to_send,
                    demod_symbol_to_send, ue_id, cfg->subcarrier_start);
            Packet pkt(demod_frame_to_send, demod_symbol_to_send, ue_id,
                cfg->server_addr_idx);
            memcpy(pkt.data, demod_ptr,
                cfg->get_ofdm_control_num() * cfg->mod_type);
            ssize_t ret = sendto(socket_[1], &pkt, cfg->packet_length, 0,
                (struct sockaddr*)&millipede_addrs_[cfg->get_server_idx_by_ue(
                    ue_id)],
                sizeof(millipede_addrs_[cfg->get_server_idx_by_ue(ue_id)]));
            rt_assert(ret > 0, "sendto() failed");
        }
        return 1;
    }
    return -1;
}

int PacketTXRX::dequeue_send(int tid)
{
    auto& c = cfg;
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
    ssize_t ret = sendto(socket_[ant_id % cfg->socket_thread_num],
        cur_buffer_ptr, c->packet_length, 0, (struct sockaddr*)&servaddr_[tid],
        sizeof(servaddr_[tid]));
    rt_assert(ret > 0, "sendto() failed");

    rt_assert(message_queue_->enqueue(*rx_ptoks_[tid],
                  Event_data(EventType::kPacketTX, event.tags[0])),
        "Socket message enqueue failed\n");
    return event.tags[0];
}
