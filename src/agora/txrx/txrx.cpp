/**
 * @file txrx.cpp
 * @brief Implementation of PacketTXRX initialization functions, and datapath
 * functions for communicating with simulators.
 */

#include "txrx.hpp"
#include "logger.h"
#include "udp_server.h"

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
        bs_rru_sockaddr_.resize(cfg->nRadios);
        if (cfg->disable_master) {
            millipede_addrs_.resize(cfg->server_addr_list.size());
        }
    } else {
        radioconfig_ = new RadioConfig(cfg);
    }
    demod_symbol_to_send = cfg->pilot_symbol_num_perframe;
    send_buffer_ = reinterpret_cast<char*>(memalign(64, cfg->packet_length));
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
    free(send_buffer_);
}

bool PacketTXRX::startTXRX(Table<char>& buffer, Table<int>& buffer_status,
    size_t packet_num_in_buffer, Table<size_t>& frame_start, char* tx_buffer,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>* demod_buffers,
    Table<int8_t>* demod_soft_buffer_to_decode)
{
    buffer_ = &buffer;
    buffer_status_ = &buffer_status;
    frame_start_ = &frame_start;

    packet_num_in_buffer_ = packet_num_in_buffer;
    tx_buffer_ = tx_buffer;

    demod_buffers_ = demod_buffers;
    demod_soft_buffer_to_decode_ = demod_soft_buffer_to_decode;

    if (kUseArgos) {
        if (!radioconfig_->radioStart()) {
            fprintf(stderr, "Failed to start radio\n");
            return false;
        }
    }

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

    if (cfg->disable_master) {
        pthread_t txrx_thread;
        auto context = new EventHandlerContext<PacketTXRX>;
        context->obj_ptr = this;
        context->id = socket_thread_num;
        int ret = pthread_create(&txrx_thread, NULL,
            pthread_fun_wrapper<PacketTXRX, &PacketTXRX::demod_thread>,
            context);
        rt_assert(ret == 0, "Failed to create threads");
    }

    if (kUseArgos)
        radioconfig_->go();
    return true;
}

void* PacketTXRX::demod_thread(int tid)
{
    std::vector<uint8_t> recv_buf(cfg->packet_length);
    UDPServer udp_server(cfg->demod_rx_port);

    pin_to_core_with_offset(
        ThreadType::kWorkerTXRX, core_offset, cfg->socket_thread_num);

    printf("Demodulation TX/RX thread\n");
    int sock_buf_size = 1024 * 1024 * 64 * 8 - 1;
    demod_tx_socket_
        = setup_socket_ipv4(cfg->demod_tx_port, true, sock_buf_size);

    for (size_t i = 0; i < cfg->server_addr_list.size(); i++) {
        setup_sockaddr_remote_ipv4(&millipede_addrs_[i], cfg->demod_rx_port,
            cfg->server_addr_list[i].c_str());
    }

    while (cfg->running) {
        // 1. Try to send demodulated data to decoders
        if (demul_status_->ready_to_decode(
                demod_frame_to_send, demod_symbol_to_send)) {
            for (size_t ue_id = 0; ue_id < cfg->UE_NUM; ue_id++) {
                int8_t* demod_ptr = (*demod_buffers_)[demod_frame_to_send
                    % kFrameWnd][demod_symbol_to_send
                    - cfg->pilot_symbol_num_perframe][ue_id];

                auto* pkt = reinterpret_cast<Packet*>(send_buffer_);
                pkt->pkt_type = Packet::PktType::kDemod;
                pkt->frame_id = demod_frame_to_send;
                pkt->symbol_id = demod_symbol_to_send;
                pkt->ue_id = ue_id;
                pkt->server_id = cfg->server_addr_idx;
                memcpy(pkt->data, demod_ptr,
                    cfg->get_num_sc_per_server() * cfg->mod_order_bits);
                ssize_t ret = sendto(demod_tx_socket_, pkt, cfg->packet_length,
                    MSG_DONTWAIT,
                    (struct sockaddr*)&millipede_addrs_
                        [cfg->get_server_idx_by_ue(ue_id)],
                    sizeof(millipede_addrs_[cfg->get_server_idx_by_ue(ue_id)]));
                rt_assert(ret > 0, "sendto() failed");
            }
            demod_symbol_to_send++;
            if (demod_symbol_to_send
                == cfg->pilot_symbol_num_perframe
                    + cfg->ul_data_symbol_num_perframe) {
                demod_symbol_to_send = cfg->pilot_symbol_num_perframe;
                demod_frame_to_send++;
            }
        }

        // 2. Try to receive demodulated data for decoding
        ssize_t ret
            = udp_server.recv_nonblocking(&recv_buf[0], cfg->packet_length);
        rt_assert(ret >= 0, "Demod thread failed to recv()");
        if (ret == 0) // No new data
            continue;

        auto* pkt = reinterpret_cast<Packet*>(&recv_buf[0]);
        rt_assert(pkt->pkt_type == Packet::PktType::kDemod,
            "Received unknown packet type in demod TX/RX thread");

        const size_t symbol_idx_ul
            = pkt->symbol_id - cfg->pilot_symbol_num_perframe;
        const size_t sc_id = pkt->server_id * cfg->get_num_sc_per_server();

        int8_t* demod_ptr
            = cfg->get_demod_buf_to_decode(*demod_soft_buffer_to_decode_,
                pkt->frame_id, symbol_idx_ul, pkt->ue_id, sc_id);
        memcpy(demod_ptr, pkt->data,
            cfg->get_num_sc_per_server() * cfg->mod_order_bits);
        decode_status_->receive_demod_data(
            pkt->ue_id, pkt->frame_id, symbol_idx_ul);
    }

    return 0;
}

void PacketTXRX::send_beacon(int tid, size_t frame_id)
{
    int radio_lo = tid * cfg->nRadios / socket_thread_num;
    int radio_hi = (tid + 1) * cfg->nRadios / socket_thread_num;

    // Send a beacon packet in the downlink to trigger user pilot
    std::vector<uint8_t> udp_pkt_buf(cfg->packet_length, 0);
    auto* pkt = reinterpret_cast<Packet*>(&udp_pkt_buf[0]);
    for (int ant_id = radio_lo; ant_id < radio_hi; ant_id++) {
        new (pkt) Packet(frame_id, 0, 0 /* cell_id */, ant_id);
        ssize_t r = sendto(socket_[ant_id], (char*)udp_pkt_buf.data(),
            cfg->packet_length, 0, (struct sockaddr*)&bs_rru_sockaddr_[ant_id],
            sizeof(bs_rru_sockaddr_[ant_id]));
        rt_assert(r > 0, "sendto() failed");
    }
}

void* PacketTXRX::loop_tx_rx(int tid)
{
    pin_to_core_with_offset(
        ThreadType::kWorkerTXRX, core_offset, tid, false /* quiet */);
    size_t* rx_frame_start = (*frame_start_)[tid];
    size_t rx_offset = 0;
    int radio_lo = tid * cfg->nRadios / socket_thread_num;
    int radio_hi = (tid + 1) * cfg->nRadios / socket_thread_num;

    int sock_buf_size = 1024 * 1024 * 64 * 8 - 1;
    for (int radio_id = radio_lo; radio_id < radio_hi; ++radio_id) {
        int local_port_id = cfg->bs_server_port + radio_id;
        socket_[radio_id]
            = setup_socket_ipv4(local_port_id, true, sock_buf_size);
        setup_sockaddr_remote_ipv4(&bs_rru_sockaddr_[radio_id],
            cfg->bs_rru_port + radio_id, cfg->bs_rru_addr.c_str());
        MLPD_INFO(
            "TXRX thread %d: set up UDP socket server listening to port %d"
            " with remote address %s:%d \n",
            tid, local_port_id, cfg->bs_rru_addr.c_str(),
            cfg->bs_rru_port + radio_id);
        fcntl(socket_[radio_id], F_SETFL, O_NONBLOCK);
    }

    size_t frame_tsc_delta(
        cfg->get_frame_duration_sec() * 1e9 * measure_rdtsc_freq());
    int prev_frame_id = -1;
    int radio_id = radio_lo;
    size_t tx_frame_start = rdtsc();
    size_t tx_frame_id = 0;
    size_t slow_start_factor = 10;
    send_beacon(
        tid, tx_frame_id++); // Send Beacons for the first time to kick off sim
    while (cfg->running) {
        if (rdtsc() - tx_frame_start > frame_tsc_delta * slow_start_factor) {
            tx_frame_start = rdtsc();
            send_beacon(tid, tx_frame_id++);
            if (tx_frame_id > 5)
                slow_start_factor = 5;
            else if (tx_frame_id > 100)
                slow_start_factor = 4;
            else if (tx_frame_id > 200)
                slow_start_factor = 2;
            else if (tx_frame_id > 500)
                slow_start_factor = 1;
        }

        if (!cfg->disable_master) {
            if (-1 != dequeue_send(tid))
                continue;
        }

        // Receive data
        Packet* pkt = cfg->disable_master
            ? recv_relocate(tid, radio_id, rx_offset)
            : recv_enqueue(tid, radio_id, rx_offset);
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

Packet* PacketTXRX::recv_relocate(int tid, int radio_id, int rx_offset)
{
    // TODO [junzhi]: Can we avoid malloc? Currently we're leaking buf because
    // we return buf as a Packet to the caller.
    uint8_t* buf = reinterpret_cast<uint8_t*>(malloc(cfg->packet_length));

    if (-1 == recv(socket_[radio_id], buf, cfg->packet_length, 0)) {
        if (errno != EAGAIN && cfg->running) {
            fprintf(stderr, "PacketTXRX: recv_relocate(), socket recv() error");
            exit(-1);
        }
        free(buf);
        return nullptr;
    }

    const auto* pkt = reinterpret_cast<Packet*>(buf);
    MLPD_TRACE("PacketTXRX: Thread %d received packet %s\n", tid,
        pkt->to_string().c_str());

    if (pkt->pkt_type == Packet::PktType::kIQFromRRU) {
        char* rx_buffer = (*buffer_)[pkt->ant_id];
        const size_t rx_offset_ = (pkt->frame_id % SOCKET_BUFFER_FRAME_NUM)
                * cfg->symbol_num_perframe
            + pkt->symbol_id;

        size_t sc_offset = Packet::kOffsetOfData
            + 2 * sizeof(unsigned short)
                * (cfg->OFDM_DATA_START
                      + cfg->server_addr_idx * cfg->get_num_sc_per_server());
        memcpy(&rx_buffer[rx_offset_ * cfg->packet_length], buf,
            Packet::kOffsetOfData);
        memcpy(&rx_buffer[rx_offset_ * cfg->packet_length + sc_offset],
            buf + Packet::kOffsetOfData,
            cfg->get_num_sc_per_server() * 2 * sizeof(unsigned short));

        // get the position in rx_buffer
        // move ptr & set status to full
        // rx_buffer_status[rx_offset] = 1;
        if (!rx_status_->add_new_packet(pkt)) {
            cfg->running = false;
        }
    } else {
        printf("Received unknown packet from rru\n");
        exit(1);
    }
    return reinterpret_cast<Packet*>(buf);
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
    size_t symbol_id = gen_tag_t(event.tags[0]).symbol_id;

    size_t data_symbol_idx_dl = cfg->get_dl_symbol_idx(frame_id, symbol_id);
    size_t offset = (c->get_total_data_symbol_idx_dl(frame_id, data_symbol_idx_dl)
                        * c->BS_ANT_NUM)
        + ant_id;

    if (kDebugPrintInTask) {
        printf("In TX thread %d: Transmitted frame %zu, symbol %zu, "
               "ant %zu, tag %zu, offset: %zu, msg_queue_length: %zu\n",
            tid, frame_id, symbol_id, ant_id,
            gen_tag_t(event.tags[0])._tag, offset,
            message_queue_->size_approx());
    }

    char* cur_buffer_ptr = tx_buffer_ + offset * c->packet_length;
    auto* pkt = (Packet*)cur_buffer_ptr;
    new (pkt) Packet(frame_id, symbol_id, 0 /* cell_id */, ant_id);

    // Send data (one OFDM symbol)
    ssize_t ret = sendto(socket_[ant_id], cur_buffer_ptr, c->packet_length, 0,
        (struct sockaddr*)&bs_rru_sockaddr_[ant_id],
        sizeof(bs_rru_sockaddr_[ant_id]));
    rt_assert(ret > 0, "sendto() failed");

    rt_assert(message_queue_->enqueue(*rx_ptoks_[tid],
                  Event_data(EventType::kPacketTX, event.tags[0])),
        "Socket message enqueue failed\n");
    return event.tags[0];
}
