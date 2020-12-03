/**
 * @file txrx.cpp
 * @brief Implementation of PacketTXRX initialization functions, and datapath
 * functions for communicating with simulators.
 */

#include "txrx.hpp"
#include "logger.h"

PacketTXRX::PacketTXRX(Config* cfg, size_t core_offset)
    : cfg(cfg)
    , core_offset(core_offset)
    , ant_per_cell(cfg->bs_ant_num() / cfg->nCells)
    , socket_thread_num(cfg->socket_thread_num)
{
    if (!kUseArgos && !kUseUHD) {
        socket_.resize(cfg->nRadios);
        bs_rru_sockaddr_.resize(cfg->nRadios);
    } else {
        radioconfig_ = new RadioConfig(cfg);
    }
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
    if (kUseArgos || kUseUHD) {
        radioconfig_->radioStop();
        delete radioconfig_;
    }
    for (size_t i = 0; i < cfg->socket_thread_num; i++)
        socket_std_threads_[i].join();
}

bool PacketTXRX::startTXRX(Table<char>& buffer, Table<int>& buffer_status,
    size_t packet_num_in_buffer, Table<size_t>& frame_start, char* tx_buffer,
    Table<complex_float>& calib_dl_buffer,
    Table<complex_float>& calib_ul_buffer)
{
    buffer_ = &buffer;
    buffer_status_ = &buffer_status;
    frame_start_ = &frame_start;

    packet_num_in_buffer_ = packet_num_in_buffer;
    tx_buffer_ = tx_buffer;

    if (kUseArgos || kUseUHD) {
        if (!radioconfig_->radioStart()) {
            std::fprintf(stderr, "Failed to start radio\n");
            return false;
        }
        std::memcpy(calib_dl_buffer[0], radioconfig_->get_calib_dl(),
            cfg->ofdm_data_num() * cfg->bf_ant_num() * sizeof(arma::cx_float));
        std::memcpy(calib_ul_buffer[0], radioconfig_->get_calib_ul(),
            cfg->ofdm_data_num() * cfg->bf_ant_num() * sizeof(arma::cx_float));
    }

    for (size_t i = 0; i < socket_thread_num; i++) {
        if (kUseArgos)
            socket_std_threads_[i]
                = std::thread(&PacketTXRX::loop_tx_rx_argos, this, i);
        else if (kUseUHD)
            socket_std_threads_[i]
                = std::thread(&PacketTXRX::loop_tx_rx_usrp, this, i);
        else
            socket_std_threads_[i]
                = std::thread(&PacketTXRX::loop_tx_rx, this, i);
    }

    if (kUseArgos || kUseUHD)
        radioconfig_->go();
    return true;
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

void PacketTXRX::loop_tx_rx(int tid)
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
    send_beacon(tid,
        tx_frame_id++); // Send Beacons for the first time to kick off sim
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
        if (-1 != dequeue_send(tid))
            continue;
        // receive data
        struct Packet* pkt = recv_enqueue(tid, radio_id, rx_offset);
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
}

struct Packet* PacketTXRX::recv_enqueue(int tid, int radio_id, int rx_offset)
{
    moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];
    char* rx_buffer = (*buffer_)[tid];
    int* rx_buffer_status = (*buffer_status_)[tid];
    int packet_length = cfg->packet_length;

    // if rx_buffer is full, exit
    if (rx_buffer_status[rx_offset] == 1) {
        std::printf("TXRX thread %d rx_buffer full, offset: %d\n", tid, rx_offset);
        cfg->running = false;
        return (NULL);
    }
    struct Packet* pkt = (struct Packet*)&rx_buffer[rx_offset * packet_length];
    if (-1 == recv(socket_[radio_id], (char*)pkt, packet_length, 0)) {
        if (errno != EAGAIN && cfg->running) {
            perror("recv failed");
            std::exit(0);
        }
        return (NULL);
    }
    if (kDebugPrintInTask) {
        std::printf("In TXRX thread %d: Received frame %d, symbol %d, ant %d\n", tid,
            pkt->frame_id, pkt->symbol_id, pkt->ant_id);
    }
    if (kDebugMulticell) {
        std::printf("Before packet combining: receiving data stream from the "
               "antenna %d in cell %d,\n",
            pkt->ant_id, pkt->cell_id);
    }
    pkt->ant_id += pkt->cell_id * ant_per_cell;
    if (kDebugMulticell) {
        std::printf("After packet combining: the combined antenna ID is %d, it "
               "comes from the cell %d\n",
            pkt->ant_id, pkt->cell_id);
    }

    // get the position in rx_buffer
    // move ptr & set status to full
    rx_buffer_status[rx_offset] = 1;

    // Push kPacketRX event into the queue.
    Event_data rx_message(EventType::kPacketRX, rx_tag_t(tid, rx_offset)._tag);
    if (!message_queue_->enqueue(*local_ptok, rx_message)) {
        std::printf("socket message enqueue failed\n");
        std::exit(0);
    }
    return pkt;
}

int PacketTXRX::dequeue_send(int tid)
{
    auto& c = cfg;
    Event_data event;
    if (!task_queue_->try_dequeue_from_producer(*tx_ptoks_[tid], event))
        return -1;

    // std::printf("tx queue length: %d\n", task_queue_->size_approx());
    assert(event.event_type == EventType::kPacketTX);

    size_t ant_id = gen_tag_t(event.tags[0]).ant_id;
    size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
    size_t symbol_id = gen_tag_t(event.tags[0]).symbol_id;

    size_t data_symbol_idx_dl = cfg->get_dl_symbol_idx(frame_id, symbol_id);
    size_t offset
        = (c->get_total_data_symbol_idx_dl(frame_id, data_symbol_idx_dl)
              * c->bs_ant_num())
        + ant_id;

    if (kDebugPrintInTask) {
        std::printf("In TXRX thread %d: Transmitted frame %zu, symbol %zu, "
               "ant %zu, tag %zu, offset: %zu, msg_queue_length: %zu\n",
            tid, frame_id, symbol_id, ant_id, gen_tag_t(event.tags[0])._tag,
            offset, message_queue_->size_approx());
    }

    char* cur_buffer_ptr = tx_buffer_ + offset * c->dl_packet_length;
    auto* pkt = (Packet*)cur_buffer_ptr;
    new (pkt) Packet(frame_id, symbol_id, 0 /* cell_id */, ant_id);

    // Send data (one OFDM symbol)
    ssize_t ret = sendto(socket_[ant_id], cur_buffer_ptr, c->dl_packet_length,
        0, (struct sockaddr*)&bs_rru_sockaddr_[ant_id],
        sizeof(bs_rru_sockaddr_[ant_id]));
    rt_assert(ret > 0, "sendto() failed");

    rt_assert(message_queue_->enqueue(*rx_ptoks_[tid],
                  Event_data(EventType::kPacketTX, event.tags[0])),
        "Socket message enqueue failed\n");
    return event.tags[0];
}
