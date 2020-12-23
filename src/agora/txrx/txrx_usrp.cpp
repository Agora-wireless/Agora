/**
 * @file txrx_usrp.cpp
 * @brief Implementation of PacketTXRX datapath functions for communicating 
 * with USRP hardware
 */

#include "txrx.hpp"

static constexpr bool kDebugDownlink = false;

void PacketTXRX::loop_tx_rx_usrp(int tid)
{
    pin_to_core_with_offset(ThreadType::kWorkerTXRX, core_offset, tid);
    size_t* rx_frame_start = (*frame_start_)[tid];
    int rx_offset = 0;
    int radio_lo = tid * cfg->nRadios / socket_thread_num;
    int radio_hi = (tid + 1) * cfg->nRadios / socket_thread_num;
    std::printf("receiver thread %d has %d radios\n", tid, radio_hi - radio_lo);

    // prepare BS beacon in host buffer
    std::vector<void*> beaconbuff(2);
    std::vector<void*> zeros(2);
    zeros[0] = calloc(cfg->sampsPerSymbol, sizeof(int16_t) * 2);
    zeros[1] = calloc(cfg->sampsPerSymbol, sizeof(int16_t) * 2);
    beaconbuff[0] = cfg->beacon_ci16().data();
    beaconbuff[1] = zeros[0];

    std::vector<std::complex<int16_t>> samp_buffer0(
        cfg->sampsPerSymbol * 14, 0);
    std::vector<std::complex<int16_t>> samp_buffer1(
        cfg->sampsPerSymbol * 14, 0);
    std::vector<void*> samp_buffer(2);
    samp_buffer[0] = samp_buffer0.data();
    if (true)
        samp_buffer[1] = samp_buffer1.data();

    // long long rxTimeBs(0);
    // long long txTimeBs(0);
    rxTimeBs = 0;
    txTimeBs = 0;

    std::cout << "Sync BS host and FGPA timestamp..." << std::endl;
    radioconfig_->radioRx(0, samp_buffer.data(), rxTimeBs);
    // Schedule the first beacon in the future
    txTimeBs = rxTimeBs + cfg->sampsPerSymbol * cfg->frame().NumTotalSyms() * 40;
    radioconfig_->radioTx(0, beaconbuff.data(), 2, txTimeBs);
    long long bsInitRxOffset = txTimeBs - rxTimeBs;
    for (int it = 0; it < std::floor(bsInitRxOffset / cfg->sampsPerSymbol);
         it++) {
        radioconfig_->radioRx(0, samp_buffer.data(), rxTimeBs);
    }

    std::cout << std::endl;
    std::cout << "Init BS sync done..." << std::endl;
    std::cout << "Start BS main recv loop..." << std::endl;

    int global_frame_id = 0;
    int global_symbol_id = 0;

    int prev_frame_id = -1;
    int radio_id = radio_lo;
    while (cfg->running() == true) {

        // transmit data
        // if (-1 != dequeue_send_usrp(tid))
        //   continue;
        // receive data
        // struct Packet* pkt = recv_enqueue_usrp(tid, radio_id, rx_offset);
        struct Packet* pkt = recv_enqueue_usrp(
            tid, radio_id, rx_offset, global_frame_id, global_symbol_id);

        // Schedule beacon in the future
        if (global_symbol_id == 0) {
            txTimeBs = rxTimeBs
                + cfg->sampsPerSymbol * cfg->frame().NumTotalSyms() * 20;
            int tx_ret
                = radioconfig_->radioTx(0, beaconbuff.data(), 2, txTimeBs);
            if (tx_ret != (int)cfg->sampsPerSymbol)
                std::cerr << "BAD Transmit(" << tx_ret << "/"
                          << cfg->sampsPerSymbol << ") at Time " << txTimeBs
                          << ", frame count " << global_frame_id << std::endl;
        }

        if (++radio_id == radio_hi)
            radio_id = radio_lo;

        // Update global frame_id and symbol_id
        global_symbol_id++;
        if (global_symbol_id == (int)cfg->frame().NumTotalSyms()) {
            global_symbol_id = 0;
            global_frame_id++;
        }

        if (pkt == NULL)
            continue;
        rx_offset = (rx_offset + cfg->nChannels) % packet_num_in_buffer_;

        if (kIsWorkerTimingEnabled) {
            int frame_id = pkt->frame_id;
            // int symbol_id = pkt->symbol_id;
            if (frame_id > prev_frame_id) {
                rx_frame_start[frame_id % kNumStatsFrames] = rdtsc();
                prev_frame_id = frame_id;
            }
        }
    }
}

struct Packet* PacketTXRX::recv_enqueue_usrp(
    int tid, int radio_id, int rx_offset, int frame_id, int symbol_id)
{
    moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];
    char* rx_buffer = (*buffer_)[tid];
    int* rx_buffer_status = (*buffer_status_)[tid];
    int packet_length = cfg->packet_length;

    // init samp_buffer for dummy read
    std::vector<std::complex<int16_t>> samp_buffer0(
        cfg->sampsPerSymbol * cfg->frame().NumTotalSyms(), 0);
    std::vector<std::complex<int16_t>> samp_buffer1(
        cfg->sampsPerSymbol * cfg->frame().NumTotalSyms(), 0);
    std::vector<void*> samp_buffer(2);
    samp_buffer[0] = samp_buffer0.data();
    if (cfg->nChannels == 2)
        samp_buffer[1] = samp_buffer1.data();

    // if rx_buffer is full, exit
    int nChannels = cfg->nChannels;
    struct Packet* pkt[nChannels];
    void* samp[nChannels];
    for (int ch = 0; ch < nChannels; ++ch) {
        // if rx_buffer is full, exit
        if (rx_buffer_status[rx_offset + ch] == 1) {
            std::printf("Receive thread %d rx_buffer full, offset: %d\n", tid,
                rx_offset);
            cfg->running( false );
            break;
        }
        pkt[ch] = (struct Packet*)&rx_buffer[(rx_offset + ch) * packet_length];
        samp[ch] = pkt[ch]->data;
    }

    int tmp_ret;

    if (cfg->IsPilot(frame_id, symbol_id)
        || cfg->IsUplink(frame_id, symbol_id)) {
        tmp_ret = radioconfig_->radioRx(radio_id, samp, rxTimeBs);
    } else {
        tmp_ret = radioconfig_->radioRx(radio_id, samp_buffer.data(), rxTimeBs);
    }

    if ((cfg->running() == false) || tmp_ret <= 0
        || (!cfg->IsPilot(frame_id, symbol_id)
               && !cfg->IsUplink(frame_id, symbol_id))) {
        return NULL;
    }

    int ant_id = radio_id * nChannels;
    if (cfg->IsPilot(frame_id, symbol_id)
        || cfg->IsUplink(frame_id, symbol_id)) {
        for (int ch = 0; ch < nChannels; ++ch) {
            new (pkt[ch]) Packet(frame_id, symbol_id, 0, ant_id + ch);
            // move ptr & set status to full
            rx_buffer_status[rx_offset + ch]
                = 1; // has data, after it is read, it is set to 0

            // Push kPacketRX event into the queue
            Event_data rx_message(
                EventType::kPacketRX, rx_tag_t(tid, rx_offset + ch)._tag);

            if (!message_queue_->enqueue(*local_ptok, rx_message)) {
                std::printf("socket message enqueue failed\n");
                std::exit(0);
            }
        }
    }

    return pkt[0];
}

int PacketTXRX::dequeue_send_usrp(int tid)
{
    auto& c = cfg;
    Event_data event;
    if (!task_queue_->try_dequeue_from_producer(*tx_ptoks_[tid], event))
        return -1;

    std::printf("tx queue length: %zu\n", task_queue_->size_approx());
    assert(event.event_type == EventType::kPacketTX);

    size_t ant_id = gen_tag_t(event.tags[0]).ant_id;
    size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
    size_t symbol_id = gen_tag_t(event.tags[0]).symbol_id;

    size_t offset
        = (c->get_total_data_symbol_idx(frame_id, symbol_id) * c->bs_ant_num())
        + ant_id;

    symbol_id += c->ue_ant_num();
    frame_id += TX_FRAME_DELTA;

    void* txbuf[2];
    int nChannels = c->nChannels;
    int ch = ant_id % nChannels;

    if (kDebugDownlink == true) {
        std::vector<std::complex<int16_t>> zeros(c->sampsPerSymbol);
        size_t dl_symbol_idx = c->GetDLSymbolIdx(frame_id, symbol_id);
        if (ant_id != c->ref_ant) {
            txbuf[ch] = zeros.data();
        }
        else if (dl_symbol_idx < c->frame().client_dl_pilot_symbols()) {
            txbuf[ch] = reinterpret_cast<void*>(c->ue_specific_pilot_t()[0]);
        }
        else {
            txbuf[ch] = reinterpret_cast<void*>(c->dl_iq_t()[dl_symbol_idx - c->frame().client_dl_pilot_symbols()]);
        }
    } else {
        char* cur_buffer_ptr = tx_buffer_ + offset * c->packet_length;
        struct Packet* pkt = (struct Packet*)cur_buffer_ptr;
        txbuf[ch] = reinterpret_cast<void*>(pkt->data);
    }

    size_t last = c->frame().GetDLSymbolLast();
    int flags = (symbol_id != last) ? 1 // HAS_TIME
                                    : 2; // HAS_TIME & END_BURST, fixme
    long long frameTime = ((long long)frame_id << 32) | (symbol_id << 16);
    radioconfig_->radioTx(ant_id / nChannels, txbuf, flags, frameTime);

    if (kDebugPrintInTask == true) {
        std::printf("In TX thread %d: Transmitted frame %zu, symbol %zu, "
               "ant %zu, offset: %zu, msg_queue_length: %zu\n",
            tid, frame_id, symbol_id, ant_id, offset,
            message_queue_->size_approx());
    }

    rt_assert(message_queue_->enqueue(*rx_ptoks_[tid],
                  Event_data(EventType::kPacketTX, event.tags[0])),
        "Socket message enqueue failed\n");
    return event.tags[0];
}

int PacketTXRX::dequeue_send_usrp(int tid, int frame_id, int symbol_id)
{
    auto& c = cfg;
    Event_data event;
    if (!task_queue_->try_dequeue_from_producer(*tx_ptoks_[tid], event))
        return -1;
    std::cout << "DDD" << std::endl;

    std::printf("tx queue length: %zu\n", task_queue_->size_approx());
    assert(event.event_type == EventType::kPacketTX);

    size_t ant_id = gen_tag_t(event.tags[0]).ant_id;
    // size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
    // size_t symbol_id = gen_tag_t(event.tags[0]).symbol_id;

    size_t offset
        = (c->get_total_data_symbol_idx(frame_id, symbol_id) * c->bs_ant_num())
        + ant_id;

    // symbol_id += c->ue_ant_num();
    // frame_id += TX_FRAME_DELTA;

    std::cout << "ant_id: " << ant_id << ", frame_id: " << frame_id
              << ", symbol_id: " << symbol_id;
    std::cout << "offset: " << offset << std::endl;

    void* txbuf[2];
    int nChannels = c->nChannels;
    int ch = ant_id % nChannels;

    if (kDebugDownlink) {
        std::vector<std::complex<int16_t>> zeros(c->sampsPerSymbol);
        size_t dl_symbol_idx = c->GetDLSymbolIdx(frame_id, symbol_id);
        if (ant_id != c->ref_ant)
            txbuf[ch] = zeros.data();
        else if (dl_symbol_idx < c->frame().client_dl_pilot_symbols())
            txbuf[ch] = reinterpret_cast <void*>(c->ue_specific_pilot_t()[0]);
        else
            txbuf[ch] = reinterpret_cast<void*>(c->dl_iq_t()[dl_symbol_idx - c->frame().client_dl_pilot_symbols()]);
    } else {
        char* cur_buffer_ptr = tx_buffer_ + offset * c->packet_length;
        struct Packet* pkt = reinterpret_cast<struct Packet*>(cur_buffer_ptr);
        txbuf[ch] = reinterpret_cast<void*>(pkt->data);
    }

    size_t last = c->frame().GetDLSymbolLast();
    int flags = (symbol_id != static_cast<int>(last)) ? 1 // HAS_TIME
                                         : 2; // HAS_TIME & END_BURST, fixme
    long long frameTime = ((long long)frame_id << 32) | (symbol_id << 16);
    radioconfig_->radioTx(ant_id / nChannels, txbuf, flags, frameTime);

    if (kDebugPrintInTask) {
        std::printf("In TX thread %d: Transmitted frame %d, symbol %d, "
               "ant %zu, offset: %zu, msg_queue_length: %zu\n",
            tid, frame_id, symbol_id, ant_id, offset,
            message_queue_->size_approx());
    }

    rt_assert(message_queue_->enqueue(*rx_ptoks_[tid],
                  Event_data(EventType::kPacketTX, event.tags[0])),
        "Socket message enqueue failed\n");
    return event.tags[0];
}
