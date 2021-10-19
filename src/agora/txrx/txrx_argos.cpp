/**
 * @file txrx_argos.cpp
 * @brief Implementation of PacketTXRX datapath functions for communicating 
 * with real Argos hardware
 */

#include "txrx.hpp"

static constexpr bool kDebugDownlink = false;

void* PacketTXRX::loop_tx_rx_argos(int tid)
{
    pin_to_core_with_offset(ThreadType::kWorkerTXRX, core_offset, tid);
    size_t* rx_frame_start = (*frame_start_)[tid];
    int rx_offset = 0;
    int radio_lo = tid * cfg->nRadios / socket_thread_num;
    int radio_hi = (tid + 1) * cfg->nRadios / socket_thread_num;
    printf("TXRX thread %d has %d radios\n", tid, radio_hi - radio_lo);

    int prev_frame_id = -1;
    int radio_id = radio_lo;
    while (cfg->running) {
        if (-1 != dequeue_send_argos(tid))
            continue;
        // receive data
        struct Packet* pkt = recv_enqueue_argos(tid, radio_id, rx_offset);
        if (pkt == NULL)
            continue;
        rx_offset = (rx_offset + cfg->nChannels) % packet_num_in_buffer_;

        if (kIsWorkerTimingEnabled) {
            int frame_id = pkt->frame_id_;
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

struct Packet* PacketTXRX::recv_enqueue_argos(
    int tid, int radio_id, int rx_offset)
{
    moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];
    char* rx_buffer = (*buffer_)[tid];
    int* rx_buffer_status = (*buffer_status_)[tid];
    int packet_length = cfg->packet_length;

    // if rx_buffer is full, exit
    int nChannels = cfg->nChannels;
    struct Packet* pkt[nChannels];
    void* samp[nChannels];
    for (int ch = 0; ch < nChannels; ++ch) {
        // if rx_buffer is full, exit
        if (rx_buffer_status[rx_offset + ch] == 1) {
            printf(
                "TXRX thread %d rx_buffer full, offset: %d\n", tid, rx_offset);
            cfg->running = false;
            break;
        }
        pkt[ch] = (struct Packet*)&rx_buffer[(rx_offset + ch) * packet_length];
        samp[ch] = pkt[ch]->data_;
    }

    long long frameTime;
    if (!cfg->running
        || radioconfig_->radioRx(radio_id, samp, frameTime) <= 0) {
        return NULL;
    }

    int frame_id = (int)(frameTime >> 32);
    int symbol_id = (int)((frameTime >> 16) & 0xFFFF);
    int ant_id = radio_id * nChannels;
    for (int ch = 0; ch < nChannels; ++ch) {
        new (pkt[ch]) Packet(frame_id, symbol_id, 0 /* cell_id */, ant_id + ch);
        // move ptr & set status to full
        rx_buffer_status[rx_offset + ch]
            = 1; // has data, after it is read, it is set to 0

        // Push kPacketRX event into the queue.
        EventData rx_message(
            EventType::kPacketRX, rx_tag_t(tid, rx_offset + ch)._tag);

        if (!message_queue_->enqueue(*local_ptok, rx_message)) {
            printf("socket message enqueue failed\n");
            exit(0);
        }
    }
    return pkt[0];
}

int PacketTXRX::dequeue_send_argos(int tid)
{
    auto& c = cfg;
    EventData event;
    if (!task_queue_->try_dequeue_from_producer(*tx_ptoks_[tid], event))
        return -1;

    // printf("tx queue length: %d\n", task_queue_->size_approx());
    assert(event.event_type_ == EventType::kPacketTX);

    size_t ant_id = gen_tag_t(event.tags_[0]).ant_id;
    size_t frame_id = gen_tag_t(event.tags_[0]).frame_id;
    size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id;

    size_t data_symbol_idx_dl = cfg->get_dl_symbol_idx(frame_id, symbol_id);
    size_t offset
        = (c->get_total_data_symbol_idx_dl(frame_id, data_symbol_idx_dl)
              * c->BS_ANT_NUM)
        + ant_id;

    symbol_id += c->UE_ANT_NUM;
    frame_id += TX_FRAME_DELTA;

    void* txbuf[2];
    int nChannels = c->nChannels;
    int ch = ant_id % nChannels;

    if (kDebugDownlink) {
        std::vector<std::complex<int16_t>> zeros(c->sampsPerSymbol);
        size_t dl_symbol_idx = c->get_dl_symbol_idx(frame_id, symbol_id);
        if (ant_id != c->ref_ant)
            txbuf[ch] = zeros.data();
        else if (dl_symbol_idx < c->DL_PILOT_SYMS)
            txbuf[ch] = (void*)c->ue_specific_pilot_t[0];
        else
            txbuf[ch] = (void*)c->dl_iq_t[dl_symbol_idx - c->DL_PILOT_SYMS];
    } else {
        char* cur_buffer_ptr = tx_buffer_ + offset * c->packet_length;
        struct Packet* pkt = (struct Packet*)cur_buffer_ptr;
        txbuf[ch] = (void*)pkt->data_;
    }

    size_t last = c->DLSymbols[0].back();
    int flags = (symbol_id != last) ? 1 // HAS_TIME
                                    : 2; // HAS_TIME & END_BURST, fixme
    long long frameTime = ((long long)frame_id << 32) | (symbol_id << 16);
    radioconfig_->radioTx(ant_id / nChannels, txbuf, flags, frameTime);

    if (kDebugBSSender) {
        printf("In TX thread %d: Transmitted frame %zu, symbol %zu, "
               "ant %zu, offset: %zu, msg_queue_length: %zu\n",
            tid, frame_id, symbol_id, ant_id, offset,
            message_queue_->size_approx());
    }

    rt_assert(message_queue_->enqueue(*rx_ptoks_[tid],
                  EventData(EventType::kPacketTX, event.tags_[0])),
        "Socket message enqueue failed\n");
    return event.tags_[0];
}
