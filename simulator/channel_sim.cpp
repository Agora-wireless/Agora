#include "channel_sim.hpp"
#include "datatype_conversion.h"

static bool running = true;
static constexpr bool kPrintChannelOutput = false;

static void simd_convert_float_to_short(
    float* in_buf, short* out_buf, size_t length)
{
    /*
    for (size_t i = 0; i < length; i += 16) {
        __m256 data1 = _mm256_load_ps(in_buf + i);
        __m256 data2 = _mm256_load_ps(in_buf + i + 8);
        __m256i integer1 = _mm256_cvtps_epi32(data1);
        __m256i integer2 = _mm256_cvtps_epi32(data2);
        integer1 = _mm256_packs_epi32(integer1, integer2);
        integer1 = _mm256_permute4x64_epi64(integer1, 0xD8);
        _mm256_stream_si256(
            (__m256i*)&out_buf[i], integer1);
    }
    for (size_t i = length / 16; i < length; i++) {
        out_buf[i] = (short)(in_buf[i] * 32768.f);
    }
    */
    for (size_t i = 0; i < length; i++) {
        out_buf[i] = (short)(in_buf[i] * 32768.f);
    }
}

static inline void print_mat(cx_fmat c)
{
    std::stringstream so;
    for (size_t i = 0; i < c.n_cols; i++) {
        so << "row" << i << " = [";
        for (size_t j = 0; j < c.n_rows; j++)
            so << std::fixed << std::setw(5) << std::setprecision(3)
               << c.at(j, i).real() << "+" << c.at(j, i).imag() << "i ";
        so << "];\n";
    }
    so << std::endl;
    std::cout << so.str();
}

ChannelSim::ChannelSim(Config* config_bs, Config* config_ue,
    size_t bs_thread_num, size_t user_thread_num, size_t worker_thread_num,
    size_t in_core_offset)
    : bscfg(config_bs)
    , uecfg(config_ue)
    , bs_thread_num(bs_thread_num)
    , user_thread_num(user_thread_num)
    , bs_socket_num(config_bs->BS_ANT_NUM)
    , user_socket_num(config_ue->UE_ANT_NUM)
    , worker_thread_num(worker_thread_num)
    , core_offset(in_core_offset)
{

    // initialize parameters from config
    srand(time(NULL));
    dl_data_plus_beacon_symbols
        = bscfg->dl_data_symbol_num_perframe + 1; // plus beacon
    ul_data_plus_pilot_symbols
        = bscfg->ul_data_symbol_num_perframe + bscfg->pilot_symbol_num_perframe;

    socket_bs_.resize(bs_socket_num);
    servaddr_bs_.resize(bs_socket_num);
    socket_ue_.resize(user_socket_num);
    servaddr_ue_.resize(user_socket_num);

    task_queue_bs
        = moodycamel::ConcurrentQueue<EventData>(TASK_BUFFER_FRAME_NUM
            * dl_data_plus_beacon_symbols * bscfg->BS_ANT_NUM * 36);
    task_queue_user
        = moodycamel::ConcurrentQueue<EventData>(TASK_BUFFER_FRAME_NUM
            * ul_data_plus_pilot_symbols * uecfg->UE_ANT_NUM * 36);
    message_queue_ = moodycamel::ConcurrentQueue<EventData>(
        TASK_BUFFER_FRAME_NUM * bscfg->symbol_num_perframe
        * (bscfg->BS_ANT_NUM + uecfg->UE_ANT_NUM) * 36);

    assert(bscfg->packet_length == uecfg->packet_length);
    payload_length = bscfg->packet_length - Packet::kOffsetOfData;

    // initialize bs-facing and client-facing data buffers
    size_t tx_buffer_ue_size = TASK_BUFFER_FRAME_NUM
        * dl_data_plus_beacon_symbols * uecfg->UE_ANT_NUM * payload_length;
    tx_buffer_ue.resize(tx_buffer_ue_size);

    size_t tx_buffer_bs_size = TASK_BUFFER_FRAME_NUM
        * ul_data_plus_pilot_symbols * bscfg->BS_ANT_NUM * payload_length;
    tx_buffer_bs.resize(tx_buffer_bs_size);

    size_t rx_buffer_ue_size = TASK_BUFFER_FRAME_NUM
        * ul_data_plus_pilot_symbols * uecfg->UE_ANT_NUM * payload_length;
    rx_buffer_ue.resize(rx_buffer_ue_size);

    size_t rx_buffer_bs_size = TASK_BUFFER_FRAME_NUM
        * dl_data_plus_beacon_symbols * bscfg->BS_ANT_NUM * payload_length;
    rx_buffer_bs.resize(rx_buffer_bs_size);

    // initilize rx and tx counters
    bs_rx_counter_
        = new size_t[dl_data_plus_beacon_symbols * TASK_BUFFER_FRAME_NUM];
    memset(bs_rx_counter_, 0,
        sizeof(size_t) * dl_data_plus_beacon_symbols * TASK_BUFFER_FRAME_NUM);

    user_rx_counter_
        = new size_t[ul_data_plus_pilot_symbols * TASK_BUFFER_FRAME_NUM];
    memset(user_rx_counter_, 0,
        sizeof(size_t) * ul_data_plus_pilot_symbols * TASK_BUFFER_FRAME_NUM);

    memset(bs_tx_counter_, 0, sizeof(size_t) * TASK_BUFFER_FRAME_NUM);
    memset(user_tx_counter_, 0, sizeof(size_t) * TASK_BUFFER_FRAME_NUM);

    // initialize channel as random matrix of size uecfg->UE_ANT_NUM * bscfg->BS_ANT_NUM
    cx_fmat H(randn<fmat>(uecfg->UE_ANT_NUM, bscfg->BS_ANT_NUM),
        randn<fmat>(uecfg->UE_ANT_NUM, bscfg->BS_ANT_NUM));
    channel = H / abs(H).max();

    for (size_t i = 0; i < worker_thread_num; i++) {
        task_ptok[i] = new moodycamel::ProducerToken(message_queue_);
    }
    alloc_buffer_1d(&task_threads, worker_thread_num, 64, 0);

    // create task threads (transmit to base station and client antennas)
    for (size_t i = 0; i < worker_thread_num; i++) {
        auto context = new EventHandlerContext<ChannelSim>;
        context->obj_ptr = this;
        context->id = i;
        if (pthread_create(&task_threads[i], NULL,
                pthread_fun_wrapper<ChannelSim, &ChannelSim::taskThread>,
                context)
            != 0) {
            perror("task thread create failed");
            exit(0);
        }
    }
}

ChannelSim::~ChannelSim()
{
    // delete buffers, UDP client and servers
    //delete[] socket_uerx_;
    //delete[] socket_bsrx_;
}

void ChannelSim::schedule_task(EventData do_task,
    moodycamel::ConcurrentQueue<EventData>* in_queue,
    moodycamel::ProducerToken const& ptok)
{
    if (!in_queue->try_enqueue(ptok, do_task)) {
        printf("need more memory\n");
        if (!in_queue->enqueue(ptok, do_task)) {
            printf("task enqueue failed\n");
            exit(0);
        }
    }
}

void ChannelSim::start()
{
    printf("Starting Channel Simulator ...\n");
    pin_to_core_with_offset(ThreadType::kMaster, core_offset, 0);

    moodycamel::ProducerToken ptok_bs(task_queue_bs);
    moodycamel::ProducerToken ptok_user(task_queue_user);
    moodycamel::ConsumerToken ctok(message_queue_);

    for (size_t i = 0; i < bs_thread_num; i++) {
        pthread_t recv_thread_bs;

        auto bs_context = new EventHandlerContext<ChannelSim>;
        bs_context->obj_ptr = this;
        bs_context->id = i;

        int ret = pthread_create(&recv_thread_bs, NULL,
            pthread_fun_wrapper<ChannelSim, &ChannelSim::bs_rx_loop>,
            bs_context);
        rt_assert(ret == 0, "Failed to create BS recv thread!");
    }

    for (size_t i = 0; i < user_thread_num; i++) {
        pthread_t recv_thread_ue;

        auto ue_context = new EventHandlerContext<ChannelSim>;
        ue_context->obj_ptr = this;
        ue_context->id = i;

        int ret = pthread_create(&recv_thread_ue, NULL,
            pthread_fun_wrapper<ChannelSim, &ChannelSim::ue_rx_loop>,
            ue_context);
        rt_assert(ret == 0, "Failed to create UE recv thread!");
    }

    sleep(1);

    int ret = 0;

    static constexpr size_t kDequeueBulkSize = 5;
    EventData events_list[kDequeueBulkSize];
    while (true) {
        ret = message_queue_.try_dequeue_bulk(
            ctok, events_list, kDequeueBulkSize);

        for (int bulk_count = 0; bulk_count < ret; bulk_count++) {
            EventData& event = events_list[bulk_count];

            switch (event.event_type_) {

            case EventType::kPacketRX: {
                size_t frame_id = gen_tag_t(event.tags_[0]).frame_id;
                size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id;
                // received a packet from a client antenna
                if (gen_tag_t(event.tags_[0]).tag_type
                    == gen_tag_t::TagType::kUsers) {
                    size_t pilot_symbol_id
                        = uecfg->get_pilot_symbol_idx(frame_id, symbol_id);
                    size_t ul_symbol_id
                        = uecfg->get_ul_symbol_idx(frame_id, symbol_id);
                    size_t total_symbol_id = pilot_symbol_id;
                    if (pilot_symbol_id == SIZE_MAX)
                        total_symbol_id
                            = ul_symbol_id + bscfg->pilot_symbol_num_perframe;
                    size_t frame_offset = (frame_id % TASK_BUFFER_FRAME_NUM)
                            * ul_data_plus_pilot_symbols
                        + total_symbol_id;
                    user_rx_counter_[frame_offset]++;
                    // when received all client antennas on this symbol, kick-off BS TX
                    if (user_rx_counter_[frame_offset] == uecfg->UE_ANT_NUM) {
                        user_rx_counter_[frame_offset] = 0;
                        if (kDebugPrintPerSymbolDone)
                            printf(
                                "Scheduling uplink transmission of frame %zu, "
                                "symbol %zu, from %zu "
                                "user to %zu BS antennas\n",
                                frame_id, symbol_id, uecfg->UE_ANT_NUM,
                                bscfg->BS_ANT_NUM);
                        schedule_task(
                            EventData(EventType::kPacketTX, event.tags_[0]),
                            &task_queue_bs, ptok_bs);
                    }
                    // received a packet from a BS antenna
                } else if (gen_tag_t(event.tags_[0]).tag_type
                    == gen_tag_t::TagType::kAntennas) {
                    size_t dl_symbol_id
                        = get_dl_symbol_idx(frame_id, symbol_id);
                    size_t frame_offset = (frame_id % TASK_BUFFER_FRAME_NUM)
                            * dl_data_plus_beacon_symbols
                        + dl_symbol_id;
                    bs_rx_counter_[frame_offset]++;

                    // when received all BS antennas on this symbol, kick-off client TX
                    if (bs_rx_counter_[frame_offset] == bscfg->BS_ANT_NUM) {
                        bs_rx_counter_[frame_offset] = 0;
                        if (kDebugPrintPerSymbolDone)
                            printf("Scheduling downlink transmission in frame "
                                   "%zu, "
                                   "symbol %zu, from %zu "
                                   "BS to %zu user antennas\n",
                                frame_id, symbol_id, bscfg->BS_ANT_NUM,
                                uecfg->UE_ANT_NUM);
                        schedule_task(
                            EventData(EventType::kPacketTX, event.tags_[0]),
                            &task_queue_user, ptok_user);
                    }
                }
            } break;

            case EventType::kPacketTX: {
                size_t frame_id = gen_tag_t(event.tags_[0]).frame_id;
                size_t offset = frame_id % TASK_BUFFER_FRAME_NUM;
                if (gen_tag_t(event.tags_[0]).tag_type
                    == gen_tag_t::TagType::kUsers) {
                    user_tx_counter_[offset]++;
                    if (user_tx_counter_[offset]
                        == dl_data_plus_beacon_symbols) {
                        if (kDebugPrintPerFrameDone)
                            printf("Finished downlink transmission %zu symbols "
                                   "in frame %zu\n",
                                dl_data_plus_beacon_symbols, frame_id);
                        user_tx_counter_[offset] = 0;
                    }
                } else if (gen_tag_t(event.tags_[0]).tag_type
                    == gen_tag_t::TagType::kAntennas) {
                    bs_tx_counter_[offset]++;
                    if (bs_tx_counter_[offset] == ul_data_plus_pilot_symbols) {
                        if (kDebugPrintPerFrameDone)
                            printf("Finished uplink transmission of %zu "
                                   "symbols in frame %zu\n",
                                ul_data_plus_pilot_symbols, frame_id);
                        bs_tx_counter_[offset] = 0;
                    }
                }
            } break;
            default:
                std::cout << "Invalid Event Type!" << std::endl;
                break;
            }
        }
    }
}

void* ChannelSim::taskThread(int tid)
{

    pin_to_core_with_offset(ThreadType::kWorker,
        core_offset + bs_thread_num + 1 + user_thread_num, tid);

    EventData event;
    while (running) {
        if (task_queue_bs.try_dequeue(event))
            do_tx_bs(tid, event.tags_[0]);
        else if (task_queue_user.try_dequeue(event))
            do_tx_user(tid, event.tags_[0]);
    }
    return 0;
}

void* ChannelSim::bs_rx_loop(int tid)
{
    size_t socket_lo = tid * bs_socket_num / bs_thread_num;
    size_t socket_hi = (tid + 1) * bs_socket_num / bs_thread_num;

    moodycamel::ProducerToken local_ptok(message_queue_);
    pin_to_core_with_offset(ThreadType::kWorkerTXRX, core_offset + 1, tid);

    // initialize bs-facing sockets
    int sock_buf_size = 1024 * 1024 * 64 * 8 - 1;
    for (size_t socket_id = socket_lo; socket_id < socket_hi; ++socket_id) {
        int local_port_id = bscfg->bs_rru_port + socket_id;
        socket_bs_[socket_id]
            = setup_socket_ipv4(local_port_id, true, sock_buf_size);
        setup_sockaddr_remote_ipv4(&servaddr_bs_[socket_id],
            bscfg->bs_server_port + socket_id, bscfg->bs_server_addr.c_str());
        printf("BS RX thread %d: set up UDP socket server listening to port %d"
               " with remote address %s:%zu\n",
            tid, local_port_id, bscfg->bs_server_addr.c_str(),
            bscfg->bs_server_port + socket_id);
        fcntl(socket_bs_[socket_id], F_SETFL, O_NONBLOCK);
    }

    std::vector<uint8_t> udp_pkt_buf(bscfg->packet_length, 0);
    size_t socket_id = socket_lo;
    while (running) {
        if (-1
            == recv(socket_bs_[socket_id], (char*)udp_pkt_buf.data(),
                   udp_pkt_buf.size(), 0)) {
            if (errno != EAGAIN && running) {
                printf("BS socket %zu receive failed\n", socket_id);
                exit(0);
            }
            continue;
        }
        const auto* pkt = reinterpret_cast<Packet*>(&udp_pkt_buf[0]);

        size_t frame_id = pkt->frame_id_;
        size_t symbol_id = pkt->symbol_id_;
        size_t ant_id = pkt->ant_id_;
        if (kDebugPrintInTask)
            printf("Received BS packet for frame %zu, symbol %zu, ant %zu from "
                   "socket %zu\n",
                frame_id, symbol_id, ant_id, socket_id);
        size_t dl_symbol_id = get_dl_symbol_idx(frame_id, symbol_id);
        size_t symbol_offset
            = (frame_id % TASK_BUFFER_FRAME_NUM) * dl_data_plus_beacon_symbols
            + dl_symbol_id;
        size_t offset = symbol_offset * bscfg->BS_ANT_NUM + ant_id;
        memcpy(
            &rx_buffer_bs[offset * payload_length], pkt->data_, payload_length);

        rt_assert(
            message_queue_.enqueue(local_ptok,
                EventData(EventType::kPacketRX,
                    gen_tag_t::frm_sym_ant(frame_id, symbol_id, ant_id)._tag)),
            "BS socket message enqueue failed!");
        if (++socket_id == socket_hi)
            socket_id = socket_lo;
    }
    return 0;
}

void* ChannelSim::ue_rx_loop(int tid)
{
    size_t socket_lo = tid * user_socket_num / user_thread_num;
    size_t socket_hi = (tid + 1) * user_socket_num / user_thread_num;

    moodycamel::ProducerToken local_ptok(message_queue_);
    pin_to_core_with_offset(
        ThreadType::kWorkerTXRX, core_offset + 1 + bs_thread_num, tid);

    // initialize client-facing sockets
    int sock_buf_size = 1024 * 1024 * 64 * 8 - 1;
    for (size_t socket_id = socket_lo; socket_id < socket_hi; ++socket_id) {
        int local_port_id = uecfg->ue_rru_port + socket_id;
        socket_ue_[socket_id]
            = setup_socket_ipv4(local_port_id, true, sock_buf_size);
        setup_sockaddr_remote_ipv4(&servaddr_ue_[socket_id],
            uecfg->ue_server_port + socket_id, uecfg->ue_server_addr.c_str());
        printf("UE RX thread %d: set up UDP socket server listening to port %d"
               " with remote address %s:%zu\n",
            tid, local_port_id, uecfg->ue_server_addr.c_str(),
            uecfg->ue_server_port + socket_id);
        fcntl(socket_ue_[socket_id], F_SETFL, O_NONBLOCK);
    }

    std::vector<uint8_t> udp_pkt_buf(bscfg->packet_length, 0);
    size_t socket_id = socket_lo;
    while (running) {
        if (-1
            == recv(socket_ue_[socket_id], (char*)&udp_pkt_buf[0],
                   udp_pkt_buf.size(), 0)) {
            if (errno != EAGAIN && running) {
                printf("UE socket %zu receive failed\n", socket_id);
                exit(0);
            }
            continue;
        }

        const auto* pkt = reinterpret_cast<Packet*>(&udp_pkt_buf[0]);

        size_t frame_id = pkt->frame_id_;
        size_t symbol_id = pkt->symbol_id_;
        size_t ant_id = pkt->ant_id_;

        size_t pilot_symbol_id
            = uecfg->get_pilot_symbol_idx(frame_id, symbol_id);
        size_t ul_symbol_id = uecfg->get_ul_symbol_idx(frame_id, symbol_id);
        size_t total_symbol_id = pilot_symbol_id;
        if (pilot_symbol_id == SIZE_MAX)
            total_symbol_id = ul_symbol_id + bscfg->pilot_symbol_num_perframe;
        if (kDebugPrintInTask)
            printf("Received UE packet for frame %zu, symbol %zu, ant %zu from "
                   "socket %zu\n",
                frame_id, symbol_id, ant_id, socket_id);
        size_t symbol_offset
            = (frame_id % TASK_BUFFER_FRAME_NUM) * ul_data_plus_pilot_symbols
            + total_symbol_id;
        size_t offset = symbol_offset * uecfg->UE_ANT_NUM + ant_id;
        memcpy(
            &rx_buffer_ue[offset * payload_length], pkt->data_, payload_length);

        rt_assert(
            message_queue_.enqueue(local_ptok,
                EventData(EventType::kPacketRX,
                    gen_tag_t::frm_sym_ue(frame_id, symbol_id, ant_id)._tag)),
            "UE Socket message enqueue failed!");
        if (++socket_id == socket_hi)
            socket_id = socket_lo;
    }
    return 0;
}

void ChannelSim::do_tx_bs(int tid, size_t tag)
{
    const size_t frame_id = gen_tag_t(tag).frame_id;
    const size_t symbol_id = gen_tag_t(tag).symbol_id;

    size_t pilot_symbol_id = bscfg->get_pilot_symbol_idx(frame_id, symbol_id);
    size_t ul_symbol_id = bscfg->get_ul_symbol_idx(frame_id, symbol_id);
    size_t total_symbol_id = pilot_symbol_id;
    if (pilot_symbol_id == SIZE_MAX)
        total_symbol_id = ul_symbol_id + bscfg->pilot_symbol_num_perframe;

    size_t symbol_offset
        = (frame_id % TASK_BUFFER_FRAME_NUM) * ul_data_plus_pilot_symbols
        + total_symbol_id;
    size_t total_offset_ue = symbol_offset * payload_length * uecfg->UE_ANT_NUM;
    size_t total_offset_bs = symbol_offset * payload_length * bscfg->BS_ANT_NUM;

    auto* src_ptr = reinterpret_cast<short*>(&rx_buffer_ue[total_offset_ue]);

    // convert received data to complex float,
    // apply channel, convert back to complex short to TX
    cx_fmat fmat_src = zeros<cx_fmat>(bscfg->sampsPerSymbol, uecfg->UE_ANT_NUM);
    simd_convert_short_to_float(src_ptr,
        reinterpret_cast<float*>(fmat_src.memptr()),
        2 * bscfg->sampsPerSymbol * uecfg->UE_ANT_NUM);

    cx_fmat fmat_dst = fmat_src * channel;
    // add 30dB SNR noise
    cx_fmat noise(1e-3 * randn<fmat>(uecfg->sampsPerSymbol, bscfg->BS_ANT_NUM),
        1e-3 * randn<fmat>(uecfg->sampsPerSymbol, bscfg->BS_ANT_NUM));
    fmat_dst += noise;
    if (kPrintChannelOutput)
        print_mat(fmat_dst);

    auto* dst_ptr = reinterpret_cast<short*>(&tx_buffer_bs[total_offset_bs]);
    simd_convert_float_to_short(reinterpret_cast<float*>(fmat_dst.memptr()),
        dst_ptr, 2 * bscfg->sampsPerSymbol * bscfg->BS_ANT_NUM);
    std::stringstream ss;

    // send the symbol to all base station antennas
    std::vector<uint8_t> udp_pkt_buf(bscfg->packet_length, 0);
    auto* pkt = reinterpret_cast<Packet*>(&udp_pkt_buf[0]);
    for (size_t ant_id = 0; ant_id < bscfg->BS_ANT_NUM; ant_id++) {
        pkt->frame_id_ = frame_id;
        pkt->symbol_id_ = symbol_id;
        pkt->ant_id_ = ant_id;
        pkt->cell_id_ = 0;
        memcpy(pkt->data_,
            &tx_buffer_bs[total_offset_bs + ant_id * payload_length],
            payload_length);
        ssize_t ret = sendto(socket_bs_[ant_id], (char*)udp_pkt_buf.data(),
            udp_pkt_buf.size(), 0, (struct sockaddr*)&servaddr_bs_[ant_id],
            sizeof(servaddr_bs_[ant_id]));
        rt_assert(ret > 0, "sendto() failed");
    }

    rt_assert(message_queue_.enqueue(*task_ptok[tid],
                  EventData(EventType::kPacketTX,
                      gen_tag_t::frm_sym_ant(frame_id, symbol_id, 0)._tag)),
        "BS TX message enqueue failed!\n");
}

void ChannelSim::do_tx_user(int tid, size_t tag)
{
    size_t frame_id = gen_tag_t(tag).frame_id;
    size_t symbol_id = gen_tag_t(tag).symbol_id;
    size_t dl_symbol_id = get_dl_symbol_idx(frame_id, symbol_id);

    size_t symbol_offset
        = (frame_id % TASK_BUFFER_FRAME_NUM) * dl_data_plus_beacon_symbols
        + dl_symbol_id;
    size_t total_offset_ue = symbol_offset * payload_length * uecfg->UE_ANT_NUM;
    size_t total_offset_bs = symbol_offset * payload_length * bscfg->BS_ANT_NUM;

    auto* src_ptr = reinterpret_cast<short*>(&rx_buffer_bs[total_offset_bs]);

    // convert received data to complex float,
    // apply channel, convert back to complex short to TX
    cx_fmat fmat_src = zeros<cx_fmat>(bscfg->sampsPerSymbol, bscfg->BS_ANT_NUM);
    simd_convert_short_to_float(src_ptr,
        reinterpret_cast<float*>(fmat_src.memptr()),
        2 * bscfg->sampsPerSymbol * bscfg->BS_ANT_NUM);

    cx_fmat fmat_dst = fmat_src * channel.st() / std::sqrt(bscfg->BS_ANT_NUM);
    // add 30dB SNR noise
    cx_fmat noise(1e-3 * randn<fmat>(uecfg->sampsPerSymbol, bscfg->UE_ANT_NUM),
        1e-3 * randn<fmat>(uecfg->sampsPerSymbol, bscfg->UE_ANT_NUM));
    fmat_dst += noise;
    if (kPrintChannelOutput)
        print_mat(fmat_dst);

    auto* dst_ptr = reinterpret_cast<short*>(&tx_buffer_ue[total_offset_ue]);
    simd_convert_float_to_short(reinterpret_cast<float*>(fmat_dst.memptr()),
        dst_ptr, 2 * bscfg->sampsPerSymbol * uecfg->UE_ANT_NUM);

    // send the symbol to all base station antennas
    std::vector<uint8_t> udp_pkt_buf(bscfg->packet_length, 0);
    auto* pkt = reinterpret_cast<Packet*>(&udp_pkt_buf[0]);
    for (size_t ant_id = 0; ant_id < uecfg->UE_ANT_NUM; ant_id++) {
        pkt->frame_id_ = frame_id;
        pkt->symbol_id_ = symbol_id;
        pkt->ant_id_ = ant_id;
        pkt->cell_id_ = 0;
        memcpy(pkt->data_,
            &tx_buffer_ue[total_offset_ue + ant_id * payload_length],
            payload_length);
        ssize_t ret = sendto(socket_ue_[ant_id], (char*)udp_pkt_buf.data(),
            udp_pkt_buf.size(), 0, (struct sockaddr*)&servaddr_ue_[ant_id],
            sizeof(servaddr_ue_[ant_id]));
        rt_assert(ret > 0, "sendto() failed");
    }

    rt_assert(message_queue_.enqueue(*task_ptok[tid],
                  EventData(EventType::kPacketTX,
                      gen_tag_t::frm_sym_ue(frame_id, symbol_id, 0)._tag)),
        "UE TX message enqueue failed!\n");
}
