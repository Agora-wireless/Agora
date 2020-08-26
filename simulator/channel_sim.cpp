#include "channel_sim.hpp"

static bool running = true;

static void convert_short_to_float_simd(
    short* in_buf, float* out_buf, size_t length)
{
#if SIMD
#ifdef __AVX512F__
    const __m512 magic = _mm512_set1_ps(float((1 << 23) + (1 << 15)) / 32768.f);
    const __m512i magic_i = _mm512_castps_si512(magic);
    for (size_t i = 0; i < length; i += 16) {
        /* get input */
        __m256i val = _mm256_load_si256((__m256i*)(in_buf + i)); // port 2,3
        /* interleave with 0x0000 */
        __m512i val_unpacked = _mm512_cvtepu16_epi32(val); // port 5
        /* convert by xor-ing and subtracting magic value:
         * VPXOR avoids port5 bottlenecks on Intel CPUs before SKL */
        __m512i val_f_int
            = _mm512_xor_si512(val_unpacked, magic_i); // port 0,1,5
        __m512 val_f = _mm512_castsi512_ps(val_f_int); // no instruction
        __m512 converted = _mm512_sub_ps(val_f, magic); // port 1,5 ?
        _mm512_store_ps(out_buf + i, converted); // port 2,3,4,7
    }
#else
    const __m256 magic = _mm256_set1_ps(float((1 << 23) + (1 << 15)) / 32768.f);
    const __m256i magic_i = _mm256_castps_si256(magic);
    for (size_t i = 0; i < length; i += 16) {
        /* get input */
        __m128i val = _mm_load_si128((__m128i*)(in_buf + i)); // port 2,3

        __m128i val1 = _mm_load_si128((__m128i*)(in_buf + i + 8));
        /* interleave with 0x0000 */
        __m256i val_unpacked = _mm256_cvtepu16_epi32(val); // port 5
        /* convert by xor-ing and subtracting magic value:
         * VPXOR avoids port5 bottlenecks on Intel CPUs before SKL */
        __m256i val_f_int
            = _mm256_xor_si256(val_unpacked, magic_i); // port 0,1,5
        __m256 val_f = _mm256_castsi256_ps(val_f_int); // no instruction
        __m256 converted = _mm256_sub_ps(val_f, magic); // port 1,5 ?
        _mm256_store_ps(out_buf + i, converted); // port 2,3,4,7

        __m256i val_unpacked1 = _mm256_cvtepu16_epi32(val1); // port 5
        __m256i val_f_int1
            = _mm256_xor_si256(val_unpacked1, magic_i); // port 0,1,5
        __m256 val_f1 = _mm256_castsi256_ps(val_f_int1); // no instruction
        __m256 converted1 = _mm256_sub_ps(val_f1, magic); // port 1,5 ?
        _mm256_store_ps(out_buf + i + 8, converted1); // port 2,3,4,7
    }
#endif
    for (size_t i = length / 16; i < length; i++) {
        out_buf[i] = (float)(in_buf[i] / 32768.f);
    }
#else
    for (size_t i = 0; i < length; i++) {
        out_buf[i] = (float)(in_buf[i] / 32768.f);
    }
#endif
}

static void convert_float_to_short_simd(
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
ChannelSim::ChannelSim(Config* config_bs, Config* config_ue,
    size_t bs_socket_num, size_t user_socket_num, size_t bs_thread_num,
    size_t user_thread_num, size_t worker_thread_num, size_t in_core_offset)
    : bs_thread_num(bs_thread_num)
    , user_thread_num(user_thread_num)
    , bs_socket_num(bs_socket_num)
    , user_socket_num(user_socket_num)
    , worker_thread_num(worker_thread_num)
    , core_offset(in_core_offset)
{
    this->bscfg = config_bs;
    this->uecfg = config_ue;

    /* initialize random seed: */
    srand(time(NULL));
    numAntennas = bscfg->BS_ANT_NUM;
    nUEs = bscfg->UE_ANT_NUM;
    samps_persymbol = bscfg->sampsPerSymbol;
    symbol_perframe = bscfg->symbol_num_perframe;
    dl_symbol_perframe = bscfg->dl_data_symbol_num_perframe + 1; // plus beacon
    ul_data_symbol_perframe = bscfg->ul_data_symbol_num_perframe;
    pilot_symbol_perframe = bscfg->pilot_symbol_num_perframe;
    ul_symbol_perframe = ul_data_symbol_perframe + pilot_symbol_perframe;
    socket_bs_.resize(bs_socket_num);
    servaddr_bs_.resize(bs_socket_num);
    socket_ue_.resize(user_socket_num);
    servaddr_ue_.resize(user_socket_num);

    task_queue_bs = moodycamel::ConcurrentQueue<Event_data>(
        TASK_BUFFER_FRAME_NUM * dl_symbol_perframe * numAntennas * 36);
    task_queue_user = moodycamel::ConcurrentQueue<Event_data>(
        TASK_BUFFER_FRAME_NUM * ul_symbol_perframe * nUEs * 36);
    message_queue_ = moodycamel::ConcurrentQueue<Event_data>(
        TASK_BUFFER_FRAME_NUM * symbol_perframe * (numAntennas + nUEs) * 36);

    assert(bscfg->packet_length == uecfg->packet_legnth);
    payload_len = bscfg->packet_length - Packet::kOffsetOfData;
    payload_samps = bscfg->sampsPerSymbol;

    // initialize buffers
    size_t tx_buffer_ue_size
        = TASK_BUFFER_FRAME_NUM * dl_symbol_perframe * nUEs * payload_len;
    alloc_buffer_1d(&tx_buffer_ue, tx_buffer_ue_size, 64, 1);

    size_t tx_buffer_bs_size = TASK_BUFFER_FRAME_NUM * ul_symbol_perframe
        * numAntennas * payload_len;
    alloc_buffer_1d(&tx_buffer_bs, tx_buffer_bs_size, 64, 1);

    size_t rx_buffer_ue_size
        = TASK_BUFFER_FRAME_NUM * ul_symbol_perframe * nUEs * payload_len;
    alloc_buffer_1d(&rx_buffer_ue, rx_buffer_ue_size, 64, 1);

    size_t rx_buffer_bs_size = TASK_BUFFER_FRAME_NUM * dl_symbol_perframe
        * numAntennas * payload_len;
    alloc_buffer_1d(&rx_buffer_bs, rx_buffer_bs_size, 64, 1);

    bs_rx_counter_ = new size_t[dl_symbol_perframe * TASK_BUFFER_FRAME_NUM];
    memset(bs_rx_counter_, 0,
        sizeof(size_t) * dl_symbol_perframe * TASK_BUFFER_FRAME_NUM);

    user_rx_counter_ = new size_t[ul_symbol_perframe * TASK_BUFFER_FRAME_NUM];
    memset(user_rx_counter_, 0,
        sizeof(size_t) * ul_symbol_perframe * TASK_BUFFER_FRAME_NUM);

    memset(bs_tx_counter_, 0, sizeof(size_t) * TASK_BUFFER_FRAME_NUM);
    memset(user_tx_counter_, 0, sizeof(size_t) * TASK_BUFFER_FRAME_NUM);

    cx_fmat H(randn<fmat>(nUEs, numAntennas), randn<fmat>(nUEs, numAntennas));
    channel = H / abs(H).max();
    std::cout << "Channel: " << channel << std::endl;

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

void ChannelSim::schedule_task(Event_data do_task,
    moodycamel::ConcurrentQueue<Event_data>* in_queue,
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

        if (pthread_create(&recv_thread_bs, NULL,
                pthread_fun_wrapper<ChannelSim, &ChannelSim::bs_rx_loop>,
                bs_context)
            != 0) {
            perror("socket send thread create failed");
            exit(0);
        }
    }

    for (size_t i = 0; i < user_thread_num; i++) {
        pthread_t recv_thread_ue;

        auto ue_context = new EventHandlerContext<ChannelSim>;
        ue_context->obj_ptr = this;
        ue_context->id = i;

        if (pthread_create(&recv_thread_ue, NULL,
                pthread_fun_wrapper<ChannelSim, &ChannelSim::ue_rx_loop>,
                ue_context)
            != 0) {
            perror("socket recv thread create failed");
            exit(0);
        }
    }

    sleep(1);

    int ret = 0;
    Event_data events_list[dequeue_bulk_size];
    while (true) {
        ret = message_queue_.try_dequeue_bulk(
            ctok, events_list, dequeue_bulk_size);
        for (int bulk_count = 0; bulk_count < ret; bulk_count++) {
            Event_data& event = events_list[bulk_count];

            switch (event.event_type) {

            case EventType::kPacketRX: {
                size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
                size_t symbol_id = gen_tag_t(event.tags[0]).symbol_id;
                if (gen_tag_t(event.tags[0]).tag_type == TagType::kUsers) {
                    size_t pilot_symbol_id
                        = uecfg->get_pilot_symbol_idx(frame_id, symbol_id);
                    size_t ul_symbol_id
                        = uecfg->get_ul_symbol_idx(frame_id, symbol_id);
                    size_t total_symbol_id = pilot_symbol_id;
                    if (pilot_symbol_id == SIZE_MAX)
                        total_symbol_id = ul_symbol_id + pilot_symbol_perframe;
                    size_t frame_offset = (frame_id % TASK_BUFFER_FRAME_NUM)
                            * ul_symbol_perframe
                        + total_symbol_id;
                    user_rx_counter_[frame_offset]++;
                    if (user_rx_counter_[frame_offset] == nUEs) {
                        user_rx_counter_[frame_offset] = 0;
                        printf("Scheduling transmission of frame %zu from %zu "
                               "users to bs\n",
                            frame_id, nUEs);
                        Event_data do_tx_bs_task(
                            EventType::kPacketTX, event.tags[0]);
                        schedule_task(do_tx_bs_task, &task_queue_bs, ptok_bs);
                    }
                } else if (gen_tag_t(event.tags[0]).tag_type
                    == TagType::kAntennas) {
                    size_t dl_symbol_id
                        = bscfg->get_dl_symbol_idx(frame_id, symbol_id);
                    size_t frame_offset = (frame_id % TASK_BUFFER_FRAME_NUM)
                            * dl_symbol_perframe
                        + dl_symbol_id;
                    bs_rx_counter_[frame_offset]++;
                    if (bs_rx_counter_[frame_offset] == numAntennas) {
                        bs_rx_counter_[frame_offset] = 0;
                        printf("Scheduling transmission of frame %zu from %zu "
                               "bs antennas to  %zu users\n",
                            frame_id, numAntennas, nUEs);
                        Event_data do_tx_user_task(
                            EventType::kPacketTX, event.tags[0]);
                        schedule_task(
                            do_tx_user_task, &task_queue_user, ptok_user);
                    }
                }
            } break;

            case EventType::kPacketTX: {
                size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
                size_t offset = frame_id % TASK_BUFFER_FRAME_NUM;
                if (gen_tag_t(event.tags[0]).tag_type == TagType::kUsers) {
                    user_tx_counter_[offset]++;
                    if (user_tx_counter_[offset] == dl_symbol_perframe - 1) {
                        printf("Finished sending frame %zu to %zu users\n",
                            frame_id, nUEs);
                        user_tx_counter_[offset] = 0;
                    }
                } else if (gen_tag_t(event.tags[0]).tag_type
                    == TagType::kAntennas) {
                    bs_tx_counter_[offset]++;
                    if (bs_tx_counter_[offset] == ul_symbol_perframe) {
                        printf(
                            "Finished sending frame %zu to %zu basestation\n",
                            frame_id, numAntennas);
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

    Event_data event;
    while (running) {
        if (task_queue_bs.try_dequeue(event))
            do_tx_bs(tid, event.tags[0]);
        else if (task_queue_user.try_dequeue(event))
            do_tx_user(tid, event.tags[0]);
    }
    return 0;
}

void* ChannelSim::bs_rx_loop(int tid)
{
    int socket_lo = tid * bs_socket_num / bs_thread_num;
    int socket_hi = (tid + 1) * bs_socket_num / bs_thread_num;

    moodycamel::ProducerToken local_ptok(message_queue_);
    pin_to_core_with_offset(ThreadType::kWorkerTXRX, core_offset + 1, tid);

    int sock_buf_size = 1024 * 1024 * 64 * 8 - 1;
    for (int socket_id = socket_lo; socket_id < socket_hi; ++socket_id) {
        int local_port_id = bscfg->bs_rru_port + socket_id;
        socket_bs_[socket_id]
            = setup_socket_ipv4(local_port_id, true, sock_buf_size);
        setup_sockaddr_remote_ipv4(&servaddr_bs_[socket_id],
            bscfg->bs_port + socket_id, bscfg->bs_addr.c_str());
        printf("BS RX thread %d: set up UDP socket server listening to port %d"
               " with remote address %s:%d \n",
            tid, local_port_id, bscfg->bs_addr.c_str(),
            bscfg->bs_port + socket_id);
        fcntl(socket_bs_[socket_id], F_SETFL, O_NONBLOCK);
    }

    std::vector<uint8_t> udp_pkt_buf(bscfg->packet_length, 0);
    int socket_id = socket_lo;
    while (running) {
        if (-1
            == recv(socket_bs_[socket_id], (char*)udp_pkt_buf.data(),
                   udp_pkt_buf.size(), 0)) {
            if (errno != EAGAIN && running) {
                printf(
                    "Thread %d socket_id %d recv bs failed\n", tid, socket_id);
                exit(0);
            }
            continue;
        }
        const auto* pkt = reinterpret_cast<Packet*>(&udp_pkt_buf[0]);

        size_t frame_id = pkt->frame_id;
        size_t symbol_id = pkt->symbol_id;
        size_t ant_id = pkt->ant_id;
        printf(
            "Received bs packet frame %zu symbol %zu ant %zu from socket %d\n",
            frame_id, symbol_id, ant_id, socket_id);
        size_t dl_symbol_id = bscfg->get_dl_symbol_idx(frame_id, symbol_id);
        size_t symbol_offset
            = (frame_id % TASK_BUFFER_FRAME_NUM) * dl_symbol_perframe
            + dl_symbol_id;
        size_t offset = symbol_offset * numAntennas + ant_id;
        memcpy((void*)(rx_buffer_bs + offset * payload_len), pkt->data,
            payload_len);

        Event_data bs_rx_message(EventType::kPacketRX,
            gen_tag_t::frm_sym_ant(frame_id, symbol_id, ant_id)._tag);
        if (!message_queue_.enqueue(local_ptok, bs_rx_message)) {
            printf("socket message enqueue failed\n");
            exit(0);
        }
        if (++socket_id == socket_hi)
            socket_id = socket_lo;
    }
    return 0;
}

void* ChannelSim::ue_rx_loop(int tid)
{
    int socket_lo = tid * user_socket_num / user_thread_num;
    int socket_hi = (tid + 1) * user_socket_num / user_thread_num;

    moodycamel::ProducerToken local_ptok(message_queue_);
    pin_to_core_with_offset(
        ThreadType::kWorkerTXRX, core_offset + 1 + bs_thread_num, tid);

    int sock_buf_size = 1024 * 1024 * 64 * 8 - 1;
    for (int socket_id = socket_lo; socket_id < socket_hi; ++socket_id) {
        int local_port_id = uecfg->ue_rru_port + socket_id;
        socket_ue_[socket_id]
            = setup_socket_ipv4(local_port_id, true, sock_buf_size);
        setup_sockaddr_remote_ipv4(&servaddr_ue_[socket_id],
            uecfg->ue_port + socket_id, uecfg->ue_addr.c_str());
        printf("UE RX thread %d: set up UDP socket server listening to port %d"
               " with remote address %s:%d \n",
            tid, local_port_id, uecfg->ue_addr.c_str(),
            uecfg->ue_port + socket_id);
        fcntl(socket_ue_[socket_id], F_SETFL, O_NONBLOCK);
    }

    std::vector<uint8_t> udp_pkt_buf(bscfg->packet_length, 0);
    int socket_id = socket_lo;
    while (running) {
        if (-1
            == recv(socket_ue_[socket_id], (char*)&udp_pkt_buf[0],
                   udp_pkt_buf.size(), 0)) {
            if (errno != EAGAIN && running) {
                printf(
                    "Thread %d socket_id %d recv ue failed\n", tid, socket_id);
                exit(0);
            }
            continue;
        }

        const auto* pkt = reinterpret_cast<Packet*>(&udp_pkt_buf[0]);

        size_t frame_id = pkt->frame_id;
        size_t symbol_id = pkt->symbol_id;
        size_t ant_id = pkt->ant_id;

        size_t pilot_symbol_id
            = uecfg->get_pilot_symbol_idx(frame_id, symbol_id);
        size_t ul_symbol_id = uecfg->get_ul_symbol_idx(frame_id, symbol_id);
        size_t total_symbol_id = pilot_symbol_id;
        if (pilot_symbol_id == SIZE_MAX)
            total_symbol_id = ul_symbol_id + pilot_symbol_perframe;
        printf("Received ue packet frame %zu symbol %zu ant %zu\n", frame_id,
            symbol_id, ant_id);
        size_t symbol_offset
            = (frame_id % TASK_BUFFER_FRAME_NUM) * ul_symbol_perframe
            + total_symbol_id;
        size_t offset = symbol_offset * nUEs + ant_id;
        memcpy((void*)(rx_buffer_ue + offset * payload_len), pkt->data,
            payload_len);

        Event_data user_rx_message(EventType::kPacketRX,
            gen_tag_t::frm_sym_ue(frame_id, symbol_id, ant_id)._tag);
        if (!message_queue_.enqueue(local_ptok, user_rx_message)) {
            printf("socket message enqueue failed\n");
            exit(0);
        }
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
        total_symbol_id = ul_symbol_id + pilot_symbol_perframe;

    size_t symbol_offset
        = (frame_id % TASK_BUFFER_FRAME_NUM) * ul_symbol_perframe
        + total_symbol_id;
    size_t total_offset_ue = symbol_offset * payload_len * nUEs;
    size_t total_offset_bs = symbol_offset * payload_len * numAntennas;

    short* src_ptr = (short*)(rx_buffer_ue + total_offset_ue);

    cx_fmat fmat_src = zeros<cx_fmat>(payload_samps, nUEs);
    convert_short_to_float_simd(
        src_ptr, (float*)fmat_src.memptr(), 2 * payload_samps * nUEs);

    cx_fmat fmat_dst = fmat_src * channel;

    short* dst_ptr = (short*)(tx_buffer_bs + total_offset_bs);
    convert_float_to_short_simd(
        (float*)fmat_dst.memptr(), dst_ptr, 2 * payload_samps * numAntennas);
    std::stringstream ss;

    // send the symbol to all base station antennas
    std::vector<uint8_t> udp_pkt_buf(bscfg->packet_length, 0);
    auto* pkt = reinterpret_cast<Packet*>(&udp_pkt_buf[0]);
    for (size_t ant_id = 0; ant_id < numAntennas; ant_id++) {
        new (pkt) Packet(frame_id, symbol_id, 0 /* cell_id */, ant_id);
        memcpy(pkt->data,
            (void*)(tx_buffer_bs + total_offset_bs + ant_id * payload_len),
            payload_len);
        ssize_t ret = sendto(socket_bs_[ant_id], (char*)udp_pkt_buf.data(),
            udp_pkt_buf.size(), 0, (struct sockaddr*)&servaddr_bs_[ant_id],
            sizeof(servaddr_bs_[ant_id]));
        rt_assert(ret > 0, "sendto() failed");
    }

    Event_data bs_tx_message(EventType::kPacketTX,
        gen_tag_t::frm_sym_ant(frame_id, symbol_id, 0)._tag);
    if (!message_queue_.enqueue(*task_ptok[tid], bs_tx_message)) {
        printf("bs tx message enqueue failed\n");
        exit(0);
    }
}

void ChannelSim::do_tx_user(int tid, size_t tag)
{
    size_t frame_id = gen_tag_t(tag).frame_id;
    size_t symbol_id = gen_tag_t(tag).symbol_id;
    size_t dl_symbol_id = bscfg->get_dl_symbol_idx(frame_id, symbol_id);

    size_t symbol_offset
        = (frame_id % TASK_BUFFER_FRAME_NUM) * dl_symbol_perframe
        + dl_symbol_id;
    size_t total_offset_ue = symbol_offset * payload_len * nUEs;
    size_t total_offset_bs = symbol_offset * payload_len * numAntennas;

    short* src_ptr = (short*)(rx_buffer_bs + total_offset_bs);

    cx_fmat fmat_src = zeros<cx_fmat>(payload_samps, numAntennas);
    convert_short_to_float_simd(
        src_ptr, (float*)fmat_src.memptr(), 2 * payload_samps * numAntennas);

    cx_fmat fmat_dst = fmat_src * channel.st();

    short* dst_ptr = (short*)(tx_buffer_ue + total_offset_ue);
    convert_float_to_short_simd(
        (float*)fmat_dst.memptr(), dst_ptr, 2 * payload_samps * nUEs);

    // send the symbol to all base station antennas
    std::vector<uint8_t> udp_pkt_buf(bscfg->packet_length, 0);
    auto* pkt = reinterpret_cast<Packet*>(&udp_pkt_buf[0]);
    for (size_t ant_id = 0; ant_id < nUEs; ant_id++) {
        new (pkt) Packet(frame_id, symbol_id, 0 /* cell_id */, ant_id);
        memcpy(pkt->data,
            (void*)(tx_buffer_ue + total_offset_ue + ant_id * payload_len),
            payload_len);
        ssize_t ret = sendto(socket_ue_[ant_id], (char*)udp_pkt_buf.data(),
            udp_pkt_buf.size(), 0, (struct sockaddr*)&servaddr_ue_[ant_id],
            sizeof(servaddr_ue_[ant_id]));
        rt_assert(ret > 0, "sendto() failed");
    }

    Event_data user_tx_message(EventType::kPacketTX,
        gen_tag_t::frm_sym_ue(frame_id, symbol_id, 0)._tag);
    if (!message_queue_.enqueue(*task_ptok[tid], user_tx_message)) {
        printf("user tx message enqueue failed\n");
        exit(0);
    }
}
