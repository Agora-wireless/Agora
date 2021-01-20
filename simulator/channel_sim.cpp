#include "channel_sim.hpp"

#include "datatype_conversion.h"

static bool running = true;
static constexpr bool kPrintChannelOutput = false;

static void simd_convert_float_to_short(float* in_buf, short* out_buf,
                                        size_t length) {
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
                       size_t bs_thread_num, size_t user_thread_num,
                       size_t worker_thread_num, size_t in_core_offset,
                       std::string in_chan_type, double in_chan_snr)
    : bscfg_(config_bs),
      uecfg_(config_ue),
      bs_thread_num_(bs_thread_num),
      user_thread_num_(user_thread_num),
      bs_socket_num_(config_bs->bs_ant_num()),
      user_socket_num_(config_ue->ue_ant_num()),
      worker_thread_num_(worker_thread_num),
      core_offset_(in_core_offset),
      channel_type_(in_chan_type),
      channel_snr_(in_chan_snr) {
  // initialize parameters from config
  srand(time(NULL));
  dl_data_plus_beacon_symbols_ = bscfg_->frame().NumDLSyms() + 1;  // plus beacon
  ul_data_plus_pilot_symbols_ =
      bscfg_->frame().NumULSyms() + bscfg_->frame().NumPilotSyms();

  socket_bs_.resize(bs_socket_num_);
  servaddr_bs_.resize(bs_socket_num_);
  socket_ue_.resize(user_socket_num_);
  servaddr_ue_.resize(user_socket_num_);

  task_queue_bs_ = moodycamel::ConcurrentQueue<EventData>(
      kFrameWnd * dl_data_plus_beacon_symbols_ * bscfg_->bs_ant_num() * 36);
  task_queue_user_ = moodycamel::ConcurrentQueue<EventData>(
      kFrameWnd * ul_data_plus_pilot_symbols_ * uecfg_->ue_ant_num() * 36);
  message_queue_ = moodycamel::ConcurrentQueue<EventData>(
      kFrameWnd * bscfg_->frame().NumTotalSyms() *
      (bscfg_->bs_ant_num() + uecfg_->ue_ant_num()) * 36);

  assert(bscfg_->packet_length() == uecfg_->packet_length());
  payload_length_ = bscfg_->packet_length() - Packet::kOffsetOfData;

  // initialize bs-facing and client-facing data buffers
  size_t tx_buffer_ue_size = kFrameWnd * dl_data_plus_beacon_symbols_ *
                             uecfg_->ue_ant_num() * payload_length_;
  tx_buffer_ue_.resize(tx_buffer_ue_size);

  size_t tx_buffer_bs_size = kFrameWnd * ul_data_plus_pilot_symbols_ *
                             bscfg_->bs_ant_num() * payload_length_;
  tx_buffer_bs_.resize(tx_buffer_bs_size);

  size_t rx_buffer_ue_size = kFrameWnd * ul_data_plus_pilot_symbols_ *
                             uecfg_->ue_ant_num() * payload_length_;
  rx_buffer_ue_.resize(rx_buffer_ue_size);

  size_t rx_buffer_bs_size = kFrameWnd * dl_data_plus_beacon_symbols_ *
                             bscfg_->bs_ant_num() * payload_length_;
  rx_buffer_bs_.resize(rx_buffer_bs_size);

  // initilize rx and tx counters
  bs_rx_counter_ = new size_t[dl_data_plus_beacon_symbols_ * kFrameWnd];
  std::memset(bs_rx_counter_, 0,
              sizeof(size_t) * dl_data_plus_beacon_symbols_ * kFrameWnd);

  user_rx_counter_ = new size_t[ul_data_plus_pilot_symbols_ * kFrameWnd];
  std::memset(user_rx_counter_, 0,
              sizeof(size_t) * ul_data_plus_pilot_symbols_ * kFrameWnd);

  std::memset(bs_tx_counter_, 0, sizeof(size_t) * kFrameWnd);
  std::memset(user_tx_counter_, 0, sizeof(size_t) * kFrameWnd);

  // Initialize channel
  channel_ = new Channel(config_bs, config_ue, channel_type_, channel_snr_);

  for (size_t i = 0; i < worker_thread_num; i++) {
    task_ptok_[i] = new moodycamel::ProducerToken(message_queue_);
  }
  alloc_buffer_1d(&task_threads_, worker_thread_num,
                  Agora_memory::Alignment_t::k64Align, 0);

  // create task threads (transmit to base station and client antennas)
  for (size_t i = 0; i < worker_thread_num; i++) {
    auto context = new EventHandlerContext<ChannelSim>;
    context->obj_ptr_ = this;
    context->id_ = i;
    if (pthread_create(&task_threads_[i], NULL,
                       pthread_fun_wrapper<ChannelSim, &ChannelSim::taskThread>,
                       context) != 0) {
      std::perror("task thread create failed");
      std::exit(0);
    }
  }
}

ChannelSim::~ChannelSim(void) {
  // delete buffers, UDP client and servers
  // delete[] socket_uerx_;
  // delete[] socket_bsrx_;
}

void ChannelSim::schedule_task(
    EventData do_task, moodycamel::ConcurrentQueue<EventData>* in_queue,
    moodycamel::ProducerToken const& ptok) {
  if (!in_queue->try_enqueue(ptok, do_task)) {
    std::printf("need more memory\n");
    if (!in_queue->enqueue(ptok, do_task)) {
      std::printf("task enqueue failed\n");
      std::exit(0);
    }
  }
}

void ChannelSim::start() {
  std::printf("Starting Channel Simulator ...\n");
  pin_to_core_with_offset(ThreadType::kMaster, core_offset_, 0);

  moodycamel::ProducerToken ptok_bs(task_queue_bs_);
  moodycamel::ProducerToken ptok_user(task_queue_user_);
  moodycamel::ConsumerToken ctok(message_queue_);

  for (size_t i = 0; i < bs_thread_num_; i++) {
    pthread_t recv_thread_bs;

    auto bs_context = new EventHandlerContext<ChannelSim>;
    bs_context->obj_ptr_ = this;
    bs_context->id_ = i;

    int ret = pthread_create(
        &recv_thread_bs, NULL,
        pthread_fun_wrapper<ChannelSim, &ChannelSim::bs_rx_loop>, bs_context);
    rt_assert(ret == 0, "Failed to create BS recv thread!");
  }

  for (size_t i = 0; i < user_thread_num_; i++) {
    pthread_t recv_thread_ue;

    auto ue_context = new EventHandlerContext<ChannelSim>;
    ue_context->obj_ptr_ = this;
    ue_context->id_ = i;

    int ret = pthread_create(
        &recv_thread_ue, NULL,
        pthread_fun_wrapper<ChannelSim, &ChannelSim::ue_rx_loop>, ue_context);
    rt_assert(ret == 0, "Failed to create UE recv thread!");
  }

  sleep(1);

  int ret = 0;

  static constexpr size_t kDequeueBulkSize = 5;
  EventData events_list[kDequeueBulkSize];
  while (true) {
    ret = message_queue_.try_dequeue_bulk(ctok, events_list, kDequeueBulkSize);

    for (int bulk_count = 0; bulk_count < ret; bulk_count++) {
      EventData& event = events_list[bulk_count];

      switch (event.event_type_) {
        case EventType::kPacketRX: {
          size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
          // received a packet from a client antenna
          if (gen_tag_t(event.tags_[0]).tag_type_ == gen_tag_t::TagType::kUsers) {
            size_t pilot_symbol_id =
                uecfg_->GetPilotSymbolIdx(frame_id, symbol_id);
            size_t ul_symbol_id = uecfg_->GetULSymbolIdx(frame_id, symbol_id);
            size_t total_symbol_id = pilot_symbol_id;
            if (pilot_symbol_id == SIZE_MAX)
              total_symbol_id = ul_symbol_id + bscfg_->frame().NumPilotSyms();
            size_t frame_offset =
                (frame_id % kFrameWnd) * ul_data_plus_pilot_symbols_ +
                total_symbol_id;
            user_rx_counter_[frame_offset]++;
            // when received all client antennas on this symbol, kick-off BS TX
            if (user_rx_counter_[frame_offset] == uecfg_->ue_ant_num()) {
              user_rx_counter_[frame_offset] = 0;
              if (kDebugPrintPerSymbolDone)
                std::printf(
                    "Scheduling uplink transmission of frame %zu, "
                    "symbol %zu, from %zu "
                    "user to %zu BS antennas\n",
                    frame_id, symbol_id, uecfg_->ue_ant_num(),
                    bscfg_->bs_ant_num());
              schedule_task(EventData(EventType::kPacketTX, event.tags_[0]),
                            &task_queue_bs_, ptok_bs);
            }
            // received a packet from a BS antenna
          } else if (gen_tag_t(event.tags_[0]).tag_type_ ==
                     gen_tag_t::TagType::kAntennas) {
            size_t dl_symbol_id = get_dl_symbol_idx(frame_id, symbol_id);
            size_t frame_offset =
                (frame_id % kFrameWnd) * dl_data_plus_beacon_symbols_ +
                dl_symbol_id;
            bs_rx_counter_[frame_offset]++;

            // when received all BS antennas on this symbol, kick-off client TX
            if (bs_rx_counter_[frame_offset] == bscfg_->bs_ant_num()) {
              bs_rx_counter_[frame_offset] = 0;
              if (kDebugPrintPerSymbolDone)
                std::printf(
                    "Scheduling downlink transmission in frame "
                    "%zu, "
                    "symbol %zu, from %zu "
                    "BS to %zu user antennas\n",
                    frame_id, symbol_id, bscfg_->bs_ant_num(),
                    uecfg_->ue_ant_num());
              schedule_task(EventData(EventType::kPacketTX, event.tags_[0]),
                            &task_queue_user_, ptok_user);
            }
          }
        } break;

        case EventType::kPacketTX: {
          size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          size_t offset = frame_id % kFrameWnd;
          if (gen_tag_t(event.tags_[0]).tag_type_ == gen_tag_t::TagType::kUsers) {
            user_tx_counter_[offset]++;
            if (user_tx_counter_[offset] == dl_data_plus_beacon_symbols_) {
              if (kDebugPrintPerFrameDone)
                std::printf(
                    "Finished downlink transmission %zu symbols "
                    "in frame %zu\n",
                    dl_data_plus_beacon_symbols_, frame_id);
              user_tx_counter_[offset] = 0;
            }
          } else if (gen_tag_t(event.tags_[0]).tag_type_ ==
                     gen_tag_t::TagType::kAntennas) {
            bs_tx_counter_[offset]++;
            if (bs_tx_counter_[offset] == ul_data_plus_pilot_symbols_) {
              if (kDebugPrintPerFrameDone)
                std::printf(
                    "Finished uplink transmission of %zu "
                    "symbols in frame %zu\n",
                    ul_data_plus_pilot_symbols_, frame_id);
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

void* ChannelSim::taskThread(int tid) {
  pin_to_core_with_offset(ThreadType::kWorker,
                          core_offset_ + bs_thread_num_ + 1 + user_thread_num_,
                          tid);

  EventData event;
  while (running) {
    if (task_queue_bs_.try_dequeue(event))
      do_tx_bs(tid, event.tags_[0]);
    else if (task_queue_user_.try_dequeue(event))
      do_tx_user(tid, event.tags_[0]);
  }
  return 0;
}

void* ChannelSim::bs_rx_loop(int tid) {
  size_t socket_lo = tid * bs_socket_num_ / bs_thread_num_;
  size_t socket_hi = (tid + 1) * bs_socket_num_ / bs_thread_num_;

  moodycamel::ProducerToken local_ptok(message_queue_);
  pin_to_core_with_offset(ThreadType::kWorkerTXRX, core_offset_ + 1, tid);

  // initialize bs-facing sockets
  int sock_buf_size = 1024 * 1024 * 64 * 8 - 1;
  for (size_t socket_id = socket_lo; socket_id < socket_hi; ++socket_id) {
    int local_port_id = bscfg_->bs_rru_port() + socket_id;
    socket_bs_[socket_id] =
        setup_socket_ipv4(local_port_id, true, sock_buf_size);
    setup_sockaddr_remote_ipv4(&servaddr_bs_[socket_id],
                               bscfg_->bs_server_port() + socket_id,
                               bscfg_->bs_server_addr().c_str());
    std::printf(
        "BS RX thread %d: set up UDP socket server listening to port %d"
        " with remote address %s:%zu\n",
        tid, local_port_id, bscfg_->bs_server_addr().c_str(),
        bscfg_->bs_server_port() + socket_id);
    fcntl(socket_bs_[socket_id], F_SETFL, O_NONBLOCK);
  }

  std::vector<uint8_t> udp_pkt_buf(bscfg_->packet_length(), 0);
  size_t socket_id = socket_lo;
  while (running) {
    if (-1 == recv(socket_bs_[socket_id], (char*)udp_pkt_buf.data(),
                   udp_pkt_buf.size(), 0)) {
      if (errno != EAGAIN && running) {
        std::printf("BS socket %zu receive failed\n", socket_id);
        std::exit(0);
      }
      continue;
    }
    const auto* pkt = reinterpret_cast<Packet*>(&udp_pkt_buf[0]);

    size_t frame_id = pkt->frame_id_;
    size_t symbol_id = pkt->symbol_id_;
    size_t ant_id = pkt->ant_id_;
    if (kDebugPrintInTask)
      std::printf(
          "Received BS packet for frame %zu, symbol %zu, ant %zu from "
          "socket %zu\n",
          frame_id, symbol_id, ant_id, socket_id);
    size_t dl_symbol_id = get_dl_symbol_idx(frame_id, symbol_id);
    size_t symbol_offset =
        (frame_id % kFrameWnd) * dl_data_plus_beacon_symbols_ + dl_symbol_id;
    size_t offset = symbol_offset * bscfg_->bs_ant_num() + ant_id;
    std::memcpy(&rx_buffer_bs_[offset * payload_length_], pkt->data_,
                payload_length_);

    rt_assert(
        message_queue_.enqueue(
            local_ptok,
            EventData(
                EventType::kPacketRX,
                gen_tag_t::frm_sym_ant(frame_id, symbol_id, ant_id).tag_)),
        "BS socket message enqueue failed!");
    if (++socket_id == socket_hi) socket_id = socket_lo;
  }
  return 0;
}

void* ChannelSim::ue_rx_loop(int tid) {
  size_t socket_lo = tid * user_socket_num_ / user_thread_num_;
  size_t socket_hi = (tid + 1) * user_socket_num_ / user_thread_num_;

  moodycamel::ProducerToken local_ptok(message_queue_);
  pin_to_core_with_offset(ThreadType::kWorkerTXRX,
                          core_offset_ + 1 + bs_thread_num_, tid);

  // initialize client-facing sockets
  int sock_buf_size = 1024 * 1024 * 64 * 8 - 1;
  for (size_t socket_id = socket_lo; socket_id < socket_hi; ++socket_id) {
    int local_port_id = uecfg_->ue_rru_port() + socket_id;
    socket_ue_[socket_id] =
        setup_socket_ipv4(local_port_id, true, sock_buf_size);
    setup_sockaddr_remote_ipv4(&servaddr_ue_[socket_id],
                               uecfg_->ue_server_port() + socket_id,
                               uecfg_->ue_server_addr().c_str());
    std::printf(
        "UE RX thread %d: set up UDP socket server listening to port %d"
        " with remote address %s:%zu\n",
        tid, local_port_id, uecfg_->ue_server_addr().c_str(),
        uecfg_->ue_server_port() + socket_id);
    fcntl(socket_ue_[socket_id], F_SETFL, O_NONBLOCK);
  }

  std::vector<uint8_t> udp_pkt_buf(bscfg_->packet_length(), 0);
  size_t socket_id = socket_lo;
  while (running) {
    if (-1 == recv(socket_ue_[socket_id], (char*)&udp_pkt_buf[0],
                   udp_pkt_buf.size(), 0)) {
      if (errno != EAGAIN && running) {
        std::printf("UE socket %zu receive failed\n", socket_id);
        std::exit(0);
      }
      continue;
    }

    const auto* pkt = reinterpret_cast<Packet*>(&udp_pkt_buf[0]);

    size_t frame_id = pkt->frame_id_;
    size_t symbol_id = pkt->symbol_id_;
    size_t ant_id = pkt->ant_id_;

    size_t pilot_symbol_id = uecfg_->GetPilotSymbolIdx(frame_id, symbol_id);
    size_t ul_symbol_id = uecfg_->GetULSymbolIdx(frame_id, symbol_id);
    size_t total_symbol_id = pilot_symbol_id;
    if (pilot_symbol_id == SIZE_MAX)
      total_symbol_id = ul_symbol_id + bscfg_->frame().NumPilotSyms();
    if (kDebugPrintInTask)
      std::printf(
          "Received UE packet for frame %zu, symbol %zu, ant %zu from "
          "socket %zu\n",
          frame_id, symbol_id, ant_id, socket_id);
    size_t symbol_offset =
        (frame_id % kFrameWnd) * ul_data_plus_pilot_symbols_ + total_symbol_id;
    size_t offset = symbol_offset * uecfg_->ue_ant_num() + ant_id;
    std::memcpy(&rx_buffer_ue_[offset * payload_length_], pkt->data_,
                payload_length_);

    rt_assert(message_queue_.enqueue(
                  local_ptok,
                  EventData(
                      EventType::kPacketRX,
                      gen_tag_t::frm_sym_ue(frame_id, symbol_id, ant_id).tag_)),
              "UE Socket message enqueue failed!");
    if (++socket_id == socket_hi) socket_id = socket_lo;
  }
  return 0;
}

void ChannelSim::do_tx_bs(int tid, size_t tag) {
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t symbol_id = gen_tag_t(tag).symbol_id_;

  size_t pilot_symbol_id = bscfg_->GetPilotSymbolIdx(frame_id, symbol_id);
  size_t ul_symbol_id = bscfg_->GetULSymbolIdx(frame_id, symbol_id);
  size_t total_symbol_id = pilot_symbol_id;
  if (pilot_symbol_id == SIZE_MAX)
    total_symbol_id = ul_symbol_id + bscfg_->frame().NumPilotSyms();

  size_t symbol_offset =
      (frame_id % kFrameWnd) * ul_data_plus_pilot_symbols_ + total_symbol_id;
  size_t total_offset_ue = symbol_offset * payload_length_ * uecfg_->ue_ant_num();
  size_t total_offset_bs = symbol_offset * payload_length_ * bscfg_->bs_ant_num();

  auto* src_ptr = reinterpret_cast<short*>(&rx_buffer_ue_[total_offset_ue]);

  // convert received data to complex float,
  // apply channel, convert back to complex short to TX
  cx_fmat fmat_src =
      zeros<cx_fmat>(bscfg_->samps_per_symbol(), uecfg_->ue_ant_num());
  simd_convert_short_to_float(
      src_ptr, reinterpret_cast<float*>(fmat_src.memptr()),
      2 * bscfg_->samps_per_symbol() * uecfg_->ue_ant_num());

  // Apply Channel
  cx_fmat fmat_dst;
  bool is_downlink = false;
  bool is_new_frame = false;

  if (symbol_id == 0) {
    is_new_frame = true;
  }
  channel_->apply_chan(fmat_src, fmat_dst, is_downlink, is_new_frame);

  if (kPrintChannelOutput) Utils::print_mat(fmat_dst);

  auto* dst_ptr = reinterpret_cast<short*>(&tx_buffer_bs_[total_offset_bs]);
  simd_convert_float_to_short(
      reinterpret_cast<float*>(fmat_dst.memptr()), dst_ptr,
      2 * bscfg_->samps_per_symbol() * bscfg_->bs_ant_num());
  std::stringstream ss;

  // send the symbol to all base station antennas
  std::vector<uint8_t> udp_pkt_buf(bscfg_->packet_length(), 0);
  auto* pkt = reinterpret_cast<Packet*>(&udp_pkt_buf[0]);
  for (size_t ant_id = 0; ant_id < bscfg_->bs_ant_num(); ant_id++) {
    pkt->frame_id_ = frame_id;
    pkt->symbol_id_ = symbol_id;
    pkt->ant_id_ = ant_id;
    pkt->cell_id_ = 0;
    std::memcpy(pkt->data_,
                &tx_buffer_bs_[total_offset_bs + ant_id * payload_length_],
                payload_length_);
    ssize_t ret = sendto(
        socket_bs_[ant_id], (char*)udp_pkt_buf.data(), udp_pkt_buf.size(), 0,
        (struct sockaddr*)&servaddr_bs_[ant_id], sizeof(servaddr_bs_[ant_id]));
    rt_assert(ret > 0, "sendto() failed");
  }

  rt_assert(
      message_queue_.enqueue(
          *task_ptok_[tid],
          EventData(EventType::kPacketTX,
                     gen_tag_t::frm_sym_ant(frame_id, symbol_id, 0).tag_)),
      "BS TX message enqueue failed!\n");
}

void ChannelSim::do_tx_user(int tid, size_t tag) {
  size_t frame_id = gen_tag_t(tag).frame_id_;
  size_t symbol_id = gen_tag_t(tag).symbol_id_;
  size_t dl_symbol_id = get_dl_symbol_idx(frame_id, symbol_id);

  size_t symbol_offset =
      (frame_id % kFrameWnd) * dl_data_plus_beacon_symbols_ + dl_symbol_id;
  size_t total_offset_ue = symbol_offset * payload_length_ * uecfg_->ue_ant_num();
  size_t total_offset_bs = symbol_offset * payload_length_ * bscfg_->bs_ant_num();

  auto* src_ptr = reinterpret_cast<short*>(&rx_buffer_bs_[total_offset_bs]);

  // convert received data to complex float,
  // apply channel, convert back to complex short to TX
  cx_fmat fmat_src =
      zeros<cx_fmat>(bscfg_->samps_per_symbol(), bscfg_->bs_ant_num());
  simd_convert_short_to_float(
      src_ptr, reinterpret_cast<float*>(fmat_src.memptr()),
      2 * bscfg_->samps_per_symbol() * bscfg_->bs_ant_num());

  // Apply Channel
  cx_fmat fmat_dst;
  bool is_downlink = true;
  bool is_new_frame = false;

  if (symbol_id == 0) {
    is_new_frame = true;
  }
  channel_->apply_chan(fmat_src, fmat_dst, is_downlink, is_new_frame);

  if (kPrintChannelOutput) Utils::print_mat(fmat_dst);

  auto* dst_ptr = reinterpret_cast<short*>(&tx_buffer_ue_[total_offset_ue]);
  simd_convert_float_to_short(
      reinterpret_cast<float*>(fmat_dst.memptr()), dst_ptr,
      2 * bscfg_->samps_per_symbol() * uecfg_->ue_ant_num());

  // send the symbol to all base station antennas
  std::vector<uint8_t> udp_pkt_buf(bscfg_->packet_length(), 0);
  auto* pkt = reinterpret_cast<Packet*>(&udp_pkt_buf[0]);
  for (size_t ant_id = 0; ant_id < uecfg_->ue_ant_num(); ant_id++) {
    pkt->frame_id_ = frame_id;
    pkt->symbol_id_ = symbol_id;
    pkt->ant_id_ = ant_id;
    pkt->cell_id_ = 0;
    std::memcpy(pkt->data_,
                &tx_buffer_ue_[total_offset_ue + ant_id * payload_length_],
                payload_length_);
    ssize_t ret = sendto(
        socket_ue_[ant_id], (char*)udp_pkt_buf.data(), udp_pkt_buf.size(), 0,
        (struct sockaddr*)&servaddr_ue_[ant_id], sizeof(servaddr_ue_[ant_id]));
    rt_assert(ret > 0, "sendto() failed");
  }

  rt_assert(message_queue_.enqueue(
                *task_ptok_[tid],
                EventData(EventType::kPacketTX,
                           gen_tag_t::frm_sym_ue(frame_id, symbol_id, 0).tag_)),
            "UE TX message enqueue failed!\n");
}
