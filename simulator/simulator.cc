/**
 * @file simulator.cc
 * @brief Implementation file for the simulator class
 */
#include "simulator.h"

#include <memory>

Simulator::Simulator(Config* cfg, size_t in_task_thread_num,
                     size_t in_core_offset, size_t sender_delay)
    : task_thread_num_(in_task_thread_num),
      socket_rx_thread_num_(in_task_thread_num),
      socket_tx_thread_num_(in_task_thread_num),
      core_offset_(in_core_offset) {
  std::string directory = TOSTRING(PROJECT_DIRECTORY);
  std::printf("PROJECT_DIRECTORY: %s\n", directory.c_str());
  std::printf("Main thread: on core %d\n", sched_getcpu());
  // setenv("MKL_THREADING_LAYER", "sequential", true /* overwrite */);
  // std::cout << "MKL_THREADING_LAYER =  " << getenv("MKL_THREADING_LAYER")
  //           << std::endl;
  std::printf("enter constructor\n");

  this->config_ = cfg;

  InitializeVarsFromCfg(cfg);
  PinToCoreWithOffset(ThreadType::kMaster, core_offset_, 0);

  InitializeQueues();

  std::printf("initialize buffers\n");
  InitializeUplinkBuffers();

  std::printf("new Sender\n");
  sender_ = std::make_unique<Sender>(config_, socket_tx_thread_num_,
                                     core_offset_ + 1, sender_delay, true);

  std::printf("new Receiver\n");
  receiver_ =
      std::make_unique<Receiver>(config_, socket_rx_thread_num_, core_offset_,
                                 &message_queue_, rx_ptoks_ptr_);
}

Simulator::~Simulator() {
  this->FreeUplinkBuffers();
  this->FreeQueues();
}

void Simulator::Stop() {
  std::cout << "stopping threads " << std::endl;
  config_->Running(false);
  usleep(1000);
  receiver_.reset();
  sender_.reset();
}

void Simulator::Start() {
  config_->Running(true);
  /* start receiver */
  std::vector<std::thread> rx_threads = receiver_->StartRecv(
      socket_buffer_, socket_buffer_status_, socket_buffer_status_size_,
      socket_buffer_size_, frame_start_);

  /* tokens used for dequeue */
  moodycamel::ConsumerToken ctok(message_queue_);
  moodycamel::ConsumerToken ctok_complete(complete_task_queue_);

  buffer_frame_num_ = symbol_num_perframe_ * bs_ant_num_ * kFrameWnd;
  max_packet_num_per_frame_ = bs_ant_num_ * dl_data_symbol_num_perframe_;

  /* counters for printing summary */
  int frame_count_rx = 0;

  int ret = 0;
  EventData events_list[kDequeueBulkSize];

  /* start transmitter */
  sender_->StartTXfromMain(frame_start_tx_, frame_end_tx_);
  while ((config_->Running() == true) &&
         (SignalHandler::GotExitSignal() == false)) {
    /* get a bulk of events */
    ret = 0;
    ret = message_queue_.try_dequeue_bulk(ctok, events_list,
                                          kDequeueBulkSizeSingle);

    if (ret == 0) {
      continue;
    }

    /* handle each event */
    for (int bulk_count = 0; bulk_count < ret; bulk_count++) {
      EventData& event = events_list[bulk_count];
      switch (event.event_type_) {
        case EventType::kPacketRX: {
          int socket_thread_id = rx_tag_t(event.tags_[0]).tid_;
          int buf_offset = rx_tag_t(event.tags_[0]).offset_;

          char* socket_buffer_ptr = socket_buffer_[socket_thread_id] +
                                    (long long)buf_offset * packet_length_;
          auto* pkt = reinterpret_cast<struct Packet*>(socket_buffer_ptr);
          int frame_id = pkt->frame_id_ % 10000;
          int symbol_id = pkt->symbol_id_;
          int ant_id = pkt->ant_id_;
          int frame_id_in_buffer = (frame_id % kFrameWnd);
          socket_buffer_status_[socket_thread_id][buf_offset] = 0;

          // std::printf(
          //     "In main: received from frame %d %d, symbol %d, ant
          //     %d\n", frame_id, frame_id_in_buffer, symbol_id,
          //     ant_id);

          UpdateRxCounters(frame_id, frame_id_in_buffer, symbol_id, ant_id);
        } break;

        default:
          std::printf("Wrong event type in message queue!");
          throw std::runtime_error(
              "Simulator: Wrong event type in message queue!");
      } /* end of switch */
    }   /* end of for */
  }     /* end of while */
  this->Stop();
  std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  std::string filename = cur_directory + "/data/timeresult_simulator.txt";
  FILE* fp = std::fopen(filename.c_str(), "w");
  if (fp == nullptr) {
    std::printf("open file failed\n");
    std::cerr << "Error: " << strerror(errno) << std::endl;
    throw std::runtime_error("Simulator: open file failed");
  }

  std::printf("Printing results to file......\n");
  for (int ii = 0; ii < frame_count_rx; ii++) {
    std::fprintf(fp, "%.3f %.3f %.3f %.3f\n", frame_start_[0][ii],
                 frame_start_[1][ii], frame_start_receive_[ii],
                 frame_end_receive_[ii]);
  }

  for (auto& join_thread : rx_threads) {
    join_thread.join();
  }
}

inline void Simulator::UpdateFrameCount(int* frame_count) {
  *frame_count = *frame_count + 1;
  if (*frame_count == 1e9) {
    *frame_count = 0;
  }
}

void Simulator::UpdateRxCounters(size_t frame_id, size_t frame_id_in_buffer,
                                 size_t symbol_id, size_t ant_id) {
  rx_counter_packets_[frame_id_in_buffer]++;
  if (rx_counter_packets_[frame_id_in_buffer] == 1) {
    frame_start_receive_[frame_id] = GetTime::GetTime();
    if (kDebugPrintPerFrameStart) {
      std::printf(
          "Main thread: data received from frame %zu, symbol %zu, ant "
          "%zu, in %.2f since tx, in %.2f us since last frame\n",
          frame_id, symbol_id, ant_id,
          frame_start_receive_[frame_id] - frame_start_tx_[frame_id],
          frame_start_receive_[frame_id] - frame_start_receive_[frame_id - 1]);
    }
  } else if (rx_counter_packets_[frame_id_in_buffer] ==
             max_packet_num_per_frame_) {
    frame_end_receive_[frame_id] = GetTime::GetTime();
    PrintPerFrameDone(PrintType::kPacketRX, frame_id);
    rx_counter_packets_[frame_id_in_buffer] = 0;
  }
}

void Simulator::PrintPerFrameDone(PrintType print_type, size_t frame_id) {
  if (!kDebugPrintPerFrameDone) {
    return;
  }
  switch (print_type) {
    case (PrintType::kPacketRX): {
      std::printf(
          "Main thread: received all packets in frame: %zu, in %.2f us since "
          "tx, in %.2f us since rx, tx duration: %.2f us\n",
          frame_id, frame_end_receive_[frame_id] - frame_start_tx_[frame_id],
          frame_end_receive_[frame_id] - frame_start_receive_[frame_id],
          frame_end_tx_[frame_id] - frame_start_tx_[frame_id]);
    } break;
    default:
      std::printf("Wrong task type in frame done print!");
  }
}

void Simulator::InitializeVarsFromCfg(Config* cfg) {
  bs_ant_num_ = cfg->BsAntNum();
  ue_num_ = cfg->UeNum();
  ofdm_ca_num_ = cfg->OfdmCaNum();
  ofdm_data_num_ = cfg->OfdmDataNum();
  symbol_num_perframe_ = cfg->Frame().NumTotalSyms();
  data_symbol_num_perframe_ = cfg->Frame().NumDataSyms();
  ul_data_symbol_num_perframe_ = cfg->Frame().NumULSyms();
  dl_data_symbol_num_perframe_ = cfg->Frame().NumDLSyms();

  if (dl_data_symbol_num_perframe_ > 0) {
    dl_data_symbol_start_ = cfg->Frame().GetDLSymbol(0);
    dl_data_symbol_end_ = cfg->Frame().GetDLSymbolLast();
  } else {
    dl_data_symbol_start_ = dl_data_symbol_end_ = 0;
  }

  packet_length_ = cfg->PacketLength();

  demul_block_size_ = cfg->DemulBlockSize();
  demul_block_num_ = ofdm_data_num_ / demul_block_size_ +
                     (ofdm_data_num_ % demul_block_size_ == 0 ? 0 : 1);
}

void Simulator::InitializeQueues() {
  message_queue_ =
      moodycamel::ConcurrentQueue<EventData>(512 * data_symbol_num_perframe_);
  complete_task_queue_ = moodycamel::ConcurrentQueue<EventData>(
      512 * data_symbol_num_perframe_ * 4);

  rx_ptoks_ptr_ =
      static_cast<moodycamel::ProducerToken**>(Agora_memory::PaddedAlignedAlloc(
          Agora_memory::Alignment_t::kAlign64,
          socket_rx_thread_num_ * sizeof(moodycamel::ProducerToken*)));
  for (size_t i = 0; i < socket_rx_thread_num_; i++) {
    rx_ptoks_ptr_[i] = new moodycamel::ProducerToken(message_queue_);
  }

  task_ptoks_ptr_ =
      static_cast<moodycamel::ProducerToken**>(Agora_memory::PaddedAlignedAlloc(
          Agora_memory::Alignment_t::kAlign64,
          task_thread_num_ * sizeof(moodycamel::ProducerToken*)));
  for (size_t i = 0; i < task_thread_num_; i++) {
    task_ptoks_ptr_[i] = new moodycamel::ProducerToken(complete_task_queue_);
  }
}

void Simulator::FreeQueues() {
  for (size_t i = 0; i < socket_rx_thread_num_; i++) {
    delete (rx_ptoks_ptr_[i]);
  }

  std::free(rx_ptoks_ptr_);
  rx_ptoks_ptr_ = nullptr;

  for (size_t i = 0; i < task_thread_num_; i++) {
    delete (task_ptoks_ptr_[i]);
  }

  std::free(task_ptoks_ptr_);
  task_ptoks_ptr_ = nullptr;
}

void Simulator::InitializeUplinkBuffers() {
  socket_buffer_size_ = (long long)packet_length_ * symbol_num_perframe_ *
                        bs_ant_num_ * kFrameWnd;
  socket_buffer_status_size_ = symbol_num_perframe_ * bs_ant_num_ * kFrameWnd;
  socket_buffer_.Malloc(socket_rx_thread_num_, socket_buffer_size_,
                        Agora_memory::Alignment_t::kAlign64);
  socket_buffer_status_.Calloc(socket_rx_thread_num_,
                               socket_buffer_status_size_,
                               Agora_memory::Alignment_t::kAlign64);

  /* initilize all uplink status checkers */
  AllocBuffer1d(&rx_counter_packets_, kFrameWnd,
                Agora_memory::Alignment_t::kAlign64, 1);

  frame_start_.Calloc(socket_rx_thread_num_, kNumStatsFrames,
                      Agora_memory::Alignment_t::kAlign4096);
  AllocBuffer1d(&frame_start_receive_, kNumStatsFrames,
                Agora_memory::Alignment_t::kAlign4096, 1);
  AllocBuffer1d(&frame_end_receive_, kNumStatsFrames,
                Agora_memory::Alignment_t::kAlign4096, 1);
  AllocBuffer1d(&frame_start_tx_, kNumStatsFrames,
                Agora_memory::Alignment_t::kAlign4096, 1);
  AllocBuffer1d(&frame_end_tx_, kNumStatsFrames,
                Agora_memory::Alignment_t::kAlign4096, 1);
}

void Simulator::FreeUplinkBuffers() {
  socket_buffer_.Free();
  socket_buffer_status_.Free();

  FreeBuffer1d(&rx_counter_packets_);

  frame_start_.Free();
  FreeBuffer1d(&frame_start_receive_);
  FreeBuffer1d(&frame_end_receive_);
  FreeBuffer1d(&frame_start_tx_);
  FreeBuffer1d(&frame_end_tx_);
}
