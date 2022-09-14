/**
 * @file simulator.cc
 * @brief Implementation file for the simulator class
 */
#include "simulator.h"

#include <memory>

#include "gettime.h"

static const bool kDebugPrintAllRxSymbols = false;
static const bool kDebugPrintSimSetup = true;

static const size_t kMessageQueueSize = 512;

Simulator::Simulator(Config* cfg, size_t in_task_thread_num,
                     size_t in_core_offset, size_t sender_delay)
    : task_thread_num_(in_task_thread_num),
      socket_rx_thread_num_(in_task_thread_num),
      socket_tx_thread_num_(in_task_thread_num),
      core_offset_(in_core_offset) {
  if (kDebugPrintSimSetup) {
    std::printf("Simulator: Main thread: on core %d\n", sched_getcpu());
    // setenv("MKL_THREADING_LAYER", "sequential", true /* overwrite */);
    // std::cout << "MKL_THREADING_LAYER =  " << getenv("MKL_THREADING_LAYER")
    //           << std::endl;
  }

  this->config_ = cfg;
  PinToCoreWithOffset(ThreadType::kMaster, core_offset_, 0);

  InitializeQueues();

  if (kDebugPrintSimSetup) {
    std::printf("Simulator: Initialize buffers\n");
  }
  InitializeBuffers();
  size_t enable_slow_start = 1;
  size_t frame_delay = 0;

  sender_ = std::make_unique<Sender>(
      config_, socket_tx_thread_num_, core_offset_ + 1, sender_delay,
      frame_delay, enable_slow_start, "ff:ff:ff:ff:ff:ff", true);
  receiver_ =
      std::make_unique<Receiver>(config_, socket_rx_thread_num_, core_offset_,
                                 &message_queue_, rx_ptoks_ptr_);
}

Simulator::~Simulator() {
  this->FreeBuffers();
  this->FreeQueues();
}

void Simulator::Stop() {
  std::cout << "Simulator: stopping threads " << std::endl;
  config_->Running(false);
  usleep(1000);
  std::printf("Simulator: stopped\n");
}

void Simulator::Start() {
  config_->Running(true);
  /* start receiver */
  std::vector<std::thread> rx_threads =
      receiver_->StartRecv(socket_buffer_, socket_buffer_size_, frame_start_);

  /* tokens used for dequeue */
  moodycamel::ConsumerToken ctok(message_queue_);
  moodycamel::ConsumerToken ctok_complete(complete_task_queue_);

  /* counters for printing summary */
  size_t frame_count_rx = 0;
  size_t ret = 0;
  std::array<EventData, kDequeueBulkSize> events_list;

  /* start transmitter */
  sender_->StartTxfromMain(frame_start_tx_, frame_end_tx_);
  while ((config_->Running() == true) &&
         (SignalHandler::GotExitSignal() == false)) {
    /* get a bulk of events */
    ret = message_queue_.try_dequeue_bulk(ctok, events_list.data(),
                                          kDequeueBulkSizeSingle);

    if (ret > 0) {
      /* handle each event */
      for (size_t bulk_count = 0; bulk_count < ret; bulk_count++) {
        EventData& event = events_list.at(bulk_count);
        switch (event.event_type_) {
          case EventType::kPacketRX: {
            RxPacket* rx_packet = rx_tag_t(event.tags_[0]).rx_packet_;
            Packet* pkt = rx_packet->RawPacket();

            size_t frame_id = pkt->frame_id_ % kNumStatsFrames;
            size_t symbol_id = pkt->symbol_id_;
            size_t ant_id = pkt->ant_id_;
            size_t frame_id_in_buffer = (frame_id % kFrameWnd);
            rx_packet->Free();

            // Only process the dl symbols & ignore beacon frames
            SymbolType symbol_type = config_->GetSymbolType(symbol_id);
            if (symbol_type == SymbolType::kDL) {
              if (kDebugPrintAllRxSymbols) {
                std::printf(
                    "Simulator: In main received from frame %zu, symbol %zu : "
                    "%zu, ant %zu\n",
                    frame_id, symbol_id, config_->Frame().NumTotalSyms() - 1,
                    ant_id);
              }
              UpdateRxCounters(frame_id, frame_id_in_buffer, symbol_id, ant_id);
            }
          } break;

          default:
            std::printf("Wrong event type in message queue!");
            throw std::runtime_error(
                "Simulator: Wrong event type in message queue!");
        }  // end of switch
      }    // end of for
    }      // end ret > 0
  }        // end of while
  this->Stop();
  sender_.reset();
  const std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  const std::string filename =
      cur_directory + "/files/experiment/timeresult_simulator.txt";
  FILE* fp = std::fopen(filename.c_str(), "w");
  if (fp == nullptr) {
    std::printf("open file failed\n");
    std::cerr << "Error: " << strerror(errno) << std::endl;
    throw std::runtime_error("Simulator: open file failed");
  }

  std::printf("Printing results to file......\n");
  for (size_t ii = 0; ii < frame_count_rx; ii++) {
    std::fprintf(fp, "%.3f %.3f %.3f %.3f\n", frame_start_[0][ii],
                 frame_start_[1][ii], frame_start_receive_[ii],
                 frame_end_receive_[ii]);
  }

  std::printf("Simulator: Joining RX Threads......\n");
  for (auto& join_thread : rx_threads) {
    join_thread.join();
  }
  receiver_.reset();
  std::printf("Rx threads joined......\n");
}

inline void Simulator::UpdateFrameCount(int* frame_count) {
  *frame_count = *frame_count + 1;
  if (*frame_count == 1e9) {
    *frame_count = 0;
  }
}

void Simulator::UpdateRxCounters(size_t frame_id, size_t frame_id_in_buffer,
                                 size_t symbol_id, size_t ant_id) {
  // Currently the rx thread only receives from 1 port / antenna
  size_t max_packet_num_per_frame =
      socket_rx_thread_num_ * config_->Frame().NumDLSyms();

  rx_counter_packets_[frame_id_in_buffer]++;
  if (rx_counter_packets_[frame_id_in_buffer] == 1) {
    frame_start_receive_[frame_id] = GetTime::GetTimeUs();
    if (kDebugPrintPerFrameStart) {
      std::printf(
          "Simulator: data received from frame %zu, symbol %zu, "
          "ant %zu, in %.2f since tx start, in %.2f us since last frame\n",
          frame_id, symbol_id, ant_id,
          frame_start_receive_[frame_id] - frame_start_tx_[frame_id],
          ((frame_id == 0) ? frame_start_receive_[frame_id]
                           : frame_start_receive_[frame_id] -
                                 frame_start_receive_[frame_id - 1]));
    }
  } else if (rx_counter_packets_[frame_id_in_buffer] ==
             max_packet_num_per_frame) {
    frame_end_receive_[frame_id] = GetTime::GetTimeUs();
    PrintPerFrameDone(PrintType::kPacketRX, frame_id);
    rx_counter_packets_[frame_id_in_buffer] = 0;

    if (frame_id >= (config_->FramesToTest() - 1)) {
      std::printf("Simulator: received the final frame\n");
      config_->Running(false);
    }
  }
}

void Simulator::PrintPerFrameDone(PrintType print_type, size_t frame_id) {
  if (kDebugPrintPerFrameDone) {
    switch (print_type) {
      case (PrintType::kPacketRX): {
        std::printf(
            "Simulator: received all packets in frame: %zu, in %.2f us since "
            "tx, rx duration: %.2f us, tx duration: %.2f us\n",
            frame_id, frame_end_receive_[frame_id] - frame_start_tx_[frame_id],
            frame_end_receive_[frame_id] - frame_start_receive_[frame_id],
            frame_end_tx_[frame_id] - frame_start_tx_[frame_id]);
      } break;
      default:
        std::printf("Wrong task type in frame done print!");
    }
  }
}

void Simulator::InitializeQueues() {
  message_queue_ = moodycamel::ConcurrentQueue<EventData>(
      kMessageQueueSize * config_->Frame().NumDataSyms());
  complete_task_queue_ = moodycamel::ConcurrentQueue<EventData>(
      kMessageQueueSize * config_->Frame().NumDataSyms() * 4);

  rx_ptoks_ptr_ =
      static_cast<moodycamel::ProducerToken**>(Agora_memory::PaddedAlignedAlloc(
          Agora_memory::Alignment_t::kAlign64,
          socket_rx_thread_num_ * sizeof(moodycamel::ProducerToken*)));
  for (size_t i = 0; i < socket_rx_thread_num_; i++) {
    rx_ptoks_ptr_[i] = new moodycamel::ProducerToken(message_queue_);
  }
}

void Simulator::FreeQueues() {
  for (size_t i = 0; i < socket_rx_thread_num_; i++) {
    delete (rx_ptoks_ptr_[i]);
  }

  std::free(rx_ptoks_ptr_);
  rx_ptoks_ptr_ = nullptr;
}

void Simulator::InitializeBuffers() {
  socket_buffer_size_ = (long long)config_->PacketLength() *
                        config_->Frame().NumTotalSyms() * config_->BsAntNum() *
                        kFrameWnd;

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

void Simulator::FreeBuffers() {
  socket_buffer_.Free();

  FreeBuffer1d(&rx_counter_packets_);

  frame_start_.Free();
  FreeBuffer1d(&frame_start_receive_);
  FreeBuffer1d(&frame_end_receive_);
  FreeBuffer1d(&frame_start_tx_);
  FreeBuffer1d(&frame_end_tx_);
}
