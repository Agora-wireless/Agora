/**
 * @file receiver.cc
 * @brief Implementation file for the receiver class
 */
#include "receiver.h"

#include "udp_server.h"

Receiver::Receiver(Config* cfg, size_t rx_thread_num, size_t core_offset)
    : rx_thread_num_(rx_thread_num), core_id_(core_offset), cfg_(cfg) {}

Receiver::Receiver(Config* cfg, size_t rx_thread_num, size_t core_offset,
                   moodycamel::ConcurrentQueue<EventData>* in_queue_message,
                   moodycamel::ProducerToken** in_rx_ptoks)
    : Receiver(cfg, rx_thread_num, core_offset) {
  message_queue_ = in_queue_message;
  rx_ptoks_ = in_rx_ptoks;
}

std::vector<std::thread> Receiver::StartRecv(Table<char>& in_buffer,
                                             Table<int>& in_buffer_status,
                                             size_t in_buffer_frame_num,
                                             size_t in_buffer_length,
                                             Table<double>& in_frame_start) {
  buffer_frame_num_ = in_buffer_frame_num;
  buffer_length_ = in_buffer_length;
  buffer_ = &in_buffer;
  buffer_status_ = &in_buffer_status;
  frame_start_ = &in_frame_start;

  std::printf("Start Recv thread\n");
  std::vector<std::thread> created_threads;
  created_threads.resize(rx_thread_num_);

  for (size_t i = 0; i < rx_thread_num_; i++) {
    created_threads.at(i) = std::thread(&Receiver::LoopRecv, this, i);
  }
  return created_threads;
}

void* Receiver::LoopRecv(int tid) {
  size_t core_offset = core_id_ + rx_thread_num_ + 2;
  PinToCoreWithOffset(ThreadType::kWorkerRX, core_offset, tid);

  size_t sock_buf_size = (1024 * 1024 * 64 * 8) - 1;
  // int socket_local =
  //    SetupSocketIpv4(cfg_->BsRruPort() + tid, true, sock_buf_size);

  auto udp_server =
      std::make_unique<UDPServer>(cfg_->BsRruPort() + tid, sock_buf_size);

  udp_server->MakeBlocking();

  /* use token to speed up */
  moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];

  char* buffer_ptr = (*buffer_)[tid];
  int* buffer_status_ptr = (*buffer_status_)[tid];
  long long buffer_length = buffer_length_;
  int buffer_frame_num = buffer_frame_num_;
  double* frame_start = (*frame_start_)[tid];

  // // walk through all the pages
  // double temp;
  // for (int i = 0; i < 20; i++) {
  //     temp = frame_start[i * 512];
  // }

  char* cur_buffer_ptr = buffer_ptr;
  int* cur_buffer_status_ptr = buffer_status_ptr;

  size_t offset = 0;
  ssize_t prev_frame_id = -1;
  while (this->cfg_->Running() == true) {
    /* if buffer is full, exit */
    if (cur_buffer_status_ptr[0] == 1) {
      std::printf("Receive thread %d buffer full, offset: %zu\n", tid, offset);
      throw std::runtime_error("Receiver: Receive thread buffer full");
    }

    ssize_t recvlen = udp_server->RecvFrom(
        reinterpret_cast<uint8_t*>(cur_buffer_ptr), cfg_->PacketLength(),
        cfg_->BsServerAddr(), cfg_->BsServerPort() + tid);
    if (recvlen < 0) {
      std::perror("recv failed");
      throw std::runtime_error("Receiver: recv failed");
    } else if (static_cast<size_t>(recvlen) == cfg_->PacketLength()) {
      // Read information from received packet
      auto* pkt = reinterpret_cast<struct Packet*>(cur_buffer_ptr);
      size_t frame_id = pkt->frame_id_;

      if (kDebugSenderReceiver) {
        std::printf("RX thread %d received frame %zu symbol %d, ant %d\n ", tid,
                    frame_id, pkt->symbol_id_, pkt->ant_id_);
      }

      if (kIsWorkerTimingEnabled) {
        if (static_cast<ssize_t>(frame_id) > prev_frame_id) {
          frame_start[frame_id] = GetTime::GetTime();
          prev_frame_id = frame_id;
        }
      }
      /* get the position in buffer */
      offset = cur_buffer_status_ptr - buffer_status_ptr;
      cur_buffer_status_ptr[0] = 1;
      cur_buffer_status_ptr =
          buffer_status_ptr + (offset + 1) % buffer_frame_num;
      cur_buffer_ptr =
          buffer_ptr +
          (cur_buffer_ptr - buffer_ptr + cfg_->PacketLength()) % buffer_length;

      /* Push packet received event into the queue */
      EventData packet_message(EventType::kPacketRX,
                               rx_tag_t(tid, offset).tag_);

      if (message_queue_->enqueue(*local_ptok, packet_message) == false) {
        std::printf("socket message enqueue failed\n");
        throw std::runtime_error("Receiver: socket message enqueue failed");
      }
    } else if (recvlen != 0) {
      std::perror("recv failed with less than requested bytes");
      throw std::runtime_error(
          "Receiver: recv failed with less than requested bytes");
    }
  }
  return nullptr;
}
