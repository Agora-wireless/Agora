/**
 * @file receiver.cc
 * @brief Implementation file for the receiver class
 */
#include "receiver.h"

#include "gettime.h"
#include "udp_comm.h"

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
                                             size_t in_buffer_length,
                                             Table<double>& in_frame_start) {
  frame_start_ = &in_frame_start;

  std::printf("Start Recv thread\n");
  std::vector<std::thread> created_threads;

  buffers_per_thread_ = in_buffer_length / rx_thread_num_;
  rx_packets_.resize(rx_thread_num_);
  created_threads.resize(rx_thread_num_);
  for (size_t i = 0; i < rx_thread_num_; i++) {
    rx_packets_.at(i).reserve(buffers_per_thread_);
    for (size_t number_packets = 0; number_packets < buffers_per_thread_;
         number_packets++) {
      auto* pkt_loc = reinterpret_cast<Packet*>(
          in_buffer[i] + (number_packets * cfg_->PacketLength()));
      rx_packets_.at(i).emplace_back(pkt_loc);
    }
    created_threads.at(i) = std::thread(&Receiver::LoopRecv, this, i);
  }
  return created_threads;
}

void* Receiver::LoopRecv(size_t tid) {
  size_t core_offset = core_id_ + rx_thread_num_ + 2;
  PinToCoreWithOffset(ThreadType::kWorkerRX, core_offset, tid);

  const size_t sock_buf_size = (1024 * 1024 * 64 * 8) - 1;
  auto udp_server = std::make_unique<UDPComm>(
      cfg_->BsRruAddr(), cfg_->BsRruPort() + tid, sock_buf_size, 0);

  udp_server->MakeBlocking(1);

  /* use token to speed up */
  moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];
  double* frame_start = (*frame_start_)[tid];

  size_t rx_slot = 0;
  RxPacket* current_packet = &rx_packets_.at(tid).at(rx_slot);

  ssize_t prev_frame_id = -1;
  while (this->cfg_->Running() == true) {
    /* if buffer is full, exit */
    if (current_packet->Empty() == false) {
      std::printf("Receive thread %zu buffer full, offset: %zu\n", tid,
                  rx_slot);
      throw std::runtime_error("Receiver: Receive thread buffer full");
    }

    ssize_t recvlen = udp_server->Recv(
        cfg_->BsServerAddr(), cfg_->BsServerPort() + tid,
        reinterpret_cast<std::byte*>(current_packet->RawPacket()),
        cfg_->PacketLength());
    if (recvlen < 0) {
      std::perror("recv failed");
      throw std::runtime_error("Receiver: recv failed");
    } else if (static_cast<size_t>(recvlen) == cfg_->PacketLength()) {
      // Read information from received packet
      size_t frame_id = current_packet->RawPacket()->frame_id_;

      if (kDebugSenderReceiver) {
        std::printf("RX thread %zu received frame %zu symbol %d, ant %d\n ",
                    tid, frame_id, current_packet->RawPacket()->symbol_id_,
                    current_packet->RawPacket()->ant_id_);
      }

      if (kIsWorkerTimingEnabled) {
        if (static_cast<ssize_t>(frame_id) > prev_frame_id) {
          frame_start[frame_id] = GetTime::GetTime();
          prev_frame_id = frame_id;
        }
      }

      current_packet->Use();
      // Push packet received event into the queue
      EventData packet_message(EventType::kPacketRX,
                               rx_tag_t(current_packet).tag_);

      if (message_queue_->enqueue(*local_ptok, packet_message) == false) {
        std::printf("socket message enqueue failed\n");
        throw std::runtime_error("Receiver: socket message enqueue failed");
      }
      rx_slot = (rx_slot + 1) % buffers_per_thread_;
      current_packet = &rx_packets_.at(tid).at(rx_slot);
    } else if (recvlen != 0) {
      std::perror("recv failed with less than requested bytes");
      throw std::runtime_error(
          "Receiver: recv failed with less than requested bytes");
    }
  }
  return nullptr;
}
