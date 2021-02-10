#include "receiver.h"

Receiver::Receiver(Config* cfg, size_t rx_thread_num, size_t core_offset)
    : rx_thread_num_(rx_thread_num), core_id_(core_offset), cfg_(cfg) {}

Receiver::Receiver(Config* cfg, size_t rx_thread_num, size_t core_offset,
                   moodycamel::ConcurrentQueue<EventData>* in_queue_message,
                   moodycamel::ProducerToken** in_rx_ptoks)
    : Receiver(cfg, rx_thread_num, core_offset) {
  message_queue_ = in_queue_message;
  rx_ptoks_ = in_rx_ptoks;
}

Receiver::~Receiver() { delete cfg_; }

std::vector<pthread_t> Receiver::StartRecv(Table<char>& in_buffer,
                                           Table<int>& in_buffer_status,
                                           size_t in_buffer_frame_num,
                                           size_t in_buffer_length,
                                           Table<double>& in_frame_start) {
  buffer_frame_num_ = in_buffer_frame_num;
  buffer_length_ = in_buffer_length;
  buffer_ = &in_buffer;
  buffer_status_ = &in_buffer_status;
  frame_start_ = &in_frame_start;

  std::printf("start Recv thread\n");
  std::vector<pthread_t> created_threads;

  for (size_t i = 0; i < rx_thread_num_; i++) {
    pthread_t recv_thread;
    auto* context = new EventHandlerContext<Receiver>;
    context->obj_ptr_ = this;
    context->id_ = i;
    if (pthread_create(&recv_thread, nullptr,
                       PthreadFunWrapper<Receiver, &Receiver::LoopRecv>,
                       context) != 0) {
      perror("Socket recv thread create failed");
      std::exit(0);
    }
    created_threads.push_back(recv_thread);
  }
  return created_threads;
}

void* Receiver::LoopRecv(int tid) {
  size_t core_offset = core_id_ + rx_thread_num_ + 2;
  PinToCoreWithOffset(ThreadType::kWorkerRX, core_offset, tid);

  int sock_buf_size = 1024 * 1024 * 64 * 8 - 1;
  struct sockaddr_in remote_addr;
  int socket_local =
      SetupSocketIpv4(cfg_->BsRruPort() + tid, true, sock_buf_size);
  SetupSockaddrRemoteIpv4(&remote_addr, cfg_->BsServerPort() + tid,
                          cfg_->BsServerAddr().c_str());

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
  // loop recv
  socklen_t addrlen = sizeof(remote_addr);
  size_t offset = 0;
  int prev_frame_id = -1;
  while (true) {
    /* if buffer is full, exit */
    if (cur_buffer_status_ptr[0] == 1) {
      std::printf("Receive thread %d buffer full, offset: %zu\n", tid, offset);
      std::exit(0);
    }

    int recvlen = -1;
    // if ((recvlen = recv(socket_local, (char*)cur_buffer_ptr,
    // packet_length, 0))<0) {
    if ((recvlen =
             recvfrom(socket_local, (char*)cur_buffer_ptr, cfg_->PacketLength(),
                      0, (struct sockaddr*)&remote_addr, &addrlen)) < 0) {
      perror("recv failed");
      std::exit(0);
    }

    // Read information from received packet
    auto* pkt = (struct Packet*)cur_buffer_ptr;
    int frame_id = pkt->frame_id_;

    if (kDebugSenderReceiver) {
      std::printf("RX thread %d received frame %d symbol %d, ant %d\n ", tid,
                  frame_id, pkt->symbol_id_, pkt->ant_id_);
    }

    if (kIsWorkerTimingEnabled) {
      if (frame_id > prev_frame_id) {
        frame_start[frame_id] = GetTime();
        prev_frame_id = frame_id;
      }
    }
    /* get the position in buffer */
    offset = cur_buffer_status_ptr - buffer_status_ptr;
    cur_buffer_status_ptr[0] = 1;
    cur_buffer_status_ptr = buffer_status_ptr + (offset + 1) % buffer_frame_num;
    cur_buffer_ptr =
        buffer_ptr +
        (cur_buffer_ptr - buffer_ptr + cfg_->PacketLength()) % buffer_length;

    /* Push packet received event into the queue */
    EventData packet_message(EventType::kPacketRX, rx_tag_t(tid, offset).tag_);

    if (!message_queue_->enqueue(*local_ptok, packet_message)) {
      std::printf("socket message enqueue failed\n");
      std::exit(0);
    }
  }
}
