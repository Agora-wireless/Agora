/**
 * @file receiver_cli.cc
 * @brief Main file for the receiver class testing.
 */
#include "receiver.h"

int main(int argc, char const* argv[]) {
  int thread_num = strtol(argv[1], NULL, 10);
  Receiver receiver(thread_num);

  int buffer_frame_num = BS_ANT_NUM * symbol_num_perframe * 10;
  int buffer_length = Receiver::packet_length * buffer_frame_num;
  auto** buffer = new char*[thread_num];
  auto** buffer_status = new int*[thread_num];
  for (int i = 0; i < thread_num; i++) {
    buffer[i] = new char[buffer_length];
    buffer_status[i] = new int[buffer_frame_num];
  }

  std::vector<std::vector> receive_threads = receiver.startRecv(
      buffer, buffer_status, buffer_frame_num, buffer_length);

  for (auto& join_thread : receive_threads) {
    std::printf("Joining Receive Threads");
    join_thread.join();
  }

  for (int i = 0; i < thread_num; i++) {
    delete[] buffer[i];
    delete[] buffer_status[i];
  }
  delete[] buffer;
  delete[] buffer_status;

  return 0;
}