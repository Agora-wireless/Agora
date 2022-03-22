/**
 * @file  throughput.cc
 * @brief Implementation file for the simple throughput class
 */

#include "throughput.h"
#include "logger.h"
#include <iostream>


Throughput::Throughput(size_t thread_num, const char* filename)
    : thread_num_(thread_num), filename_(filename) {
  // init throughput when mac_receiver start_t
  // malloc recv buffer for clock
  std::printf("+++++Throughput: Create throughput_buffer of size %zu * %zu\n", thread_num_, kNumStatsFrames);
  buffer_queue_.Malloc(thread_num_ /* RX */, kNumStatsFrames, Agora_memory::Alignment_t::kAlign64);
  AllocBuffer1d(&number_buffers_, thread_num_, Agora_memory::Alignment_t::kAlign32, 1);
  AllocBuffer1d(&total_buffers_, thread_num_, Agora_memory::Alignment_t::kAlign64, 1);
  baseTime = GetTime::GetTimeUs();
}

Throughput::~Throughput() {
  std::printf("+++++Throughput: Exiting\n");
  if (strlen(filename_)==0) std::printf("result file empty\n");
  else WriteFile();
  buffer_queue_.Free();
  FreeBuffer1d(&number_buffers_);
  FreeBuffer1d(&total_buffers_);
}

void Throughput::Stamp(size_t tid, size_t buffer_size, size_t frame_id, size_t ue_id) {
  double curr = GetTime::GetTimeUs() - baseTime; // us
  if (!buffer_size) return;
  TimePoint* tp = &buffer_queue_[tid][number_buffers_[tid]++];
  total_buffers_[tid] += buffer_size;
  tp->buffer_size_ = buffer_size;
  tp->buffer_time_ = curr;
  tp->frame_id = frame_id;
  tp->ue_id = ue_id;
  std::printf("+++++Throughput stamps\n thread: %zu, buffer_size: %zu, time: %f,\n Buffer num: %u, Total buffer size: %zu\n", 
              tid, buffer_size, curr, number_buffers_[tid], total_buffers_[tid]);
}

void Throughput::WriteFile() {
  std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  std::string filename = cur_directory + "/log/" + filename_;
  MLPD_INFO("+++++Printing log results to file \"%s\"...\n", filename.c_str());

  std::ofstream debug_file;
  debug_file.open(filename, std::ifstream::out);
  debug_file.setf(std::ios::fixed, std::ios::floatfield);
  debug_file.precision(2);

  RtAssert(debug_file.is_open() == true, "Failed to open stats file");
  debug_file << "Buffer id: [buffer index:frame_id:ue_id]" << std::endl;
  for (size_t tid = 0; tid < thread_num_; ++tid) {
    debug_file << "thread " << tid << " handled: " << number_buffers_[tid] << " buffers" << ":\n";
    double last_stamp = buffer_queue_[tid][0].buffer_time_;
    baseTime = last_stamp; // set base time to time 0
    // ssize_t total_buffer = buffer_queue_[tid][0].buffer_size_;
    ssize_t total_buffer = 0;
    debug_file << "Buffer[" << 0 << ":" << buffer_queue_[tid][0].frame_id << ":"
               << buffer_queue_[tid][0].ue_id << "] buffer_size: " << buffer_queue_[tid][0].buffer_size_ 
                << "[Byte] Time: " << last_stamp << "[us]\n"; 
    for (size_t i = 1; i < number_buffers_[tid]; ++i) {
      TimePoint &tp = buffer_queue_[tid][i];
      
      debug_file << "Buffer[" << i << ":" << tp.frame_id << ":"
                << tp.ue_id << "] buffer_size: " << tp.buffer_size_
                << "[Byte] Time: " << tp.buffer_time_ << "[us], Duration: " << tp.buffer_time_ - last_stamp << "[us]\n"
                << "Last Interval Throughput: " << tp.buffer_size_/((tp.buffer_time_ - last_stamp)/1e6) << "[B/s]\n"
                << "Average Throughput: " << (total_buffer += tp.buffer_size_)/((tp.buffer_time_ - baseTime)/1e6) << "[B/s]\n"; 
      last_stamp = tp.buffer_time_;
    }
    debug_file << "\n";
  }
  debug_file << std::endl;
  debug_file.close();
}


