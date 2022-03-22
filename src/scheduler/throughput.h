/**
 * @file throughput.h
 * @brief Declaration file for the throughput class
 */
#ifndef THROUGHPUT_H_
#define THROUGHPUT_H_

#include "config.h"

struct TimePoint {
  ssize_t buffer_size_;
  double buffer_time_;
  size_t frame_id;
  size_t ue_id;
};

class Throughput {
  public:
    Throughput(size_t thread_num, const char* filename);
    ~Throughput();
    void Stamp(size_t tid, size_t buffer_size, size_t frame_id, size_t ue_id);
    void WriteFile();

  private:
    // process -> send/recv
    const size_t thread_num_; // number of threads
    const char* filename_; // file to write log

    unsigned int* number_buffers_; // number of buffers processed in each thread
    size_t* total_buffers_; // accumulated buffer size [Byte]
    // Times when processing a buffer
    // 1st dimension: num_receiver_threads
    // 2nd dimension: number of times receiving a buffer from phy (kNumStatsFrames)
    Table<TimePoint> buffer_queue_;
    double baseTime; // base time
};

#endif // THROUGHPUT_H_