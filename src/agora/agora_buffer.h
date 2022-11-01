/**
 * @file agora_buffer.h
 * @brief Declaration file for the AgoraBuffer class
 */

#ifndef AGORA_BUFFER_H_
#define AGORA_BUFFER_H_

#include <array>
#include <cstddef>

#include "common_typedef_sdk.h"
#include "concurrentqueue.h"
#include "config.h"
#include "memory_manage.h"
#include "message.h"
#include "symbols.h"
#include "utils.h"

class AgoraBuffer {
 public:
  explicit AgoraBuffer(Config* const cfg);
  // Delete copy constructor and copy assignment
  AgoraBuffer(AgoraBuffer const&) = delete;
  AgoraBuffer& operator=(AgoraBuffer const&) = delete;
  ~AgoraBuffer();

  inline PtrGrid<kFrameWnd, kMaxUEs, complex_float>& GetCsi() {
    return csi_buffer_;
  }
  inline PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& GetUlBeamMatrix() {
    return ul_beam_matrix_;
  }
  inline PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& GetDlBeamMatrix() {
    return dl_beam_matrix_;
  }
  inline PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& GetDemod() {
    return demod_buffer_;
  }
  inline PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& GetDecod() {
    return decoded_buffer_;
  }
  inline Table<complex_float>& GetFft() { return fft_buffer_; }
  inline Table<complex_float>& GetEqual() { return equal_buffer_; }
  inline Table<complex_float>& GetUeSpecPilot() {
    return ue_spec_pilot_buffer_;
  }
  inline Table<complex_float>& GetIfft() { return dl_ifft_buffer_; }
  inline Table<complex_float>& GetCalibUlMsum() {
    return calib_ul_msum_buffer_;
  }
  inline Table<complex_float>& GetCalibDlMsum() {
    return calib_dl_msum_buffer_;
  }
  inline Table<int8_t>& GetDlModBits() { return dl_mod_bits_buffer_; }
  inline Table<int8_t>& GetDlBits() { return dl_bits_buffer_; }
  inline Table<int8_t>& GetDlBitsStatus() { return dl_bits_buffer_status_; }

  inline size_t GetUlSocketSize() const { return ul_socket_buf_size_; }
  inline Table<char>& GetUlSocket() { return ul_socket_buffer_; }
  inline char* GetDlSocket() { return dl_socket_buffer_; }
  inline Table<complex_float>& GetCalibUl() { return calib_ul_buffer_; }
  inline Table<complex_float>& GetCalibDl() { return calib_dl_buffer_; }

 private:
  void AllocateTables();
  void FreeTables();

  Config* const config_;
  const size_t ul_socket_buf_size_;

  PtrGrid<kFrameWnd, kMaxUEs, complex_float> csi_buffer_;
  PtrGrid<kFrameWnd, kMaxDataSCs, complex_float> ul_beam_matrix_;
  PtrGrid<kFrameWnd, kMaxDataSCs, complex_float> dl_beam_matrix_;
  PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t> demod_buffer_;
  PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t> decoded_buffer_;
  Table<complex_float> fft_buffer_;
  Table<complex_float> equal_buffer_;
  Table<complex_float> ue_spec_pilot_buffer_;
  Table<complex_float> dl_ifft_buffer_;
  Table<complex_float> calib_ul_msum_buffer_;
  Table<complex_float> calib_dl_msum_buffer_;
  Table<int8_t> dl_mod_bits_buffer_;
  Table<int8_t> dl_bits_buffer_;
  Table<int8_t> dl_bits_buffer_status_;

  Table<char> ul_socket_buffer_;
  char* dl_socket_buffer_;
  Table<complex_float> calib_ul_buffer_;
  Table<complex_float> calib_dl_buffer_;
};

struct SchedInfo {
  moodycamel::ConcurrentQueue<EventData> concurrent_q_;
  moodycamel::ProducerToken* ptok_;
};

// Used to communicate between the manager and the worker class
//Needs to manage its own memory
class MessageInfo {
 public:
  explicit MessageInfo(size_t queue_size) { Alloc(queue_size); }
  ~MessageInfo() { Free(); }

  inline moodycamel::ProducerToken* GetPtok(EventType event_type, size_t qid) {
    return sched_info_arr_.at(qid).at(static_cast<size_t>(event_type)).ptok_;
  }

  inline moodycamel::ConcurrentQueue<EventData>* GetConq(EventType event_type,
                                                         size_t qid) {
    return &sched_info_arr_.at(qid)
                .at(static_cast<size_t>(event_type))
                .concurrent_q_;
  }

  inline moodycamel::ConcurrentQueue<EventData>& GetCompQueue(size_t qid) {
    return complete_task_queue_.at(qid);
  }

  inline moodycamel::ProducerToken* GetWorkerPtok(size_t qid,
                                                  size_t worker_id) {
    return worker_ptoks_ptr_.at(qid).at(worker_id);
  }

 private:
  inline void Alloc(size_t queue_size) {
    for (auto& queue : complete_task_queue_) {
      queue = moodycamel::ConcurrentQueue<EventData>(queue_size);
    }
    for (auto& queue : sched_info_arr_) {
      for (auto& event : queue) {
        event.concurrent_q_ =
            moodycamel::ConcurrentQueue<EventData>(queue_size);
        event.ptok_ = new moodycamel::ProducerToken(event.concurrent_q_);
      }
    }

    size_t queue_count = 0;
    for (auto& queue : worker_ptoks_ptr_) {
      for (auto& worker : queue) {
        worker =
            new moodycamel::ProducerToken(complete_task_queue_.at(queue_count));
      }
      queue_count++;
    }
  }

  inline void Free() {
    for (auto& queue : sched_info_arr_) {
      for (auto& event : queue) {
        delete event.ptok_;
        event.ptok_ = nullptr;
      }
    }
    for (auto& queue : worker_ptoks_ptr_) {
      for (auto& worker : queue) {
        delete worker;
        worker = nullptr;
      }
    }
  }

  std::array<moodycamel::ConcurrentQueue<EventData>, kScheduleQueues>
      complete_task_queue_;
  std::array<std::array<moodycamel::ProducerToken*, kMaxThreads>,
             kScheduleQueues>
      worker_ptoks_ptr_;
  std::array<std::array<SchedInfo, kNumEventTypes>, kScheduleQueues>
      sched_info_arr_;
};

struct FrameInfo {
  size_t cur_sche_frame_id_;
  size_t cur_proc_frame_id_;
};

#endif  // AGORA_BUFFER_H_