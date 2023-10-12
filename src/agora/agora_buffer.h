/**
 * @file agora_buffer.h
 * @brief Declaration file for the AgoraBuffer class
 */

#ifndef AGORA_BUFFER_H_
#define AGORA_BUFFER_H_

#include <array>
#include <cstddef>
#include <queue>

#include "common_typedef_sdk.h"
#include "concurrentqueue.h"
#include "config.h"
#include "mac_scheduler.h"
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
  inline Table<std::complex<int16_t>> GetDlBcastSignal() {
    return dl_bcast_socket_buffer_;
  }
  inline Table<int8_t>& GetDlModBits() { return dl_mod_bits_buffer_; }
  inline Table<int8_t>& GetDlBits() { return dl_bits_buffer_; }
  inline Table<int8_t>& GetDlBitsStatus() { return dl_bits_buffer_status_; }

  inline size_t GetUlSocketSize() const { return ul_socket_buf_size_; }
  inline Table<char>& GetUlSocket() { return ul_socket_buffer_; }
  inline char* GetDlSocket() { return dl_socket_buffer_; }
  inline Table<complex_float>& GetCalibUl() { return calib_ul_buffer_; }
  inline Table<complex_float>& GetCalibDl() { return calib_dl_buffer_; }
  inline Table<complex_float>& GetCalib() { return calib_buffer_; }

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
  Table<complex_float> calib_buffer_;
  Table<int8_t> dl_mod_bits_buffer_;
  Table<int8_t> dl_bits_buffer_;
  Table<int8_t> dl_bits_buffer_status_;
  Table<std::complex<int16_t>> dl_bcast_socket_buffer_;

  Table<char> ul_socket_buffer_;
  char* dl_socket_buffer_;
  Table<complex_float> calib_ul_buffer_;
  Table<complex_float> calib_dl_buffer_;
};

// Used to communicate between the manager and the streamer/worker class
// Needs to manage its own memory
class MessageInfo {
 public:
  explicit MessageInfo(size_t tx_queue_size, size_t rx_queue_size, size_t num_socket_thread) :
      num_socket_thread(num_socket_thread) {
    tx_concurrent_queue = moodycamel::ConcurrentQueue<EventData>(tx_queue_size);
    rx_concurrent_queue = moodycamel::ConcurrentQueue<EventData>(rx_queue_size);

    for (size_t i = 0; i < num_socket_thread; i++) {
      rx_ptoks_ptr_[i] = new moodycamel::ProducerToken(rx_concurrent_queue);
      tx_ptoks_ptr_[i] = new moodycamel::ProducerToken(tx_concurrent_queue);
    }
  }
  ~MessageInfo() {
    for (size_t i = 0; i < num_socket_thread; i++) {
      delete rx_ptoks_ptr_[i];
      delete tx_ptoks_ptr_[i];
      rx_ptoks_ptr_[i] = nullptr;
      tx_ptoks_ptr_[i] = nullptr;
    }
  }

  inline moodycamel::ConcurrentQueue<EventData>* GetTxConQ() {
    return &tx_concurrent_queue;
  }
  inline moodycamel::ConcurrentQueue<EventData>* GetRxConQ() {
    return &rx_concurrent_queue;
  }
  inline moodycamel::ProducerToken** GetTxPTokPtr() { return tx_ptoks_ptr_; }
  inline moodycamel::ProducerToken** GetRxPTokPtr() { return rx_ptoks_ptr_; }
  inline moodycamel::ProducerToken* GetTxPTokPtr(size_t idx) {
    return tx_ptoks_ptr_[idx];
  }
  inline moodycamel::ProducerToken* GetRxPTokPtr(size_t idx) {
    return rx_ptoks_ptr_[idx];
  }

  inline std::queue<EventData>* GetTaskQueue(EventType event_type, size_t qid) {
    return &task_queues.at(qid).at(static_cast<size_t>(event_type));
  }

  inline std::queue<EventData>& GetCompQueue(size_t qid) {
    return complete_task_queues_.at(qid);
  }

 private:
  size_t num_socket_thread;
  // keep the concurrent queue to communicate to streamer thread
  moodycamel::ConcurrentQueue<EventData> tx_concurrent_queue;
  moodycamel::ConcurrentQueue<EventData> rx_concurrent_queue;
  moodycamel::ProducerToken* rx_ptoks_ptr_[kMaxThreads];
  moodycamel::ProducerToken* tx_ptoks_ptr_[kMaxThreads];
  
  std::array<std::array<std::queue<EventData>, kNumEventTypes>, kScheduleQueues>
    task_queues;
  std::array<std::queue<EventData>, kScheduleQueues> complete_task_queues_;
};

struct FrameInfo {
  size_t cur_sche_frame_id_;
  size_t cur_proc_frame_id_;
};

#endif  // AGORA_BUFFER_H_
