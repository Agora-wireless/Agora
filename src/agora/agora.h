/**
 * @file agora.h
 * @brief Declaration file for the main agora class
 */

#ifndef AGORA_H_
#define AGORA_H_

#include <unistd.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <queue>
#include <system_error>

#include "agora_helper.h"
#include "buffer.h"
#include "concurrent_queue_wrapper.h"
#include "concurrentqueue.h"
#include "config.h"
#include "logger.h"  // TODO: remove this
#include "mac_thread_basestation.h"
#include "manager.h"
#include "mat_logger.h"
#include "memory_manage.h"
#include "packet_txrx.h"
#include "phy_stats.h"
#include "signal_handler.h"
#include "stats.h"
#include "symbols.h"
#include "utils.h"
#include "worker.h"

class Agora {
 public:
  explicit Agora(
      Config* /*cfg*/);  /// Create an Agora object and start the worker threads
  ~Agora();

  void Start();
  void Stop();
  void GetEqualData(float** ptr, int* size);

  // Flags that allow developer control over Agora internals
  struct {
    //     void getEqualData(float** ptr, int* size);Before exiting, save
    //     LDPC-decoded or demodulated data to a file
    bool enable_save_decode_data_to_file_ = false;

    // Before exiting, save data sent on downlink to a file
    bool enable_save_tx_data_to_file_ = false;
  } flags_;

 private:
  void InitializeQueues();
  void InitializeUplinkBuffers();
  void InitializeDownlinkBuffers();
  void InitializeThreads();
  void FreeQueues();
  void FreeBuffers();

  std::unique_ptr<Worker> worker_cli;

  Config* const config_;
  MessageInfo* queues_;
  SchedInfo sched_info_arr_[kScheduleQueues][kNumEventTypes];
  Buffer* buffers_;
  Counter* counters_;
  FrameInfo* frame_info_;
  Thread* threads_;

  const size_t base_worker_core_offset_;
  size_t max_equaled_frame_ = SIZE_MAX;

  std::unique_ptr<Stats> stats_;
  std::unique_ptr<PhyStats> phy_stats_;
  std::unique_ptr<PacketTxRx> packet_tx_rx_;

  // Data after equalization
  // 1st dimension: kFrameWnd * uplink data symbols per frame
  // 2nd dimension: number of OFDM data subcarriers * number of UEs
  Table<complex_float> equal_buffer_;
};

void SaveDecodeDataToFile(
    int frame_id, Config* config,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& decoded_buffer) {
  const size_t num_decoded_bytes =
      config->NumBytesPerCb(Direction::kUplink) *
      config->LdpcConfig(Direction::kUplink).NumBlocksInSymbol();

  std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  std::string filename = cur_directory + "/data/decode_data.bin";
  AGORA_LOG_INFO("Saving decode data to %s\n", filename.c_str());
  FILE* fp = std::fopen(filename.c_str(), "wb");

  for (size_t i = 0; i < config->Frame().NumULSyms(); i++) {
    for (size_t j = 0; j < config->UeAntNum(); j++) {
      int8_t* ptr = decoded_buffer[(frame_id % kFrameWnd)][i][j];
      std::fwrite(ptr, num_decoded_bytes, sizeof(uint8_t), fp);
    }
  }
  std::fclose(fp);
}

void SaveTxDataToFile(UNUSED int frame_id, Config* config,
                      char* dl_socket_buffer) {
  std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  std::string filename = cur_directory + "/data/tx_data.bin";
  AGORA_LOG_INFO("Saving Frame %d TX data to %s\n", frame_id, filename.c_str());
  FILE* fp = std::fopen(filename.c_str(), "wb");

  for (size_t i = 0; i < config->Frame().NumDLSyms(); i++) {
    size_t total_data_symbol_id = config->GetTotalDataSymbolIdxDl(frame_id, i);

    for (size_t ant_id = 0; ant_id < config->BsAntNum(); ant_id++) {
      size_t offset = total_data_symbol_id * config->BsAntNum() + ant_id;
      auto* pkt = reinterpret_cast<Packet*>(
          &dl_socket_buffer[offset * config->DlPacketLength()]);
      short* socket_ptr = pkt->data_;
      std::fwrite(socket_ptr, config->SampsPerSymbol() * 2, sizeof(short), fp);
    }
  }
  std::fclose(fp);
}

// Return a string containing the sizes of the FFT queues
// std::string GetFftQueueSizesString() {
//   std::ostringstream ret;
//   ret << "[";
//   for (size_t i = 0; i < kFrameWnd; i++) {
//     ret << std::to_string(fft_queue_arr_[i].size()) << " ";
//   }
//   ret << "]";
//   return ret.str();
// }

#endif  // AGORA_H_