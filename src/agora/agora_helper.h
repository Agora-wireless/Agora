#include <unistd.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <queue>
#include <system_error>

#include "concurrent_queue_wrapper.h"
#include "concurrentqueue.h"
#include "config.h"
#include "memory_manage.h"
#include "stats.h"
#include "symbols.h"

struct SchedInfoT {
  moodycamel::ConcurrentQueue<EventData> concurrent_q;
  moodycamel::ProducerToken* ptok;
};

// Fetch the concurrent queue for this event type
moodycamel::ConcurrentQueue<EventData>* GetConq(
    SchedInfoT sched_info_arr[kScheduleQueues][kNumEventTypes],
    EventType event_type, size_t qid) {
  return &sched_info_arr[qid][static_cast<size_t>(event_type)].concurrent_q;
}

// Fetch the producer token for this event type
moodycamel::ProducerToken* GetPtok(
    SchedInfoT sched_info_arr[kScheduleQueues][kNumEventTypes],
    EventType event_type, size_t qid) {
  return sched_info_arr[qid][static_cast<size_t>(event_type)].ptok;
}

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