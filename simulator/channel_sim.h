/**
 * @file channel_sim.h
 * @brief Declaration file for the channel simulator class
 */
#ifndef CHANNEL_SIM_H_
#define CHANNEL_SIM_H_

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "armadillo"
#include "channel.h"
#include "concurrentqueue.h"
#include "config.h"
#include "logger.h"
#include "message.h"
#include "simd_types.h"
#include "udp_comm.h"

class WorkerThreadStorage {
 public:
  WorkerThreadStorage(size_t tid, size_t ue_ant_count, size_t bs_ant_count,
                      size_t samples_per_symbol, size_t udp_packet_size)
      : tid_(tid), udp_tx_buffer_(udp_packet_size) {
    //UE
    const size_t ue_input_storage_size =
        (ue_ant_count * samples_per_symbol * sizeof(arma::cx_float));
    const size_t ue_output_storage_size =
        (bs_ant_count * samples_per_symbol * sizeof(arma::cx_float));

    auto* ue_input_float_storage = PaddedAlignedAlloc(
        Agora_memory::Alignment_t::kAlign64, ue_input_storage_size);
    ue_input_matrix_ = std::make_unique<arma::cx_fmat>(
        reinterpret_cast<arma::cx_float*>(ue_input_float_storage),
        samples_per_symbol, ue_ant_count, false, true);
    AGORA_LOG_TRACE("Ue input location %zu:%zu  diff %zu size %zu\n",
                    reinterpret_cast<intptr_t>(ue_input_matrix_->memptr()),
                    reinterpret_cast<intptr_t>(ue_input_float_storage),
                    reinterpret_cast<intptr_t>(ue_input_matrix_->memptr()) -
                        reinterpret_cast<intptr_t>(ue_input_float_storage),
                    ue_input_storage_size);

    AGORA_LOG_TRACE("storage %zu:%zu, matrix %zu:%zu memstate %d\n",
                    reinterpret_cast<intptr_t>(&ue_input_float_storage[0u]),
                    reinterpret_cast<intptr_t>(&reinterpret_cast<std::byte*>(
                        ue_input_float_storage)[ue_input_storage_size - 1]),
                    reinterpret_cast<intptr_t>(&ue_input_matrix_->at(0, 0)),
                    reinterpret_cast<intptr_t>(&ue_input_matrix_->at(
                        samples_per_symbol - 1, ue_ant_count - 1)),
                    ue_input_matrix_->mem_state);
    //Validate the memory is being reused
    RtAssert(ue_input_matrix_->memptr() == ue_input_float_storage,
             "Ue Input storage not at correct location");
    ue_input_matrix_->zeros(samples_per_symbol, ue_ant_count);

    auto* ue_output_float_storage = PaddedAlignedAlloc(
        Agora_memory::Alignment_t::kAlign64, ue_output_storage_size);
    ue_output_matrix_ = std::make_unique<arma::cx_fmat>(
        reinterpret_cast<arma::cx_float*>(ue_output_float_storage),
        samples_per_symbol, bs_ant_count, false, true);
    RtAssert(ue_output_matrix_->memptr() == ue_output_float_storage,
             "Ue Input storage not at correct location");
    ue_output_matrix_->zeros(samples_per_symbol, bs_ant_count);

    //BS
    void* bs_input_float_storage = PaddedAlignedAlloc(
        Agora_memory::Alignment_t::kAlign64, ue_output_storage_size);
    bs_input_matrix_ = std::make_unique<arma::cx_fmat>(
        reinterpret_cast<arma::cx_float*>(bs_input_float_storage),
        samples_per_symbol, bs_ant_count, false, true);
    RtAssert(bs_input_matrix_->memptr() == bs_input_float_storage,
             "Bs Input storage not at correct location");
    bs_input_matrix_->zeros(samples_per_symbol, bs_ant_count);

    void* bs_output_float_storage = PaddedAlignedAlloc(
        Agora_memory::Alignment_t::kAlign64, ue_input_storage_size);
    bs_output_matrix_ = std::make_unique<arma::cx_fmat>(
        reinterpret_cast<arma::cx_float*>(bs_output_float_storage),
        samples_per_symbol, ue_ant_count, false, true);
    RtAssert(bs_output_matrix_->memptr() == bs_output_float_storage,
             "Bs Output storage not at correct location");
    bs_output_matrix_->zeros(samples_per_symbol, ue_ant_count);
  }
  ~WorkerThreadStorage() {
    std::free(ue_input_matrix_->memptr());
    ue_input_matrix_.reset();
    std::free(ue_output_matrix_->memptr());
    ue_output_matrix_.reset();
    std::free(bs_input_matrix_->memptr());
    bs_input_matrix_.reset();
    std::free(bs_output_matrix_->memptr());
    bs_output_matrix_.reset();
  };

  inline size_t Id() const { return tid_; }
  inline arma::cx_fmat* UeInput() { return ue_input_matrix_.get(); }
  inline arma::cx_fmat* UeOutput() { return ue_output_matrix_.get(); }
  inline SimdAlignByteVector& TxBuffer() { return udp_tx_buffer_; }

  inline arma::cx_fmat* BsInput() { return bs_input_matrix_.get(); }
  inline arma::cx_fmat* BsOutput() { return bs_output_matrix_.get(); }

 private:
  size_t tid_;
  // Aligned
  std::unique_ptr<arma::cx_fmat> ue_input_matrix_;
  std::unique_ptr<arma::cx_fmat> ue_output_matrix_;

  // Aligned
  std::unique_ptr<arma::cx_fmat> bs_input_matrix_;
  std::unique_ptr<arma::cx_fmat> bs_output_matrix_;

  SimdAlignByteVector udp_tx_buffer_;
};

/**
 * @brief Simualtor for many-antenna MU-MIMO channel to work with
 * Agora BS and UE applications. It generates channel matrice(s)
 * and applies it to incoming baseband samples from BS and sends them
 * to the UE application. Similarly, applies the same channel (TDD) to
 * uplink baseband samples from UE and sends them to BS.
 */
class ChannelSim {
 public:
  ChannelSim(const Config* const config, size_t bs_thread_num,
             size_t user_thread_num, size_t worker_thread_num,
             size_t in_core_offset = 30,
             std::string in_chan_type = std::string("RAYLEIGH"),
             double in_chan_snr = 20);
  ~ChannelSim();

  void Run();

  // Loop thread receiving symbols from client antennas
  void* UeRxLoop(size_t tid);

  // Loop thread receiving symbols from BS antennas
  void* BsRxLoop(size_t tid);

  // Transmits symbol to BS antennas after applying channel
  void DoTxBs(WorkerThreadStorage& local, size_t tag);

  // Transmit symbols to client antennas after applying channel
  void DoTxUser(WorkerThreadStorage& local, size_t tag);

  void ScheduleTask(EventData do_task,
                    moodycamel::ConcurrentQueue<EventData>* in_queue,
                    moodycamel::ProducerToken const& ptok);
  void* TaskThread(size_t tid);

 private:
  void DoTx(size_t frame_id, size_t symbol_id, size_t max_ant,
            size_t ant_per_socket, const arma::cx_float* source_data,
            SimdAlignByteVector* udp_pkt_buf,
            std::vector<std::unique_ptr<UDPComm>>& udp_clients);

  // BS-facing sockets
  std::vector<std::unique_ptr<UDPComm>> bs_comm_;
  // UE-facing sockets
  std::vector<std::unique_ptr<UDPComm>> ue_comm_;

  const Config* const cfg_;
  std::unique_ptr<Channel> channel_;

  // Data buffer for received symbols from BS antennas (downlink)
  SimdAlignByteVector rx_buffer_bs_;

  // Data buffer for received symbols from client antennas (uplink)
  SimdAlignByteVector rx_buffer_ue_;

  // Task Queue for tasks related to incoming BS packets
  moodycamel::ConcurrentQueue<EventData> task_queue_bs_;

  // Task Queue for tasks related to incoming Users' packets
  moodycamel::ConcurrentQueue<EventData> task_queue_user_;

  // Master thread's message queue for event completions;
  moodycamel::ConcurrentQueue<EventData> message_queue_;
  std::array<std::unique_ptr<moodycamel::ProducerToken>, kMaxThreads>
      task_ptok_;

  std::vector<std::thread> task_threads_;

  size_t ul_data_plus_pilot_symbols_;
  size_t dl_data_plus_beacon_symbols_;
  size_t payload_length_;

  size_t bs_thread_num_;
  size_t user_thread_num_;
  size_t bs_socket_num_;
  size_t user_socket_num_;
  size_t worker_thread_num_;
  size_t core_offset_;

  std::string channel_type_;
  double channel_snr_;

  //size_t* bs_rx_counter_;
  std::unique_ptr<size_t[]> bs_rx_counter_;
  std::unique_ptr<size_t[]> user_rx_counter_;
  std::array<size_t, kFrameWnd> bs_tx_counter_;
  std::array<size_t, kFrameWnd> user_tx_counter_;

  //Returns Beacon+Dl symbol index
  inline size_t GetBsDlIdx(size_t symbol_id) const {
    size_t symbol_idx = SIZE_MAX;
    const auto type = cfg_->GetSymbolType(symbol_id);
    if (type == SymbolType::kBeacon) {
      symbol_idx = cfg_->Frame().GetBeaconSymbolIdx(symbol_id);
    } else if (type == SymbolType::kDL) {
      symbol_idx = cfg_->Frame().GetDLSymbolIdx(symbol_id) +
                   cfg_->Frame().NumBeaconSyms();
    } else {
      throw std::runtime_error("Invalid BS Beacon or DL symbol id");
    }
    return symbol_idx;
  }

  //Returns Pilot+Ul symbol index
  inline size_t GetUeUlIdx(size_t symbol_id) const {
    size_t symbol_idx = SIZE_MAX;
    const auto type = cfg_->GetSymbolType(symbol_id);
    if (type == SymbolType::kPilot) {
      symbol_idx = cfg_->Frame().GetPilotSymbolIdx(symbol_id);
    } else if (type == SymbolType::kUL) {
      symbol_idx = cfg_->Frame().GetULSymbolIdx(symbol_id) +
                   cfg_->Frame().NumPilotSyms();
    } else {
      throw std::runtime_error("Invalid Ue Pilot or UL symbol id");
    }
    return symbol_idx;
  }
};

#endif  // CHANNEL_SIM_H_
