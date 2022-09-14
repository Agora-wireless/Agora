/**
 * @file chsim_worker_storage.h
 * @brief Declaration file for the ChSimWorkerStorage class
 */
#ifndef CHSIM_WORKER_STORAGE_H_
#define CHSIM_WORKER_STORAGE_H_

#include <cstddef>
#include <memory>

#include "armadillo"
#include "concurrentqueue.h"
#include "logger.h"
#include "memory_manage.h"
#include "message.h"
#include "simd_types.h"
#include "udp_comm.h"

class ChSimWorkerStorage {
 public:
  ChSimWorkerStorage(size_t tid, size_t ue_ant_count, size_t bs_ant_count,
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
                    reinterpret_cast<intptr_t>(ue_input_float_storage),
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
  ~ChSimWorkerStorage() {
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

class ChSimRxBuffer {
 public:
  enum ChSimRxType { kRxTypePilotUl, kRxTypeBeaconDl };
  ChSimRxBuffer(ChSimRxType type, const Config* cfg, size_t max_frames,
                size_t max_symbols, size_t max_antennas,
                size_t symbol_size_bytes)
      : type_(type),
        cfg_(cfg),
        max_frame_(max_frames),
        storage_(max_frames, std::vector<std::vector<SimdAlignByteVector>>(
                                 max_symbols,
                                 std::vector<SimdAlignByteVector>(
                                     max_antennas,
                                     SimdAlignByteVector(symbol_size_bytes)))) {
    //Each location accessed by frame / symbol / ant will be aligned at 64
  }

  inline void Copy(size_t frame, size_t symbol, size_t ant, const short* input,
                   size_t data_size) {
    const size_t frame_idx = frame % max_frame_;
    const size_t symbol_idx = GetSymbolIdx(symbol);
    auto* dest = storage_.at(frame_idx).at(symbol_idx).at(ant).data();
    AGORA_LOG_TRACE(
        "Adding data %zu:%zu, (Frame %zu:%zu, Symbol %zu:%zu, Ant %zu)\n",
        data_size, storage_.at(frame_idx).at(symbol_idx).at(ant).size(), frame,
        frame_idx, symbol, symbol_idx, ant);
    RtAssert(data_size <= storage_.at(frame_idx).at(symbol_idx).at(ant).size(),
             "Add data must fit inside of the storage element");
    //Can make this faster (the destination is 64byte aligned), input is too
    std::memcpy(dest, input, data_size);
  }

  inline const std::byte* Read(size_t frame, size_t symbol, size_t ant) const {
    const size_t frame_idx = frame % max_frame_;
    const size_t symbol_idx = GetSymbolIdx(symbol);
    return storage_.at(frame_idx).at(symbol_idx).at(ant).data();
  }

  inline size_t GetSymbolIdx(size_t symbol_id) const {
    if (type_ == ChSimRxType::kRxTypePilotUl) {
      return cfg_->GetPilotUlIdx(symbol_id);
    } else if (type_ == ChSimRxType::kRxTypeBeaconDl) {
      return cfg_->GetBeaconDlIdx(symbol_id);
    } else {
      return SIZE_MAX;
    }
  }

 private:
  //Need the config for the symbol index functions
  const ChSimRxType type_;
  const Config* const cfg_;
  const size_t max_frame_;
  std::vector<std::vector<std::vector<SimdAlignByteVector>>> storage_;
};

class ChSimRxStorage {
 public:
  ChSimRxStorage(size_t tid, size_t core_id, size_t rx_packet_size,
                 size_t socket_offset, size_t socket_number,
                 std::vector<std::unique_ptr<UDPComm>>* udp_comm,
                 ChSimRxBuffer* rx_output_storage,
                 moodycamel::ConcurrentQueue<EventData>* response_queue)
      : tid_(tid),
        core_id_(core_id),
        rx_packet_size_(rx_packet_size),
        socket_offset_(socket_offset),
        socket_number_(socket_number),
        comm_(udp_comm),
        rx_output_(rx_output_storage),
        response_queue_(response_queue) {}

  inline size_t Id() const { return tid_; }
  inline size_t CoreId() const { return core_id_; }
  inline size_t PacketLength() const { return rx_packet_size_; }
  inline size_t SocketOffset() const { return socket_offset_; }
  inline size_t SocketNumber() const { return socket_number_; }
  inline UDPComm* Socket(size_t id) { return comm_->at(id).get(); }
  inline void TransferRxData(size_t frame, size_t symbol, size_t ant,
                             const short* input, size_t data_size) {
    return rx_output_->Copy(frame, symbol, ant, input, data_size);
  }
  inline moodycamel::ConcurrentQueue<EventData>& ResponseQueue() {
    return *response_queue_;
  }

 private:
  size_t tid_;
  size_t core_id_;
  size_t rx_packet_size_;
  size_t socket_offset_;
  size_t socket_number_;

  std::vector<std::unique_ptr<UDPComm>>* const comm_;
  ChSimRxBuffer* const rx_output_;
  moodycamel::ConcurrentQueue<EventData>* const response_queue_;
};

#endif  // CHSIM_WORKER_STORAGE_H_
