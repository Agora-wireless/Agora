/**
 * @file buffer.h
 * @brief Declaration file for the buffer class
 */

#ifndef BUFFER_H_
#define BUFFER_H_

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
  explicit AgoraBuffer(Config* const cfg)
      : config_(cfg),
        socket_buffer_size_(cfg->PacketLength() * cfg->BsAntNum() * kFrameWnd *
                            cfg->Frame().NumTotalSyms()),
        csi_buffer_(kFrameWnd, cfg->UeAntNum(),
                    cfg->BsAntNum() * cfg->OfdmDataNum()),
        ul_beam_matrix_(kFrameWnd, cfg->OfdmDataNum(),
                        cfg->BsAntNum() * cfg->UeAntNum()),
        dl_beam_matrix_(kFrameWnd, cfg->OfdmDataNum(),
                        cfg->UeAntNum() * cfg->BsAntNum()),
        demod_buffer_(kFrameWnd, cfg->Frame().NumULSyms(), cfg->UeAntNum(),
                      kMaxModType * cfg->OfdmDataNum()),
        decoded_buffer_(
            kFrameWnd, cfg->Frame().NumULSyms(), cfg->UeAntNum(),
            cfg->LdpcConfig(Direction::kUplink).NumBlocksInSymbol() *
                Roundup<64>(cfg->NumBytesPerCb(Direction::kUplink))) {
    this->AllocateTables(cfg);
  }

  ~AgoraBuffer() { this->FreeTables(); }

  void AllocateTables(Config* const cfg) {
    // Uplink
    const size_t task_buffer_symbol_num_ul =
        cfg->Frame().NumULSyms() * kFrameWnd;

    socket_buffer_.Malloc(cfg->SocketThreadNum() /* RX */, socket_buffer_size_,
                          Agora_memory::Alignment_t::kAlign64);

    data_buffer_.Malloc(task_buffer_symbol_num_ul,
                        cfg->OfdmDataNum() * cfg->BsAntNum(),
                        Agora_memory::Alignment_t::kAlign64);

    equal_buffer_.Malloc(task_buffer_symbol_num_ul,
                         cfg->OfdmDataNum() * cfg->UeAntNum(),
                         Agora_memory::Alignment_t::kAlign64);
    ue_spec_pilot_buffer_.Calloc(
        kFrameWnd, cfg->Frame().ClientUlPilotSymbols() * cfg->UeAntNum(),
        Agora_memory::Alignment_t::kAlign64);

    // Downlink
    if (cfg->Frame().NumDLSyms() > 0) {
      const size_t task_buffer_symbol_num =
          cfg->Frame().NumDLSyms() * kFrameWnd;

      size_t dl_socket_buffer_status_size =
          cfg->BsAntNum() * task_buffer_symbol_num;
      size_t dl_socket_buffer_size =
          cfg->DlPacketLength() * dl_socket_buffer_status_size;
      AllocBuffer1d(&dl_socket_buffer_, dl_socket_buffer_size,
                    Agora_memory::Alignment_t::kAlign64, 1);

      size_t dl_bits_buffer_size =
          kFrameWnd * cfg->MacBytesNumPerframe(Direction::kDownlink);
      dl_bits_buffer_.Calloc(cfg->UeAntNum(), dl_bits_buffer_size,
                             Agora_memory::Alignment_t::kAlign64);
      dl_bits_buffer_status_.Calloc(cfg->UeAntNum(), kFrameWnd,
                                    Agora_memory::Alignment_t::kAlign64);

      dl_ifft_buffer_.Calloc(cfg->BsAntNum() * task_buffer_symbol_num,
                             cfg->OfdmCaNum(),
                             Agora_memory::Alignment_t::kAlign64);
      calib_dl_buffer_.Malloc(kFrameWnd, cfg->BfAntNum() * cfg->OfdmDataNum(),
                              Agora_memory::Alignment_t::kAlign64);
      calib_ul_buffer_.Malloc(kFrameWnd, cfg->BfAntNum() * cfg->OfdmDataNum(),
                              Agora_memory::Alignment_t::kAlign64);
      calib_dl_msum_buffer_.Malloc(kFrameWnd,
                                   cfg->BfAntNum() * cfg->OfdmDataNum(),
                                   Agora_memory::Alignment_t::kAlign64);
      calib_ul_msum_buffer_.Malloc(kFrameWnd,
                                   cfg->BfAntNum() * cfg->OfdmDataNum(),
                                   Agora_memory::Alignment_t::kAlign64);
      //initialize the calib buffers
      const complex_float complex_init = {0.0f, 0.0f};
      //const complex_float complex_init = {1.0f, 0.0f};
      for (size_t frame = 0u; frame < kFrameWnd; frame++) {
        for (size_t i = 0; i < (cfg->OfdmDataNum() * cfg->BfAntNum()); i++) {
          calib_dl_buffer_[frame][i] = complex_init;
          calib_ul_buffer_[frame][i] = complex_init;
          calib_dl_msum_buffer_[frame][i] = complex_init;
          calib_ul_msum_buffer_[frame][i] = complex_init;
        }
      }
      dl_mod_bits_buffer_.Calloc(
          task_buffer_symbol_num,
          Roundup<64>(cfg->GetOFDMDataNum()) * cfg->UeAntNum(),
          Agora_memory::Alignment_t::kAlign64);
    }
  }

  void FreeTables() {
    // Uplink
    socket_buffer_.Free();
    data_buffer_.Free();
    equal_buffer_.Free();
    ue_spec_pilot_buffer_.Free();

    // Downlink
    if (config_->Frame().NumDLSyms() > 0) {
      FreeBuffer1d(&dl_socket_buffer_);
      dl_ifft_buffer_.Free();
      calib_dl_buffer_.Free();
      calib_ul_buffer_.Free();
      calib_dl_msum_buffer_.Free();
      calib_ul_msum_buffer_.Free();
      dl_mod_bits_buffer_.Free();
      dl_bits_buffer_.Free();
      dl_bits_buffer_status_.Free();
    }
  }

  // Get functions
  size_t GetSocketBufferSize() { return this->socket_buffer_size_; }
  complex_float* GetCsiBuffer(size_t x, size_t y) {
    return this->csi_buffer_[x][y];
  }
  complex_float* GetUlBeamMatrix(size_t x, size_t y) {
    return this->ul_beam_matrix_[x][y];
  }
  complex_float* GetDlBeamMatrix(size_t x, size_t y) {
    return this->dl_beam_matrix_[x][y];
  }
  int8_t* GetDemodBuffer(size_t x, size_t y, size_t z) {
    return this->demod_buffer_[x][y][z];
  }
  int8_t* GetDecodedBuffer(size_t x, size_t y, size_t z) {
    return this->decoded_buffer_[x][y][z];
  }
  complex_float* GetDataBuffer(size_t x) { return this->data_buffer_[x]; }
  complex_float* GetEqualBuffer(size_t x) { return this->equal_buffer_[x]; }
  complex_float* GetUeSpecPilotBuffer(size_t x) {
    return this->ue_spec_pilot_buffer_[x];
  }
  complex_float* GetDlIfftBuffer(size_t x) { return this->dl_ifft_buffer_[x]; }
  complex_float* GetCalibUlBuffer(size_t x) {
    return this->calib_ul_buffer_[x];
  }
  complex_float* GetCalibDlBuffer(size_t x) {
    return this->calib_dl_buffer_[x];
  }
  complex_float* GetCalibUlMsumBuffer(size_t x) {
    return this->calib_ul_msum_buffer_[x];
  }
  complex_float* GetCalibDlMsumBuffer(size_t x) {
    return this->calib_dl_msum_buffer_[x];
  }
  int8_t* GetDlModBitsBuffer(size_t x) { return this->dl_mod_bits_buffer_[x]; }

  int8_t* GetDlBitsBuffer(size_t x) { return this->dl_bits_buffer_[x]; }
  int8_t* GetDlBitsBufferStatus(size_t x) {
    return this->dl_bits_buffer_status_[x];
  }
  char* GetDlSocketBuffer() { return this->dl_socket_buffer_; }

  // Set functions
  void SetDlBitsBufferStatus(int8_t value, size_t x, size_t y) {
    this->dl_bits_buffer_status_[x][y] = value;
  }

  // Delete copy constructor and copy assignment
  AgoraBuffer(AgoraBuffer const&) = delete;
  AgoraBuffer& operator=(AgoraBuffer const&) = delete;

  // TX RX Buffers
  // Direct access is allowed for packetTXRX classes
  Table<char> socket_buffer_;
  char* dl_socket_buffer_;
  Table<complex_float> calib_ul_buffer_;
  Table<complex_float> calib_dl_buffer_;

 private:
  Config* const config_;
  const size_t socket_buffer_size_;

  PtrGrid<kFrameWnd, kMaxUEs, complex_float> csi_buffer_;
  PtrGrid<kFrameWnd, kMaxDataSCs, complex_float> ul_beam_matrix_;
  PtrGrid<kFrameWnd, kMaxDataSCs, complex_float> dl_beam_matrix_;
  PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t> demod_buffer_;
  PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t> decoded_buffer_;
  Table<complex_float> data_buffer_;
  Table<complex_float> equal_buffer_;
  Table<complex_float> ue_spec_pilot_buffer_;
  Table<complex_float> dl_ifft_buffer_;
  Table<complex_float> calib_ul_msum_buffer_;
  Table<complex_float> calib_dl_msum_buffer_;
  Table<int8_t> dl_mod_bits_buffer_;
  Table<int8_t> dl_bits_buffer_;
  Table<int8_t> dl_bits_buffer_status_;
};

struct SchedInfo {
  moodycamel::ConcurrentQueue<EventData> concurrent_q_;
  moodycamel::ProducerToken* ptok_;
};

// Used to communicate between the manager and the worker class
struct MessageInfo {
  moodycamel::ConcurrentQueue<EventData> complete_task_queue_[kScheduleQueues];
  moodycamel::ProducerToken* worker_ptoks_ptr_[kMaxThreads][kScheduleQueues];
  SchedInfo sched_info_arr_[kScheduleQueues][kNumEventTypes];
};

struct FrameInfo {
  size_t cur_sche_frame_id_;
  size_t cur_proc_frame_id_;
};

#endif  // BUFFER_H_