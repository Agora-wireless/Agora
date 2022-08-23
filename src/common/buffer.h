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
        ul_socket_buf_size(cfg->PacketLength() * cfg->BsAntNum() * kFrameWnd *
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
    AllocateTables(cfg);
  }
  // Delete copy constructor and copy assignment
  AgoraBuffer(AgoraBuffer const&) = delete;
  AgoraBuffer& operator=(AgoraBuffer const&) = delete;

  ~AgoraBuffer() { FreeTables(); }

  void AllocateTables(Config* const cfg) {
    // Uplink
    const size_t task_buffer_symbol_num_ul =
        cfg->Frame().NumULSyms() * kFrameWnd;

    ul_socket_buffer_.Malloc(cfg->SocketThreadNum() /* RX */,
                             ul_socket_buf_size,
                             Agora_memory::Alignment_t::kAlign64);

    fft_buffer_.Malloc(task_buffer_symbol_num_ul,
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
    ul_socket_buffer_.Free();
    fft_buffer_.Free();
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

  inline size_t GetUlSocketSize() const { return ul_socket_buf_size; }
  inline Table<char>& GetUlSocket() { return ul_socket_buffer_; }
  inline char* GetDlSocket() { return dl_socket_buffer_; }
  inline Table<complex_float>& GetCalibUl() { return calib_ul_buffer_; }
  inline Table<complex_float>& GetCalibDl() { return calib_dl_buffer_; }

 private:
  Config* const config_;
  const size_t ul_socket_buf_size;

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