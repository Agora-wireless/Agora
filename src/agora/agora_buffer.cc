/**
 * @file agora_buffer.h
 * @brief Defination file for the AgoraBuffer class
 */
#include "agora_buffer.h"

AgoraBuffer::AgoraBuffer(Config* const cfg)
    : config_(cfg),
      ul_socket_buf_size_(cfg->PacketLength() * cfg->BsAntNum() * kFrameWnd *
                          cfg->Frame().NumTotalSyms()),
      csi_buffer_(kFrameWnd, cfg->UeAntNum(),
                  cfg->BsAntNum() * cfg->OfdmDataNum()),
      ul_beam_matrix_(kFrameWnd, cfg->OfdmDataNum(),
                      cfg->BsAntNum() * cfg->UeAntNum()),
      dl_beam_matrix_(kFrameWnd, cfg->OfdmDataNum(),
                      cfg->UeAntNum() * cfg->BsAntNum()),
      demod_buffer_(kFrameWnd, cfg->Frame().NumULSyms(), cfg->UeAntNum(),
                    kMaxModType * cfg->OfdmDataNum()),
      decoded_buffer_(kFrameWnd, cfg->Frame().NumULSyms(), cfg->UeAntNum(),
                      cfg->LdpcConfig(Direction::kUplink).NumBlocksInSymbol() *
                          Roundup<64>(cfg->NumBytesPerCb(Direction::kUplink))) {
  AllocateTables();
}

AgoraBuffer::~AgoraBuffer() { FreeTables(); }

void AgoraBuffer::AllocateTables() {
  // Uplink
  const size_t task_buffer_symbol_num_ul =
      config_->Frame().NumULSyms() * kFrameWnd;

  ul_socket_buffer_.Malloc(config_->SocketThreadNum() /* RX */,
                           ul_socket_buf_size_,
                           Agora_memory::Alignment_t::kAlign64);

  fft_buffer_.Malloc(task_buffer_symbol_num_ul,
                     config_->OfdmDataNum() * config_->BsAntNum(),
                     Agora_memory::Alignment_t::kAlign64);

  equal_buffer_.Malloc(task_buffer_symbol_num_ul,
                       config_->OfdmDataNum() * config_->UeAntNum(),
                       Agora_memory::Alignment_t::kAlign64);
  ue_spec_pilot_buffer_.Calloc(
      kFrameWnd, config_->Frame().ClientUlPilotSymbols() * config_->UeAntNum(),
      Agora_memory::Alignment_t::kAlign64);

  // Downlink
  if (config_->Frame().NumDLSyms() > 0) {
    const size_t task_buffer_symbol_num =
        config_->Frame().NumDLSyms() * kFrameWnd;

    size_t dl_socket_buffer_status_size =
        config_->BsAntNum() * task_buffer_symbol_num;
    size_t dl_socket_buffer_size =
        config_->DlPacketLength() * dl_socket_buffer_status_size;
    AllocBuffer1d(&dl_socket_buffer_, dl_socket_buffer_size,
                  Agora_memory::Alignment_t::kAlign64, 1);

    size_t dl_bits_buffer_size =
        kFrameWnd * config_->MacBytesNumPerframe(Direction::kDownlink);
    dl_bits_buffer_.Calloc(config_->UeAntNum(), dl_bits_buffer_size,
                           Agora_memory::Alignment_t::kAlign64);
    dl_bits_buffer_status_.Calloc(config_->UeAntNum(), kFrameWnd,
                                  Agora_memory::Alignment_t::kAlign64);

    dl_ifft_buffer_.Calloc(config_->BsAntNum() * task_buffer_symbol_num,
                           config_->OfdmCaNum(),
                           Agora_memory::Alignment_t::kAlign64);
    calib_dl_buffer_.Malloc(kFrameWnd,
                            config_->BfAntNum() * config_->OfdmDataNum(),
                            Agora_memory::Alignment_t::kAlign64);
    calib_ul_buffer_.Malloc(kFrameWnd,
                            config_->BfAntNum() * config_->OfdmDataNum(),
                            Agora_memory::Alignment_t::kAlign64);
    calib_dl_msum_buffer_.Malloc(kFrameWnd,
                                 config_->BfAntNum() * config_->OfdmDataNum(),
                                 Agora_memory::Alignment_t::kAlign64);
    calib_ul_msum_buffer_.Malloc(kFrameWnd,
                                 config_->BfAntNum() * config_->OfdmDataNum(),
                                 Agora_memory::Alignment_t::kAlign64);
    //initialize the calib buffers
    const complex_float complex_init = {0.0f, 0.0f};
    //const complex_float complex_init = {1.0f, 0.0f};
    for (size_t frame = 0u; frame < kFrameWnd; frame++) {
      for (size_t i = 0; i < (config_->OfdmDataNum() * config_->BfAntNum());
           i++) {
        calib_dl_buffer_[frame][i] = complex_init;
        calib_ul_buffer_[frame][i] = complex_init;
        calib_dl_msum_buffer_[frame][i] = complex_init;
        calib_ul_msum_buffer_[frame][i] = complex_init;
      }
    }
    dl_mod_bits_buffer_.Calloc(
        task_buffer_symbol_num,
        Roundup<64>(config_->GetOFDMDataNum()) * config_->UeAntNum(),
        Agora_memory::Alignment_t::kAlign64);
  }
}

void AgoraBuffer::FreeTables() {
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
