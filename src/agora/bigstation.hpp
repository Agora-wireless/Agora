#pragma once

#include "config.hpp"
#include "dycoding.hpp"
#include "dofft.hpp"
#include "dyprecode.hpp"
#include "dysubcarrier.hpp"
#include "gettime.h"
#include "shared_counters.hpp"
#include "signalHandler.hpp"
#include "txrx.hpp"
#include "utils.h"
#include <iostream>
#include <memory>
#include <pthread.h>
#include <queue>
#include <signal.h>
#include <stdint.h>
#include <system_error>
#include <unistd.h>
#include <vector>

class BigStation
{
public:
    BigStation(Config*);
    ~BigStation();

    void Start();
    void Stop();

private:
    void fftWorker(int tid);
    void zfWorker(int tid);
    void demulWorker(int tid);
    void decodeWorker(int tid);

    void runCsi(size_t frame_id, size_t base_sc_id, size_t sc_block_size);

    void initializeBigStationBuffers();
    void freeBigStationBuffers();

    const double freq_ghz_; // RDTSC frequency in GHz

    // Worker thread i runs on core base_worker_core_offset_ + i
    size_t base_worker_core_offset_;

    Config* config_;
    std::unique_ptr<BigStationTXRX> bigstation_tx_rx_;

    std::vector<std::thread> do_fft_threads_;
    std::vector<std::thread> do_zf_threads_;
    std::vector<std::thread> do_demul_threads_;
    std::vector<std::thread> do_decode_threads_;

    BigStationState bigstation_state_;

    // Buffers for the BigStation mode
    PtrCube<kFrameWnd, kMaxSymbols, kMaxAntennas, uint8_t> time_iq_buffer_;
    PtrGrid<kFrameWnd, kMaxAntennas, uint8_t> pilot_buffer_;
    PtrCube<kFrameWnd, kMaxSymbols, kMaxAntennas, uint8_t> freq_iq_buffer_to_send_;
    PtrCube<kFrameWnd, kMaxSymbols, kMaxAntennas, uint8_t> data_buffer_;
    PtrGrid<kFrameWnd, kMaxDataSCs, uint8_t> post_zf_buffer_to_send_;
    PtrGrid<kFrameWnd, kMaxDataSCs, uint8_t> post_zf_buffer_;
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t> post_demul_buffer_to_send_;
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t> post_demul_buffer_;
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t> post_decode_buffer_;
};