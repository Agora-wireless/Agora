#ifndef AGORA_HEAD
#define AGORA_HEAD

#include "buffer.hpp"
#include "concurrent_queue_wrapper.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "control.hpp"
#include "dycoding.hpp"
#include "dofft.hpp"
#include "doprecode.hpp"
#include "dysubcarrier.hpp"
#include "gettime.h"
#include "mac_thread.hpp"
#include "memory_manage.h"
#include "phy_stats.hpp"
#include "shared_counters.hpp"
#include "signalHandler.hpp"
#include "stats.hpp"
#include "txrx.hpp"
#include "utils.h"
#include <algorithm>
#include <iostream>
#include <memory>
#include <pthread.h>
#include <queue>
#include <signal.h>
#include <stdint.h>
#include <system_error>
#include <unistd.h>
#include <vector>

class Agora {
public:
    Agora(Config*); /// Create an Agora object and start the worker threads
    ~Agora();

    void Start(); /// The main Agora event loop
    void Stop();

private:
    void* worker(int tid); // TODO: implement later
    void* fftWorker(int tid);
    void* subcarrierWorker(int tid);
    void* decodeWorker(int tid);
    void* encodeWorker(int tid); // TODO: implement later

    void initializeQueues(); // TODO: implement later
    void initializeUplinkBuffers();
    void initializeDownlinkBuffers(); // TODO: implement later
    void freeUplinkBuffers();
    void freeDownlinkBuffers(); // TODO: implement later

    void saveDecodeDataToFile(int frame_id); // TODO: implement later
    void saveTxDataToFile(int frame_id); // TODO: implement later
    void saveLatencyDataToFile();

    void initControlInfo();

    // Flags that allow developer control over Agora internals
    struct {
        // Before exiting, save LDPC-decoded or demodulated data to a file
        bool enable_save_decode_data_to_file_ = false;

        // Before exiting, save data sent on downlink to a file
        bool enable_save_tx_data_to_file_ = false;
    } flags_;

    /// Fetch the concurrent queue for this event type
    inline moodycamel::ConcurrentQueue<EventData>* getConq(EventType event_type)
    {
        return &sched_info_arr[static_cast<size_t>(event_type)].concurrent_q_;
    }

    /// Fetch the producer token for this event type
    inline moodycamel::ProducerToken* getPtok(EventType event_type)
    {
        return sched_info_arr[static_cast<size_t>(event_type)].ptok_;
    }

    const double freq_ghz_; // RDTSC frequency in GHz

    // Worker thread i runs on core base_worker_core_offset_ + i
    size_t base_worker_core_offset_;

    Config* config_;
    std::unique_ptr<PacketTXRX> packet_tx_rx_;

    /*****************************************************
     * Buffers
     *****************************************************/

    Table<char> time_domain_iq_buffer_;
    Table<char> freq_domain_iq_buffer_to_send_;

    // Received data buffers
    // 1st dimension: number of antennas
    // 2nd dimension: kFrameWnd * kMaxSymbols * packet_size
    Table<char> freq_domain_iq_buffer_;

    // Preliminary CSI buffers. Each buffer has [number of antennas] rows and
    // [number of OFDM data subcarriers] columns.
    PtrGrid<kFrameWnd, kMaxUEs, complex_float> csi_buffer_;

    // Calculated uplink zeroforcing detection matrices. Each matrix has
    // [number of antennas] rows and [number of UEs] columns.
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float> ul_zf_matrices_;

    // Data after equalization
    // 1st dimension: TASK_BUFFER_FRAME_NUM * uplink data symbols per frame
    // 2nd dimension: number of OFDM data subcarriers * number of UEs
    Table<complex_float> equal_buffer_;

    // Data after demodulation. Each buffer has kMaxModType * number of OFDM
    // data subcarriers
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t> demod_buffer_to_send_;

    // Buffers to store demodulated data received by TX/RX threads in the
    // distributed version
    Table<int8_t> demod_buffer_to_decode_;

    // Data after LDPC decoding. Each buffer has [decoded bytes per UE] bytes.
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t> decoded_buffer_;

    // Data for IFFT
    // 1st dimension: TASK_BUFFER_FRAME_NUM * number of antennas * number of
    // data symbols per frame
    // 2nd dimension: number of OFDM carriers (including non-data carriers)
    Table<complex_float> dl_ifft_buffer_;

    // Calculated uplink zeroforcing detection matrices. Each matrix has
    // [number of UEs] rows and [number of antennas] columns.
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float> dl_zf_matrices_;

    // 1st dimension: TASK_BUFFER_FRAME_NUM
    // 2nd dimension: number of OFDM data subcarriers * number of antennas
    Table<complex_float> calib_buffer_;

    // 1st dimension: TASK_BUFFER_FRAME_NUM * number of data symbols per frame
    // 2nd dimension: number of OFDM data subcarriers * number of UEs
    Table<int8_t> dl_encoded_buffer_;

    Table<int8_t> dl_encoded_buffer_to_precode_;

    // 1st dimension: TASK_BUFFER_FRAME_NUM * number of DL data symbols per frame
    // 2nd dimension: number of OFDM data subcarriers * number of UEs
    Table<uint8_t> dl_bits_buffer_;

    // 1st dimension: number of UEs
    // 2nd dimension: number of OFDM data subcarriers * TASK_BUFFER_FRAME_NUM
    //                * number of DL data symbols per frame
    // Use different dimensions from dl_bits_buffer_ to avoid cache false sharing
    Table<uint8_t> dl_bits_buffer_status_;

    /**
     * Data for transmission
     *
     * Number of downlink socket buffers and status entries:
     * SOCKET_BUFFER_FRAME_NUM * symbol_num_perframe * BS_ANT_NUM
     *
     * Size of each downlink socket buffer entry: packet_length bytes
     * Size of each downlink socket buffer status entry: one integer
     */
    char* dl_socket_buffer_;
    int* dl_socket_buffer_status_;

    sched_info_t sched_info_arr[kMaxThreads];

    std::vector<std::thread> do_fft_threads_;

    // Threads running the subcarrier-parallel processing
    std::vector<std::thread> do_subcarrier_threads_;

    // Threads running the decoders
    std::vector<std::thread> do_decode_threads_;

    // Threads running the encoders
    std::vector<std::thread> do_encode_threads_;

    // Shared states between socket threads and dosubcarriers
    SharedState shared_state_;

#if 0
    // Shared states between dosubcarriers and doencoders
    EncodeStatus encode_status_;

    // Shared states between dosubcarriers and socket threads
    PrecodeStatus precode_status_;
#endif

    // Control info list
    std::vector<std::vector<ControlInfo>> control_info_table_;
    std::vector<size_t> control_idx_list_;
};

#endif
