# Scheduler Instruction

## Environment set up

1. Create a new directory `Agora/src/scheduler` and put source code of `scheduler` and `throughput` into it.
2. Add dependencies to `CMakeLists.txt`: add `src/scheduler/throughput.cc` to the set of `MAC_CLIENT_SOURCES` `MAC_BS_SOURCES `; add `src/scheduler/` to `include_directories`; add `src/scheduler/scheduler.cc` to the set of `AGORA_SOURCES `.
3. Put `#include "throughput.h"` at the beginning of `mac_receiver.h` and put `#include "scheduler.h"` at the beginning of `doer.h`

## Enable Throughput

Throughput class is designed to compute and record the real throughput of data received on MAC layer (corrupted data will be filtered during PHY->MAC)

1. Add a new private variable `Throughput* tp` to `MacReceiver` in `mac_receiver.h`.

2. Initialize `tp` in `MacReceiver::StartRecv` by calling `tp = new Throughput(rx_thread_num_);`

   ```c++
   std::vector<std::thread> MacReceiver::StartRecv() {
     std::vector<std::thread> created_threads;
   
     // init Throughput calculator
     tp = new Throughput(rx_thread_num_);
   
     std::printf("MacReceiver:  Start Recv threads %zu\n", rx_thread_num_);
     created_threads.resize(rx_thread_num_);
   
     for (size_t i = 0; i < rx_thread_num_; i++) {
       created_threads.at(i) = std::thread(&MacReceiver::LoopRecv, this, i);
     }
     return created_threads;
   } 
   ```

   In `LoopRecv` record the timestamp by calling `tp->Stamp(tid, recvlen)` each loop after receive message from UDP server, and delete `tp` after the first thread exit.

   ```c++
   void* MacReceiver::LoopRecv(size_t tid) {
     /* ..... */
     while ((SignalHandler::GotExitSignal() == false) &&
            (cfg_->Running() == true)) {
       ssize_t recvlen = udp_server->RecvFrom(&rx_buffer[0u], max_packet_length,
                                              phy_address_, phy_port_ + ue_id);
       // throughput timestamp here
       tp->Stamp(tid, recvlen);
   	
       /* ..... */
     }
     if (!tid) { // first thread delete throughput
       delete tp;
     }
     delete[] rx_buffer;
     std::printf("MacReceiver: Finished\n");
     return nullptr;
   }
   
   ```

## Enable Scheduler

Scheduler class is designed to select a subset of users every time Agora beamforms to reduce the pressure on the channel and optimize the quality of experience. 

**Note: only MKL version is supported.**

Scheduler is only used by `dozf` `doprecode`. For simplicity, its header file is included in `doer`.

1. Add a new private variable `Scheduler *scheduler_` to `DoPrecode` `DoZF`.

2. Create new constructors for `DoPrecode` `DoZF` which takes in `Scheduler *scheduler` so that Agora can pass the variable when they are initialized.

3. In `agora.h`, add a new private variable `std::unique_ptr<Scheduler> scheduler` to `Agora` and in `agora.cc`, initialize the scheduler

   ```c++
   	// Create scheduler
   	scheduler = std::make_unique<Scheduler>(cfg, csi_buffers_);
   ```

4. Call `scheduler.get()->UpdateQueue(frame_id); // update data queues` at the `last_tx_symbol` of `EventType:kPacketTX`. And call `scheduler.get()->Launch(frame_id); // launch scheduler` at the `last_pilot_fft` of `HandleEventFft`.

5. In `Agora::Worker`, modify `compute_zf` `compute_precode` as 

   ```c++
   auto compute_zf = std::make_unique<DoZF>(
         this->config_, tid, this->csi_buffers_, calib_dl_buffer_,
         calib_ul_buffer_, this->calib_dl_msum_buffer_,
         this->calib_ul_msum_buffer_, this->ul_zf_matrices_, this->dl_zf_matrices_,
         this->phy_stats_.get(), this->stats_.get(), scheduler.get()); // add input scheduler
   
   auto compute_precode = std::make_unique<DoPrecode>(
         this->config_, tid, this->dl_zf_matrices_, this->dl_ifft_buffer_,
         this->dl_encoded_buffer_, this->stats_.get(), scheduler.get()); // add input scheduler
   ```

Now Agora will initialize a scheduler and shares with `dozf` `doprecode`. To fully enable scheduling, we have to call functions to modify the buffer.

1. In `dozf`, add a new function to compute precoder, because `DoZF::ComputePrecoder` takes limited input, the new one should be like

   ```c++
   float DoZF::ComputePrecoderSchedule(const arma::cx_fmat& mat_csi, const arma::cx_fmat& sche_mat_csi,complex_float* calib_ptr, complex_float* _mat_ul_zf, complex_float* _mat_dl_zf) { // new parameter sche_mat_csi
       /* ..... */
       if (kUseUlZfForDownlink == true) { // if true, never schedule
         // With orthonormal calib matrix:
         // pinv(calib * csi) = pinv(csi)*inv(calib)
         // This probably causes a performance hit since we are throwing
         // magnitude info away by taking the sign of the calibration matrix
         arma::cx_fmat calib_mat = arma::diagmat(arma::sign(calib_vec));
         mat_dl_zf_tmp = mat_ul_zf_tmp * inv(calib_mat);
       } else {
         arma::cx_fmat mat_dl_csi = arma::diagmat(calib_vec) * sche_mat_csi; // schedule dl here
         /* ..... */
       }
       /* ..... */
     return rcond;
   }
   ```

   Then in `DoZF::ZfTimeOrthogonal` , modify as 

   ```c++
   /* ..... */
   	double start_tsc3 = GetTime::WorkerRdtsc();
       duration_stat_->task_duration_[2] += start_tsc3 - start_tsc2;
   
       arma::cx_fmat sche_mat_csi;
       this->scheduler_->ScheduleCSI(frame_slot, cur_sc_id, sche_mat_csi, mat_csi);
   
       auto rcond = ComputePrecoderSchedule(mat_csi, sche_mat_csi, calib_gather_buffer_, ul_zf_matrices_[frame_slot][cur_sc_id], dl_zf_matrices_[frame_slot][cur_sc_id]);
   
   /* ..... */
   ```

2. Then in `doprecode`, modify the constructor as

   ```c++
   /* ..... */
   #if USE_MKL_JIT
     MKL_Complex8 alpha = {1, 0}; 
     MKL_Complex8 beta = {0, 0};
     // Input: A: BsAntNum() x UeAntNum() , B: UeAntNum() x 1
     // Output: C: BsAntNum() x 1
     // Leading dimensions: A: bs_ant_num(), B: ue_num(), C: bs_ant_num()
     size_t dim = (scheduler_ == nullptr ? cfg_->UeNum() : scheduler_->GetSelectNum());
     std::cout << "Dim: " << dim << "\n";
     mkl_jit_status_t status = mkl_jit_create_cgemm(
         &jitter_, MKL_COL_MAJOR, MKL_NOTRANS, MKL_NOTRANS, cfg_->BsAntNum(), 1,
         dim, &alpha, cfg_->BsAntNum(), dim, &beta,
         cfg_->BsAntNum());
   
     if (MKL_JIT_ERROR == status) {
       std::fprintf(
           stderr,
           "Error: insufficient memory to JIT and store the DGEMM kernel\n");
       throw std::runtime_error(
           "DoPrecode: insufficient memory to JIT and store the DGEMM kernel");
     }
     my_cgemm_ = mkl_jit_get_cgemm_ptr(jitter_);
   #endif
   /* ..... */
   ```

   to correct the dimension of GEMM of MKL based on the number of selected users.

   Then modify `DoPrecode::Launch` as 

   ```c++
   /* ..... */
   for (size_t user_id = 0; user_id < cfg_->UeAntNum(); user_id++) {
           size_t total_symbol_idx =
                 cfg_->GetTotalDataSymbolIdxDl(scheduler_->GetFrameID(user_id), symbol_idx_dl); // use frame_id in queue
           for (size_t j = 0; j < kSCsPerCacheline; j++) {
             LoadInputData(symbol_idx_dl, total_symbol_idx, user_id,
                           base_sc_id + i + j, j);
           }
         }
   
         size_t start_tsc2 = GetTime::WorkerRdtsc();
         duration_stat_->task_duration_[1] += start_tsc2 - start_tsc1;
         for (size_t j = 0; j < kSCsPerCacheline; j++) {
           PrecodingPerSc(frame_slot, base_sc_id + i + j, i + j);
         }
         duration_stat_->task_count_ =
             duration_stat_->task_count_ + kSCsPerCacheline;
         duration_stat_->task_duration_[2] += GetTime::WorkerRdtsc() - start_tsc2;
       }
     } else {
       for (size_t i = 0; i < max_sc_ite; i++) {
         size_t start_tsc1 = GetTime::WorkerRdtsc();
         int cur_sc_id = base_sc_id + i;
         for (size_t user_id = 0; user_id < cfg_->UeAntNum(); user_id++) {
           size_t total_symbol_idx =
                 cfg_->GetTotalDataSymbolIdxDl(scheduler_->GetFrameID(user_id), symbol_idx_dl); // use frame_id in queue
           LoadInputData(symbol_idx_dl, total_symbol_idx, user_id, cur_sc_id,
                         0);
         }
   /* ..... */
   ```

   and also in `DoPrecode::PrecodingPerSc`

   ```c++
   void DoPrecode::PrecodingPerSc(size_t frame_slot, size_t sc_id,
                                  size_t sc_id_in_block) {
     auto* precoder_ptr = reinterpret_cast<arma::cx_float*>(
         dl_zf_matrices_[frame_slot][cfg_->GetZfScId(sc_id)]);
     auto* data_ptr = reinterpret_cast<arma::cx_float*>(
         modulated_buffer_temp_ +
         (kUseSpatialLocality ? (sc_id_in_block % kSCsPerCacheline * cfg_->UeAntNum())
              : 0));
   
     arma::cx_fmat mat_data(data_ptr, cfg_->UeNum(), 1, false);
     arma::cx_fmat dst_mat;
     scheduler_->ScheduleDATA(frame_slot, cfg_->GetZfScId(sc_id), dst_mat, mat_data); // schedule data
     // std::cout << "***********data_select size: " << dst_mat.size() << "\n";
     auto* dst_ptr = dst_mat.begin();
   
     auto* precoded_ptr = reinterpret_cast<arma::cx_float*>(
         precoded_buffer_temp_ + sc_id_in_block * cfg_->BsAntNum());
   
   #if USE_MKL_JIT
     my_cgemm_(jitter_, (MKL_Complex8*)precoder_ptr, (MKL_Complex8*)dst_ptr,
               (MKL_Complex8*)precoded_ptr); // use dst_ptr
   #else
   /* ..... */
   ```

   

