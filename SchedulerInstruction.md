# USE_MKL_JIT has been disabled

# Scheduler Instruction

## Environment set up

1. Create a new directory `Agora/src/scheduler` and put source code of `scheduler` and `throughput` into it.
2. Create a new directory `Agora/log` and output file of throughput will be stored here. 
3. Add dependencies to `CMakeLists.txt`: add `src/scheduler/throughput.cc` to the set of `MAC_CLIENT_SOURCES` `MAC_BS_SOURCES `; add `src/scheduler/` to `include_directories`; add `src/scheduler/scheduler.cc` to the set of `AGORA_SOURCES `.
4. Put `#include "throughput.h"` at the beginning of `mac_receiver.h` and put `#include "scheduler.h"` at the beginning of `config.h`

## Enable Throughput

All modifications in the code is **marked with `// Throughput: xxx`.**

Throughput class is designed to compute and record the real throughput of data **received** on MAC layer (corrupted data will be filtered during PHY->MAC)

1. Add a new private variable `Throughput* tp` to `MacReceiver` in `mac_receiver.h`.

2. Initialize `tp` in `MacReceiver::StartRecv` by calling `tp = new Throughput(rx_thread_num_);`

   ```c++
   std::vector<std::thread> MacReceiver::StartRecv() {
     std::vector<std::thread> created_threads;
   
     // Throughput: init
     tp = new Throughput(rx_thread_num_);
   
     MLPD_INFO("MacReceiver:  Start Recv threads %zu\n", rx_thread_num_);
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
       // Throughput: timestamp here
       tp->Stamp(tid, recvlen);
   	
       /* ..... */
     }
       
     // Throughput: free memory
     if (tid == 0) {
       delete tp;
     }
     // Throughput
       
     delete[] rx_buffer;
     std::printf("MacReceiver: Finished\n");
     return nullptr;
   }
   
   ```

## Enable Scheduler

All modifications in the code is **marked with `// QMACS: xxx`.**

Scheduler class is designed to schedule the downlink stream to reduce the pressure on the channel and optimize the quality of experience. In each frame the scheduler will select a subset of UEs to receive data and developer can customize the scheduling algorithm.

**Note: only MKL version is supported.**

**Note: only TimeOrthogonal mode has been tested.**

**Note: `kUseUlZfForDownlink = false` (dozf.cc) must be held because we don't schedule UL ** 

Scheduler is shared by `agora` `dozf` `doprecode`. For simplicity, the scheduler's pointer is stored in `config` class, which can be called by `config_->scheduler_`(agora) or `cfg_->scheduler_`(doer).

1. Add a new public variable `Scheduler *scheduler_` to `Config`.

2. In `agora.h`, add a new private variable `std::unique_ptr<Scheduler> scheduler` to `Agora` and in `agora.cc`, initialize the scheduler in the constructor and destructor,

   ```c++
   /* ..... */
     // Create worker threads
     CreateThreads();
   
     // QMACS: Create Scheduler
     config_->scheduler_ = new Scheduler (config_->UeNum(),
                                         config_->Frame().NumDLSyms(),
                                         config_->UeAntNum(), 
                                         csi_buffers_);
   
     MLPD_INFO(
         "Master thread core %zu, TX/RX thread cores %zu--%zu, worker thread "
         "cores %zu--%zu\n",
         cfg->CoreOffset(), cfg->CoreOffset() + 1,
         cfg->CoreOffset() + 1 + cfg->SocketThreadNum() - 1,
         base_worker_core_offset_,
         base_worker_core_offset_ + cfg->WorkerThreadNum() - 1);
   }
   
   Agora::~Agora() {
     if (kEnableMac == true) {
       mac_std_thread_.join();
     }
   
     // QMACS: free scheduler
     delete config_->scheduler_;
   /* ..... */
   ```

3. Call `config_->scheduler_->UpdateQueue(frame_id);` at the `last_tx_symbol` of `EventType:kPacketTX`. And call `config_->scheduler_->Launch(frame_id);` at the `last_pilot_fft` of `HandleEventFft`.

   ```c++
   case EventType::kPacketTX: {
             /* ..... */
               bool last_tx_symbol = this->tx_counters_.CompleteSymbol(frame_id);
               if (last_tx_symbol == true) {
                 this->stats_->MasterSetTsc(TsType::kTXDone, frame_id);
                 PrintPerFrameDone(PrintType::kPacketTX, frame_id);
   
                 // QMACS: update data queue
                 config_->scheduler_->UpdateQueue(frame_id);
   
                 bool work_finished = this->CheckFrameComplete(frame_id);
                 if (work_finished == true) {
                   goto finish;
                 }
               }
       /* ..... */
                 
   void Agora::HandleEventFft(size_t tag) {
     /* ..... */
           // If CSI of all UEs is ready, schedule ZF/prediction
           bool last_pilot_fft = pilot_fft_counters_.CompleteSymbol(frame_id);
           if (last_pilot_fft == true) {
             /* ..... */
             if (kEnableMac == true) {
               SendSnrReport(EventType::kSNRReport, frame_id, symbol_id);
             }
   
             // QMACS: launch scheduler
             config_->scheduler_->Launch(frame_id);
   
             ScheduleSubcarriers(EventType::kZF, frame_id, 0);
           }
         }
       }
   /* ..... */
   ```

Now Agora will initialize the `scheduler` sharing with `dozf` `doprecode`. To fully enable scheduling, we have to call functions to modify the buffer.

1. In `dozf`, modify precoder computation, because `DoZF::ComputePrecoder` takes limited input, the new one should be like

   ```c++
   // QMACS: takes sche_mat_csi as input parameter for dl
   float DoZF::ComputePrecoder(const arma::cx_fmat& mat_csi, const arma::cx_fmat& sche_mat_csi, complex_float* calib_ptr, complex_float* _mat_ul_zf, complex_float* _mat_dl_zf) {
     arma::cx_fmat mat_ul_zf(reinterpret_cast<arma::cx_float*>(_mat_ul_zf),
                             cfg_->UeAntNum(), cfg_->BsAntNum(), false);
     arma::cx_fmat mat_ul_zf_tmp;
     if (kUseInverseForZF != 0u) {
       try {
         mat_ul_zf_tmp = arma::inv_sympd(mat_csi.t() * mat_csi) * mat_csi.t();
       } catch (std::runtime_error&) {
         MLPD_WARN("Failed to invert channel matrix, falling back to pinv()\n");
         arma::pinv(mat_ul_zf_tmp, mat_csi, 1e-2, "dc");
       }
     } else {
       arma::pinv(mat_ul_zf_tmp, mat_csi, 1e-2, "dc");
     }
   
     if (cfg_->Frame().NumDLSyms() > 0) {
       arma::cx_fvec calib_vec(reinterpret_cast<arma::cx_float*>(calib_ptr),
                               cfg_->BfAntNum(), false);
       arma::cx_fmat mat_dl_zf_tmp;
       if (kUseUlZfForDownlink == true) { // QMACS: if use ul zf, no schedule
         // With orthonormal calib matrix:
         // pinv(calib * csi) = pinv(csi)*inv(calib)
         // This probably causes a performance hit since we are throwing
         // magnitude info away by taking the sign of the calibration matrix
         arma::cx_fmat calib_mat = arma::diagmat(arma::sign(calib_vec));
         mat_dl_zf_tmp = mat_ul_zf_tmp * inv(calib_mat);
       } else { 
          
         // QMACS: if use different zf, schedule for dl
         arma::cx_fmat mat_dl_csi = arma::diagmat(calib_vec) * sche_mat_csi; 
         // QMACS
           
         try {
           mat_dl_zf_tmp =
               arma::inv_sympd(mat_dl_csi.t() * mat_dl_csi) * mat_dl_csi.t();
         } catch (std::runtime_error&) {
           arma::pinv(mat_dl_zf_tmp, mat_dl_csi, 1e-2, "dc");
         }
       }
       // We should be scaling the beamforming matrix, so the IFFT
       // output can be scaled with OfdmCaNum() across all antennas.
       // See Argos paper (Mobicom 2012) Sec. 3.4 for details.
       float scale = 1 / (abs(mat_dl_zf_tmp).max());
       mat_dl_zf_tmp *= scale;
   
       for (size_t i = 0; i < cfg_->NumCells(); i++) {
         if (cfg_->ExternalRefNode(i) == true) {
           mat_dl_zf_tmp.insert_cols(
               cfg_->RefAnt(i),
               arma::cx_fmat(cfg_->UeAntNum(), cfg_->NumChannels(),
                             arma::fill::zeros));
         }
       }
         
       // QMACS: correct the shape 
       // arma::cx_fmat mat_dl_zf(reinterpret_cast<arma::cx_float*>(_mat_dl_zf),
                               // cfg_->BsAntNum(), cfg_->UeAntNum(), false);
       arma::cx_fmat mat_dl_zf(reinterpret_cast<arma::cx_float*>(_mat_dl_zf),
                               cfg_->BsAntNum(), cfg_->scheduler_->GetSelectNum(), false); // change this shape
       // QMACS
         
       mat_dl_zf = mat_dl_zf_tmp.st();
     }
     /* ..... */
   ```

   Then in `DoZF::ZfTimeOrthogonal` `DoZF::ZfFreqOrthogonal` , modify as 

   ```c++
   /* ..DoZF::ZfTimeOrthogonal... */
   /* ..... */
   	double start_tsc3 = GetTime::WorkerRdtsc();
       duration_stat_->task_duration_[2] += start_tsc3 - start_tsc2;
   
       // QMACS: compute scheduled CSI
       arma::cx_fmat sche_mat_csi;
       this->cfg_->scheduler_->ScheduleCSI(frame_slot, cur_sc_id, sche_mat_csi, mat_csi);
   
       auto rcond = ComputePrecoder(mat_csi, sche_mat_csi, calib_gather_buffer_, 
                   ul_zf_matrices_[frame_slot][cur_sc_id], dl_zf_matrices_[frame_slot][cur_sc_id]);
   
       // QMACS
   
   	if (kPrintZfStats) {
         phy_stats_->UpdateCsiCond(frame_id, cur_sc_id, rcond);
       }
   /* ..... */
   /* ..DoZF::ZfTimeOrthogonal... */
   /* ..... */
     double start_tsc3 = GetTime::WorkerRdtsc();
     duration_stat_->task_duration_[2] += start_tsc3 - start_tsc2;
   
     arma::cx_fmat mat_csi(reinterpret_cast<arma::cx_float*>(csi_gather_buffer_),
                           cfg_->BsAntNum(), cfg_->UeAntNum(), false);
   
     // QMACS: compute scheduled CSI
     arma::cx_fmat sche_mat_csi;
     this->cfg_->scheduler_->ScheduleCSI(frame_slot, cfg_->GetZfScId(base_sc_id), sche_mat_csi, mat_csi);
     ComputePrecoder(mat_csi, sche_mat_csi, calib_gather_buffer_,
                     ul_zf_matrices_[frame_slot][cfg_->GetZfScId(base_sc_id)],
                     dl_zf_matrices_[frame_slot][cfg_->GetZfScId(base_sc_id)]);
     // QMACS
   
     duration_stat_->task_duration_[3] += GetTime::WorkerRdtsc() - start_tsc3;
     duration_stat_->task_count_++;
     duration_stat_->task_duration_[0] += GetTime::WorkerRdtsc() - start_tsc1;
   /* ..... */
   ```

2. Then in `doprecode`, modify the definition of `mkl_jit_status_t` in precoder constructor as

   ```c++
   /* ..... */
   #if USE_MKL_JIT
     MKL_Complex8 alpha = {1, 0}; 
     MKL_Complex8 beta = {0, 0};
     // Input: A: BsAntNum() x UeAntNum() , B: UeAntNum() x 1
     // Output: C: BsAntNum() x 1
     // Leading dimensions: A: bs_ant_num(), B: ue_num(), C: bs_ant_num()
   
     // QMACS: dim = select_num, modify gemm
     size_t dim = (cfg_->scheduler_ == nullptr ? cfg_->UeNum() : cfg_->scheduler_->GetSelectNum());
     std::cout << "Dim: " << dim << "\n";
   
     mkl_jit_status_t status = mkl_jit_create_cgemm(
         &jitter_, MKL_COL_MAJOR, MKL_NOTRANS, MKL_NOTRANS, cfg_->BsAntNum(), 1,
         dim, &alpha, cfg_->BsAntNum(), dim, &beta,
         cfg_->BsAntNum());
   
     // mkl_jit_status_t status = mkl_jit_create_cgemm(
     //     &jitter_, MKL_COL_MAJOR, MKL_NOTRANS, MKL_NOTRANS, cfg_->BsAntNum(), 1,
     //     cfg_->UeAntNum(), &alpha, cfg_->BsAntNum(), cfg_->UeAntNum(), &beta,
     //     cfg_->BsAntNum());
     // QMACS
   
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
   EventData DoPrecode::Launch(size_t tag) {
     size_t start_tsc = GetTime::WorkerRdtsc();
     const size_t frame_id = gen_tag_t(tag).frame_id_;
     const size_t base_sc_id = gen_tag_t(tag).sc_id_;
     const size_t symbol_id = gen_tag_t(tag).symbol_id_;
     const size_t symbol_idx_dl = cfg_->Frame().GetDLSymbolIdx(symbol_id);
     const size_t total_data_symbol_idx =
         cfg_->GetTotalDataSymbolIdxDl(frame_id, symbol_idx_dl); // QMACS: unused
     const size_t frame_slot = frame_id % kFrameWnd;
   
     /* ..... */
     if (kUseSpatialLocality) {
       for (size_t i = 0; i < max_sc_ite; i = i + kSCsPerCacheline) {
         size_t start_tsc1 = GetTime::WorkerRdtsc();
         for (size_t user_id = 0; user_id < cfg_->UeAntNum(); user_id++) {
           // QMACS: separate queue of data buffer
           size_t total_symbol_idx =
                 cfg_->GetTotalDataSymbolIdxDl(cfg_->scheduler_->GetFrameID(user_id), 
                                             symbol_idx_dl); // use frame_id in queue
           for (size_t j = 0; j < kSCsPerCacheline; j++) {
             LoadInputData(symbol_idx_dl, total_symbol_idx, user_id,
                           base_sc_id + i + j, j);
           }
           // QMACS 
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
           // QMACS: separate queue of data buffer
           size_t total_symbol_idx =
                 cfg_->GetTotalDataSymbolIdxDl(cfg_->scheduler_->GetFrameID(user_id), 
                                             symbol_idx_dl); // use frame_id in queue
           LoadInputData(symbol_idx_dl, total_symbol_idx, user_id, cur_sc_id,
                         0);
           // QMACS
         }
         size_t start_tsc2 = GetTime::WorkerRdtsc();
         duration_stat_->task_duration_[1] += start_tsc2 - start_tsc1;
   
         PrecodingPerSc(frame_slot, cur_sc_id, i);
         duration_stat_->task_count_++;
         duration_stat_->task_duration_[2] += GetTime::WorkerRdtsc() - start_tsc2;
       }
     }
   
     /* ..... */
     return EventData(EventType::kPrecode, tag);
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
         (kUseSpatialLocality
              ? (sc_id_in_block % kSCsPerCacheline * cfg_->UeAntNum())
              : 0));
   
     // QMACS: schedule the data
     arma::cx_fmat mat_data(data_ptr, cfg_->UeNum(), 1, false);
     arma::cx_fmat dst_mat;
     cfg_->scheduler_->ScheduleDATA(frame_slot, cfg_->GetZfScId(sc_id), dst_mat, mat_data); // schedule data
     // std::cout << "***********data_select size: " << dst_mat.size() << "\n";
     auto* dst_ptr = dst_mat.begin();
     auto* precoded_ptr = reinterpret_cast<arma::cx_float*>(
         precoded_buffer_temp_ + sc_id_in_block * cfg_->BsAntNum());
   #if USE_MKL_JIT
     my_cgemm_(jitter_, (MKL_Complex8*)precoder_ptr, (MKL_Complex8*)dst_ptr,
               (MKL_Complex8*)precoded_ptr);
     // QMACS
   #else
   /* ..... */
   ```

## Commands to run the simulator

### Compile

To compile the program with MAC enabled

```shell
/Agora$ rm -rf build
/Agora$ cd build
/Agora/build$ cmake .. -DENABLE_MAC=true
/Agora/build$ make -j8
```

### Run

You need 5-6 terminals to run the following commands separately:

Run this command in advance:

```shell
./build/data_generator --conf_file data/mac-dl-sim.json
```

Run the following two commands at first:  

```shell
./build/chsim --bs_threads 1 --ue_threads 1 --worker_threads 2 --core_offset 1 --conf_file data/mac-dl-sim.json --chan_snr 120.0
```

```shell
./build/user --conf_file data/mac-dl-sim.json
```

Run the following three commands together (as quick as possible):

```shell
./build/macuser --conf_file data/mac-dl-sim.json --core_offset 19 --num_receiver_threads 8 --rx_file rx_ue.txt
```

```shell
./build/agora --conf_file data/mac-dl-sim.json --select_num 8
```

```shell
./build/macbs --conf_file data/mac-dl-sim.json --core_offset 16 --enable_slow_start 1 --data_file "" --tx_file tx_bs.txt
```

To reach higher bandwidth for test, here we use `data/mac-dl-sim.json` as configuration file, where

`bs_radio_num` : number of antennas of base station.

`ue_radio_num` : number of user equipment, i.e. UeNum.

`frame_schedule` : distribution of symbols in a frame. **Note: the number of Ps = UeNum**

`core_offset` : core offset of agora

`ue_core_offset` : core offset of user   

The `xx_threads` `core_offset` parameters can be adjust based on your hardware (physical thread of CPU), but make sure there is no core overlapping.

`RCVT` : number of receiver threads, normally equal to UeNum

`RXUE` : filename of DL (client) throughput, it will be stored in the folder `Agora/log`, no need to add path to `RXUE` (e.g. flag --result_file rx_ue.txt will store the log of throughput in `Agora/log/rx_ue.txt`)

`RXBS` : Same as the above

`CAP` : Channel capacity, number of **users** that base station will send data to in every time slot. It will be clip to UeNum if too large.

## Customize Scheduling algorithm

In `scheduler` class, there is a function called `void Scheduler::Launch(size_t frame_id)`, where developer can select the user to receive data in this frame slot. Because scheduling task is called in agora as a part of a event, the amount of **preprocessed frames** is depends on the setting of Agora.

**Note: Agora only buffered `kFrameWnd = 40` of frames configuration, so the `frame_id` should be translated by `frame_slot = frame_id%kFrameWnd;`.** 

**Note: Scheduler was designed to support scheduling in the unit of symbols or even subcarriers, but due to the beamforming setting of Agora, currently it only supports scheduling each frame. Therefore, when calling `selection_[frame_id][symbol_id]`, `symbol_id = 0` must be held.**  

```c++
void Scheduler::Launch(size_t frame_id) {
  // called after kFFT finish CSI of all UEs (kPilot)
  std::cout << "**************scheduler launch!**************\n frame id:" << frame_id << std::endl;
  frame_id %= kFrameWnd;
  unsigned int *selection = selection_[frame_id][0];
  // 
  // You can customize scheduling algorithm here by adjusting selection
  //
  PrintSelect(frame_id, 0);
}
```

The `selection` is an association matrix in a shape of `select_num` $\times$ `UeNum_`. 

To give an example, suppose `select_num=3` `UeNum=4`
$$
\text{selection} =\left(\begin{array}{lll}
0 & 0 & 0 & 1\\
1 & 0 & 0 & 0\\
0 & 1 & 0 & 0
\end{array}\right)
$$
This matrix means only `{user[3],user[0],user[1]}` will receive data in this frame.

**Note: as to definition of association matrix, each column and row should only have a single 1 while others be 0, but the order doesn't matter.**

To help with debugging, simply call `Scheduler::PrintSelect(size_t frame_id, size_t symbol_id)`  to see the selection matrix.

To support real application running with frame scheduling, scheduler separate the pointer of each users buffer, which means the data block of user that is not selected in this frame will be blocked by base station until its turn to receive data.




