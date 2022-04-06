/**
 * @file scheduler.cc
 * @brief Implementation file for resource scheduler
 */
#include "scheduler.h"

#include <algorithm>
#include <gflags/gflags.h>

DEFINE_uint64(select_num, 100,
              "Number of selected ue every symbol");

// NOTE: frame_id [0, inf), frame_slot [0, kFrameWnd)

Scheduler::Scheduler(size_t UeNum, size_t NumDLSyms, size_t UeAntNum, PtrGrid<kFrameWnd, kMaxUEs, complex_float>& csi_buffers) 
    : csi_buffers_(csi_buffers), UeNum_(UeNum), NumDLSyms_(NumDLSyms), UeAntNum_(UeAntNum) {
  // init with Agora
  select_num = std::min(FLAGS_select_num, UeNum_);
  this->selection_.Alloc(kFrameWnd, NumDLSyms_, select_num*UeNum_);
  last_ue = 0;
  AllocBuffer1d(&data_queue_, UeAntNum_, Agora_memory::Alignment_t::kAlign32, 1);
  AllocBuffer1d(&capacity_, kFrameWnd,  Agora_memory::Alignment_t::kAlign64, 1);
  std::cout << "**************scheduler init!**************\n" << "Select UE num: " << select_num << std::endl;
  std::cout << "DLSyms: " << NumDLSyms_ << std::endl;
}

Scheduler::~Scheduler() {
  std::cout << "**************scheduler destroyed!**************" << std::endl;
  FreeBuffer1d(&data_queue_);
}

void Scheduler::Launch(size_t frame_id) {
  // called after kFFT finish CSI of all UEs (kPilot)
  std::cout << "**************scheduler launch!**************\n frame id:" << frame_id << std::endl;
  frame_id %= kFrameWnd;
#if USE_MKL_JIT
  size_t cap = this->select_num;
#else
  size_t cap = this->select_num - frame_id%3; // just test select_num : select_num-1
  this->capacity_[frame_id] = cap;
#endif
  // currently use round robin
  // for (size_t symbol_id = 0; symbol_id < NumDLSyms_; ++symbol_id) {
  for (size_t symbol_id = 0; !symbol_id; ++symbol_id) {
    unsigned int *selection = selection_[frame_id][symbol_id];
    memset(selection, 0, sizeof(unsigned int)*select_num*UeNum_);
    for (size_t i = 0; i < cap; ++i) {
      // selection[last_ue*select_num + i] = 1; // last_ue is col, i is row
      // selection[last_ue*select_num + i] = 1;
      selection[i*UeNum_ + last_ue] = 1;
      last_ue = (last_ue + 1)%UeNum_;
    }
  }
  // update.store(true);
  PrintSelect(frame_id, 0);
}

void Scheduler::PrintQueue() {
    std::cout << "Frame queues of Users: \n";
    for (size_t ue_id = 0; ue_id < UeNum_; ++ue_id) {
        std::cout << "User: " << ue_id << " next_frame: " << data_queue_[ue_id] << "\n";
    }
}

void Scheduler::PrintSelect(size_t frame_slot, size_t symbol_id) {
  unsigned int *selection = selection_[frame_slot][symbol_id];
  arma::Mat<arma::u32> selection_mat(reinterpret_cast<arma::u32*>(selection), UeNum_, this->capacity_[frame_slot], false);
  // std::cout << "Full Matrix--- \n" << selection_mat << std::endl;
  std::cout << "Selection Matrix--- \n" << selection_mat.t() << std::endl;
}

void Scheduler::ScheduleCSI(const size_t frame_slot, const size_t sc_id, arma::cx_fmat& dst_csi_mat, const arma::cx_fmat& src_csi_mat) {
  unsigned int *selection = selection_[frame_slot][0];
  arma::Mat<arma::u32> selection_mat(reinterpret_cast<arma::u32*>(selection), UeNum_, this->capacity_[frame_slot], false);
  dst_csi_mat = src_csi_mat * selection_mat; 
  // if (update.exchange(false)) {
  //   std::cout << "*************Schedule Frame_slot: " << frame_slot << " sc_id: " << sc_id << "\n Sched CSI Matrix--- \n" << dst_csi_mat << std::endl;
  // }
}

void Scheduler::ScheduleDATA(const size_t frame_slot, const size_t sc_id, arma::cx_fmat& dst_data_mat, arma::cx_fmat& src_data_mat) {
  unsigned int *selection = selection_[frame_slot][0];
  arma::Mat<arma::u32> selection_mat(reinterpret_cast<arma::u32*>(selection), UeNum_, this->capacity_[frame_slot], false);
  dst_data_mat = (selection_mat.t() * src_data_mat); 
  if (update.exchange(false)) {
    std::cout << "*************Schedule Frame_slot: " << frame_slot << " sc_id: " << sc_id 
          // << "\n Sched data Matrix--- \n" << dst_data_mat << std::endl;
          << std::endl;
  }
}

void Scheduler::UpdateQueue(size_t frame_id) {
  frame_id %= kFrameWnd;
  unsigned int *selection = selection_[frame_id][0];
  arma::Mat<arma::u32> selection_mat(reinterpret_cast<arma::u32*>(selection), UeNum_, select_num, false);
  auto compression = arma::sum(selection_mat, 1);
  auto dqueue = arma::Col<arma::u32>(reinterpret_cast<arma::u32*>(data_queue_), UeNum_, false);
  dqueue += compression; // enable separate queue
  // dqueue += 1; // disable separate queue
  PrintQueue();
}

