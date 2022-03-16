/**
 * @file scheduler.h
 * @brief Declaration file for resource scheduler
 */

#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include <armadillo>
#include <iostream>
#include <vector>

#include "buffer.h"
#include "concurrentqueue.h"
// #include "config.h"
#include "gettime.h"
#include "memory_manage.h"
#include "symbols.h"

// #define SELECT_NUM 2
class Scheduler {
  public:
    Scheduler(size_t UeNum, size_t NumDLSyms, size_t UeAntNum, PtrGrid<kFrameWnd, kMaxUEs, complex_float>& csi_buffers);
    ~Scheduler();
    
    void Launch(size_t frame_id); // scheduling based on csi_buffer
    void PrintSelect(size_t frame_id, size_t symbol_id); // print the selection mat
    void PrintQueue(); // print frame queues of each user
    size_t GetSelectNum() {return this->select_num;} // return select_num
    size_t GetFrameID(size_t ue_id) {return (size_t) this->data_queue_[ue_id];} // return frame_id in queue

    void ScheduleCSI(const size_t frame_id, const size_t sc_id, arma::cx_fmat& dst_csi_mat, arma::cx_fmat& src_csi_mat); // schedule csi_buffer
    void ScheduleDATA(const size_t frame_id, const size_t sc_id, arma::cx_fmat& dst_data_mat, arma::cx_fmat& src_data_mat); // schedule data_mat
    void UpdateQueue(size_t frame_id); // update the data queue according to schedule

  private:
    // Config* const cfg_;
    PtrGrid<kFrameWnd, kMaxUEs, complex_float>& csi_buffers_;
    size_t UeNum_;
    size_t NumDLSyms_;
    size_t UeAntNum_;

    PtrGrid<kFrameWnd, kMaxSymbols, unsigned int> selection_;
    // e.g. 
    // 3 ue in total and select_num = 2
    // size: (2, 3)
    // when select ue0, ue2
    // 1 0 0
    // 0 0 1
    unsigned int *data_queue_; // in frame as unit

    size_t select_num;
    size_t last_ue;
    // bool update;
    std::atomic<bool> update;
};



#endif // SCHEDULER_H_
