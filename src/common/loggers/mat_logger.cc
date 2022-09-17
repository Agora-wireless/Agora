/**
 * @file mat_logger.cc
 * @brief Implementation file for the MatLogger class which records runtime
 * zero-forcing matrices into csv files. Enabled or disabled by cmake.
 */

#include "mat_logger.h"

#include "logger.h"
#include "utils.h"

namespace CsvLog {

static const std::string kMatHeader = "Frame,SC,BS-Ant,UE-Ant,Real,Imag";

MatLogger::MatLogger(size_t mat_log_id, Config* const cfg, Direction dir)
    : CsvLogger(kCsvLogs + mat_log_id, cfg, dir, true) {}

MatLogger::~MatLogger() {
  Write(kMatHeader);
  SaveMatBuf();
}

bool MatLogger::UpdateMatBuf([[maybe_unused]] const size_t frame_id,
                             [[maybe_unused]] const size_t sc_id,
                             [[maybe_unused]] const arma::cx_fmat& mat_in) {
  bool status = false;
#if defined(ENABLE_MAT_LOG)
  if (frame_id >= kFrameStart && frame_id < kFrameStart + kFrames &&
      sc_id < kSCs) {
    const auto mat_copy_size =
        arma::size(mat_in.n_rows < kBSAnts ? mat_in.n_rows : kBSAnts,
                   mat_in.n_cols < kUEAnts ? mat_in.n_cols : kUEAnts);
    mat_buffer_.at(frame_id - kFrameStart).at(sc_id)(0, 0, mat_copy_size) =
        mat_in(0, 0, mat_copy_size);
    status = true;
  }
#endif
  return status;
}

void MatLogger::SaveMatBuf() {
#if defined(ENABLE_MAT_LOG)
  if (logger_) {
    AGORA_LOG_INFO("MatLogger: saving %s\n", logger_->name());
    for (size_t frame_id = 0; frame_id < kFrames; frame_id++) {
      for (size_t sc_id = 0; sc_id < kSCs; sc_id++) {
        for (size_t i = 0; i < kBSAnts; i++) {
          for (size_t j = 0; j < kUEAnts; j++) {
            const arma::cx_float& cx = mat_buffer_.at(frame_id).at(sc_id)(i, j);
            logger_->info("{},{},{},{},{},{}", frame_id + kFrameStart, sc_id, i,
                          j, cx.real(), cx.imag());
          }  // end kUEAnts
        }    // end kBSAnts
      }      // end kSCs
    }        // end kFrames
  }
#endif
}

}  //namespace CsvLog