/**
 * @file mat_logger.cc
 * @brief Implementation file for the MatLogger class which records runtime
 * zero-forcing matrices into csv files. Enabled or disabled by cmake.
 */

#include "mat_logger.h"

#include "logger.h"
#include "utils.h"

namespace CsvLog {

MatLogger::MatLogger(size_t mat_log_id, const std::string& radio_name)
    : CsvLogger(kCsvLogs + mat_log_id, radio_name) {
#if defined(ENABLE_MAT_LOG)
  logger_->info(kMatHeader.at(mat_log_id));
#endif
}

MatLogger::~MatLogger() { SaveMatBuf(); }

bool MatLogger::UpdateMatBuf(const size_t frame_id, const size_t sc_id,
                             const arma::cx_fmat& mat_in) {
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
#else
  unused(frame_id);
  unused(sc_id);
  unused(mat_in);
#endif
  return status;
}

void MatLogger::SaveMatBuf() {
#if defined(ENABLE_MAT_LOG)
  AGORA_LOG_INFO("MatLogger: saving %s\n", logger_->name());
  for (size_t frame_id = 0; frame_id < kFrames; frame_id++) {
    for (size_t sc_id = 0; sc_id < kSCs; sc_id++) {
      for (size_t i = 0; i < kBSAnts; i++) {
        for (size_t j = 0; j < kUEAnts; j++) {
          const arma::cx_float& cx = mat_buffer_.at(frame_id).at(sc_id)(i, j);
          Write(frame_id + kFrameStart, sc_id, i, j, cx.real(), cx.imag());
        }  // end kUEAnts
      }    // end kBSAnts
    }      // end kSCs
  }        // end kFrames
#endif
}

}  //namespace CsvLog