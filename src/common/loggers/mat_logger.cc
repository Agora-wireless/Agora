/**
 * @file mat_logger.cc
 * @brief Implementation file for the MatLogger class which records runtime
 * zero-forcing matrices into csv files. Enabled or disabled by cmake.
 */

#include "mat_logger.h"

#include "logger.h"
#include "utils.h"

namespace CsvLog {

MatLogger::MatLogger(const std::string& radio_id, size_t mat_log_id)
    : CsvLogger(radio_id, mat_log_id + kMatIdStart) {}

MatLogger::~MatLogger() { SaveMatBuf(); }

bool MatLogger::UpdateMatBuf(const size_t frame_id, const size_t sc_id,
                             const arma::cx_fmat& mat_in) {
  bool status = false;
#if defined(ENABLE_MAT_LOG)
  if (frame_id < kFrames && sc_id < kSCs) {
    const size_t bs_ants = mat_in.n_rows < kBSAnts ? mat_in.n_rows : kBSAnts;
    const size_t ue_ants = mat_in.n_cols < kUEAnts ? mat_in.n_cols : kUEAnts;
    for (size_t i = 0; i < bs_ants; i++) {
      for (size_t j = 0; j < ue_ants; j++) {
        mat_buffer_.at(frame_id).at(sc_id).at(i).at(j) = mat_in(i, j);
      }
    }
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
          const arma::cx_float& cx =
              mat_buffer_.at(frame_id).at(sc_id).at(i).at(j);
          Write(frame_id, sc_id, i, j, cx.real(), cx.imag());
        }  // end kUEAnts
      }    // end kBSAnts
    }      // end kSCs
  }        // end kFrames
#endif
}

}  //namespace CsvLog