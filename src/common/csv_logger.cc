/**
 * @file csv_logger.h
 * @brief Implementation file for the CsvLogger and MatLogger classes which
 * record runtime physical-layer performance data and zero-forcing matrices
 * into csv files. Enabled or disabled by cmake flags.
 */

#include "csv_logger.h"

#if defined(ENABLE_CSV_LOG)

CsvLogger::CsvLogger(int dev_id, LogID log_id) {
  mtx.lock();
  logger_ = spdlog::get(kCsvName[log_id]);
  if (logger_ != nullptr) {
    mtx.unlock();
    return;
  }
  std::string filename = kCsvName[log_id] + "-" + std::to_string(dev_id)
                                          + ".csv";
  std::remove(filename.c_str());
  logger_ = spdlog::create_async_nb<spdlog::sinks::basic_file_sink_mt>
            (kCsvName[log_id], filename);
  mtx.unlock();
  logger_->set_level(spdlog::level::info);
  logger_->set_pattern("%v");
  logger_->info(kCsvHeader[log_id]);
}

#if defined(ENABLE_MAT_LOG)

bool MatLogger::mat_log_active[kMatLogs];
size_t MatLogger::mat_last_frame[kMatLogs];
std::complex<float> MatLogger::mat_buffer[kMatLogs][kMatLogFrames][kMatLogSCs]
                                         [kMatLogBSAnts][kMatLogUEAnts];

MatLogger::MatLogger(int dev_id, LogID log_id)
         : CsvLogger(dev_id, log_id), mat_idx_(-1) {
  for (size_t i = 0; i < kMatLogs; i++) {
    if (log_id == kMatLogID[i]) {
      mat_idx_ = i;
      mat_log_active[i] = true;
      break;
    }
  }
}

void MatLogger::UpdateMatBuf(size_t frame_id, size_t sc_id,
                             const arma::cx_fmat& mat_in) {
  if (mat_idx_ == -1 || frame_id >= kMatLogFrames || sc_id >= kMatLogSCs) {
    return;
  }
  const size_t bs_ants = mat_in.n_rows < kMatLogBSAnts ?
                         mat_in.n_rows : kMatLogBSAnts;
  const size_t ue_ants = mat_in.n_cols < kMatLogUEAnts ?
                         mat_in.n_cols : kMatLogUEAnts;
  for (size_t i = 0; i < bs_ants; i++) {
    for (size_t j = 0; j < ue_ants; j++) {
      mat_buffer[mat_idx_][frame_id][sc_id][i][j] = mat_in(i, j);
    }
  }
  if (frame_id > mat_last_frame[mat_idx_]) {
    mat_last_frame[mat_idx_] = frame_id;
  }
}

void MatLogger::SaveMatBuf() {
  mtx.lock();
  if (mat_idx_ == -1 || mat_log_active[mat_idx_] == false) {
    mtx.unlock();
    return;
  }
  mat_log_active[mat_idx_] = false;
  mtx.unlock();
  for (size_t frame_id = 0; frame_id <= mat_last_frame[mat_idx_]; frame_id++) {
    for (size_t sc_id = 0; sc_id < kMatLogSCs; sc_id++) {
      for (size_t i = 0; i < kMatLogBSAnts; i++) {
        for (size_t j = 0; j < kMatLogUEAnts; j++) {
          const std::complex<float>& cx = mat_buffer[mat_idx_][frame_id]
                                                    [sc_id][i][j];
          Write(frame_id, sc_id, i, j, cx.real(), cx.imag());
        }
      }
    }
  }
}

#endif //ENABLE_MAT_LOG

#endif //ENABLE_CSV_LOG