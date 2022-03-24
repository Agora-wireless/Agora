#include "csv_logger.h"

#if defined(ENABLE_CSV_LOG)

static const char* const kCsvName[] = {
  "log-dlpsnr-ue",
  "log-evmsnr-ue",
  "log-berser-ue",
  "log-matcsi-bs",
  "log-matdlzf-bs"
};

static const char* const kCsvHeader[] = {
  "Frame,Symbol,UE-Ant,DL-Pilot-SNR",
  "Frame,Symbol,UE-Ant,EVM,EVM-SNR",
  "Frame,Symbol,UE-Ant,Bit-Error-Rate,Symbol-Error-Rate",
  "Frame,Subcarrier,BS-Ant,UE-Ant,CSI-Real,CSI-Imag",
  "Frame,Subcarrier,BS-Ant,UE-Ant,DLZF-Real,DLZF-Imag"
};

CsvLogger::CsvLogger(size_t dev_id, enum CsvLogID log_id) {
  std::string filename = fmt::sprintf("%s-%zu.csv", kCsvName[log_id], dev_id);
  std::remove(filename.c_str());
  logger_ = spdlog::create_async_nb<spdlog::sinks::basic_file_sink_mt>
      (fmt::sprintf("csv_logger_%zu", log_id), filename);
  logger_->set_level(spdlog::level::info);
  logger_->set_pattern("%v");
  logger_->info(kCsvHeader[log_id]);
}

#if defined(ENABLE_MAT_LOG)

static constexpr enum CsvLogID kMatLogID[] = {kMatLogCSI, kMatLogDLZF};
static constexpr size_t kMatLogs = sizeof(kMatLogID) / sizeof(enum CsvLogID),
                        kMatLogFrames = 1000, kMatLogSCs = 304,
                        kMatLogBSAnts = 8, kMatLogUEAnts = 1;
static std::complex<float> mat_buffer[kMatLogs][kMatLogFrames][kMatLogSCs]
                                     [kMatLogBSAnts][kMatLogUEAnts];

MatLogger::MatLogger(size_t dev_id, enum CsvLogID log_id)
         : CsvLogger(dev_id, log_id) {
  size_t i;
  for (i = 0; i < kMatLogs; i++) {
    if (log_id == kMatLogID[i]) {
      this->mat_idx_ = i;
      break;
    }
  }
  this->is_active_ = (i < kMatLogs);
}

void MatLogger::UpdateMatBuf(size_t frame_id, size_t sc_id,
                             const arma::cx_fmat& mat_in) {
  if (this->is_active_ == false || frame_id >= kMatLogFrames
                                || sc_id >= kMatLogSCs) {
    return;
  }
  const size_t bs_ants = mat_in.n_rows < kMatLogBSAnts ?
                         mat_in.n_rows : kMatLogBSAnts;
  const size_t ue_ants = mat_in.n_cols < kMatLogUEAnts ?
                         mat_in.n_cols : kMatLogUEAnts;
  for (size_t i = 0; i < bs_ants; i++) {
    for (size_t j = 0; j < ue_ants; j++) {
      mat_buffer[this->mat_idx_][frame_id][sc_id][i][j] = mat_in(i, j);
    }
  }
}

void MatLogger::SaveMatBuf() {
  if (this->is_active_ == false) {
    return;
  }
  this->is_active_ = false;
  for (size_t frame_id = 0; frame_id < kMatLogFrames; frame_id++) {
    for (size_t sc_id = 0; sc_id < kMatLogSCs; sc_id++) {
      for (size_t i = 0; i < kMatLogBSAnts; i++) {
        for (size_t j = 0; j < kMatLogUEAnts; j++) {
          const std::complex<float>& cx = mat_buffer[this->mat_idx_][frame_id]
                                                    [sc_id][i][j];
          this->Write("%zu,%zu,%zu,%zu,%f,%f", frame_id, sc_id, i, j,
                      cx.real(), cx.imag());
        }
      }
    }
  }
}

#endif //ENABLE_MAT_LOG

#endif //ENABLE_CSV_LOG