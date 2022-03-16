#include "csv_logger.h"

#if defined(CSV_LOG_LEVEL)

std::shared_ptr<spdlog::logger> csv_logger[kCsvLogNUM] = {nullptr};

static const char* kCsvLogName[kCsvLogNUM] = {
  "log-dlpsnr-ue",
  "log-evmsnr-ue",
  "log-berser-ue",
  "log-matcsi-bs",
  "log-matdlzf-bs"
};

static const char* kCsvLogHeader[kCsvLogNUM] = {
  "Frame,Symbol,UE-Ant,DL-Pilot-SNR",
  "Frame,Symbol,UE-Ant,EVM,EVM-SNR",
  "Frame,Symbol,UE-Ant,Bit-Error-Rate,Symbol-Error-Rate",
  "Frame,Subcarrier,BS-Ant,UE-Ant,CSI-Real,CSI-Imag",
  "Frame,Subcarrier,BS-Ant,UE-Ant,DLZF-Real,DLZF-Imag"
};

void CsvLogInit(size_t dev_id, size_t log_id) {
  std::string filename = fmt::sprintf("%s-%zu.csv", kCsvLogName[log_id], dev_id);
  std::remove(filename.c_str());
  csv_logger[log_id] = spdlog::create_async_nb<spdlog::sinks::basic_file_sink_mt>
      (fmt::sprintf("csv_logger_%zu", log_id), filename);
  csv_logger[log_id]->set_level(spdlog::level::CSV_LOG_LEVEL);
  csv_logger[log_id]->set_pattern("%v");
  csv_logger[log_id]->CSV_LOG_LEVEL(kCsvLogHeader[log_id]);
}

#endif //CSV_LOG_LEVEL