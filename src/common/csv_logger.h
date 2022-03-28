/**
 * @file csv_logger.h
 * @brief Declaration file for the CsvLogger and MatLogger classes which
 * record runtime physical-layer performance data and zero-forcing matrices
 * into csv files. Enabled or disabled by cmake flags.
 */

#ifndef CSV_LOGGER_H_
#define CSV_LOGGER_H_

#if defined(ENABLE_CSV_LOG)
#include "armadillo"
#include "spdlog/spdlog.h"
#include "spdlog/async.h"
#include "spdlog/pattern_formatter.h"
#include "spdlog/sinks/basic_file_sink.h"

class CsvLogger {
public:
  enum LogID { kDLPSNR, kEVMSNR, kBERSER, kCSI, kDLZF };

  CsvLogger(int dev_id, enum LogID log_id);
  inline void Write(size_t u1, size_t u2, size_t u3, float f1) {
    logger_->info("{},{},{},{}", u1, u2, u3, f1);
  }
  inline void Write(size_t u1, size_t u2, size_t u3, float f1, float f2) {
    logger_->info("{},{},{},{},{}", u1, u2, u3, f1, f2);
  }
  inline void Write(size_t u1, size_t u2, size_t u3, size_t u4, float f1, float f2) {
    logger_->info("{},{},{},{},{},{}", u1, u2, u3, u4, f1, f2);
  }
  inline void Write(const std::string& str) {
    logger_->info(str);
  }

protected:
  inline static const std::string kCsvName[] = {
    "log-dlpsnr-ue",
    "log-evmsnr-ue",
    "log-berser-ue",
    "log-matcsi-bs",
    "log-matdlzf-bs"
  };
  inline static const std::string kCsvHeader[] = {
    "Frame,Symbol,UE-Ant,DL-Pilot-SNR",
    "Frame,Symbol,UE-Ant,EVM,EVM-SNR",
    "Frame,Symbol,UE-Ant,Bit-Error-Rate,Symbol-Error-Rate",
    "Frame,Subcarrier,BS-Ant,UE-Ant,CSI-Real,CSI-Imag",
    "Frame,Subcarrier,BS-Ant,UE-Ant,DLZF-Real,DLZF-Imag"
  };
  inline static std::mutex mtx;
  std::shared_ptr<spdlog::logger> logger_;
};
#else
class CsvLogger {
public:
  enum LogID { kDLPSNR, kEVMSNR, kBERSER, kCSI, kDLZF };
  CsvLogger(int, enum LogID);
  void Write(...);
};
#endif //ENABLE_CSV_LOG

#if defined(ENABLE_MAT_LOG)
class MatLogger : public CsvLogger {
public:
  MatLogger(int dev_id, enum LogID log_id);
  void UpdateMatBuf(size_t frame_id, size_t sc_id, const arma::cx_fmat& mat_in);
  void SaveMatBuf();
private:
  static constexpr LogID kMatLogID[] = {kCSI, kDLZF};
  static constexpr size_t kMatLogs = sizeof(kMatLogID) / sizeof(LogID);
  static constexpr size_t kMatLogFrames = 1000;
  static constexpr size_t kMatLogSCs = 304;
  static constexpr size_t kMatLogBSAnts = 8;
  static constexpr size_t kMatLogUEAnts = 1;
  static bool mat_log_active[kMatLogs];
  static size_t mat_last_frame[kMatLogs];
  static std::complex<float> mat_buffer[kMatLogs][kMatLogFrames][kMatLogSCs]
                                       [kMatLogBSAnts][kMatLogUEAnts];
  int mat_idx_;
};
#else
class MatLogger : public CsvLogger {
public:
  MatLogger(int, enum LogID);
  void UpdateMatBuf(...);
  void SaveMatBuf();
};
#endif //ENABLE_MAT_LOG

#endif //CSV_LOGGER_H_