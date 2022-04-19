/**
 * @file csv_logger.h
 * @brief Declaration file for the CsvLogger class which records runtime
 * physical-layer performance into csv files. Enabled or disabled by cmake.
 */

#ifndef CSV_LOGGER_H_
#define CSV_LOGGER_H_

#include <array>
#include <memory>

#if defined(ENABLE_CSV_LOG)
#include "spdlog/async.h"
#include "spdlog/pattern_formatter.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/spdlog.h"
#endif

namespace CsvLog {

constexpr size_t kDLPSNR = 0;
constexpr size_t kEVMSNR = 1;
constexpr size_t kBERSER = 2;
constexpr size_t kCsvLogs = 3;
constexpr size_t kMatIdStart = kCsvLogs;
constexpr size_t kMatCSI = 0;
constexpr size_t kMatDLZF = 1;
constexpr size_t kMatLogs = 2;
constexpr size_t kAllLogs = kCsvLogs + kMatLogs;

#if defined(ENABLE_CSV_LOG)
const std::array<std::string, kAllLogs> kCsvName = {
    "log-dlpsnr-ue", "log-evmsnr-ue", "log-berser-ue", "log-matcsi-bs",
    "log-matdlzf-bs"};
const std::array<std::string, kAllLogs> kCsvHeader = {
    "Frame,UE-Ant,DL-Pilot-SNR-0,DL-Pilot-SNR-1",
    "Frame,Symbol,UE-Ant,EVM,EVM-SNR",
    "Frame,Symbol,UE-Ant,Bit-Error-Rate,Symbol-Error-Rate",
    "Frame,Subcarrier,BS-Ant,UE-Ant,CSI-Real,CSI-Imag",
    "Frame,Subcarrier,BS-Ant,UE-Ant,DLZF-Real,DLZF-Imag"};
#endif

class CsvLogger {
 public:
  CsvLogger(const std::string& radio_id, const size_t log_id);

#if defined(ENABLE_CSV_LOG)
  inline void Write(size_t u1, size_t u2, size_t u3, float f1, float f2) {
    logger_->info("{},{},{},{},{}", u1, u2, u3, f1, f2);
  }

  inline void Write(size_t u1, size_t u2, size_t u3, size_t u4, float f1,
                    float f2) {
    logger_->info("{},{},{},{},{},{}", u1, u2, u3, u4, f1, f2);
  }

  inline void Write(const std::string& str) { logger_->info(str); }

 protected:
  std::shared_ptr<spdlog::logger> logger_;
#else
  inline void Write(size_t, size_t, size_t, float, float) {}
  inline void Write(size_t, size_t, size_t, size_t, float, float) {}
  inline void Write(const std::string&) {}
#endif
};  // CsvLogger

}  //namespace CsvLog
#endif  //CSV_LOGGER_H_