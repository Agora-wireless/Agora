/**
 * @file csv_logger.h
 * @brief Declaration file for the CsvLogger class which records runtime
 * physical-layer performance into csv files. Enabled or disabled by cmake.
 */

#ifndef CSV_LOGGER_H_
#define CSV_LOGGER_H_

#include <array>
#include <string>

#if defined(ENABLE_CSV_LOG)
#include "spdlog/spdlog.h"
#endif

namespace CsvLog {

enum CsvLogId {
  kSNR,
  kRSSI,
  kNOISE,
  kEVM,
  kEVMSNR,
  kBER,
  kSER,
  kCSI,
  kCsvLogs
};

enum MatLogId { kDLCSI, kDlBeam, kMatLogs };

constexpr size_t kAllLogs = kCsvLogs + kMatLogs;

#if defined(ENABLE_CSV_LOG)
const std::array<std::string, kAllLogs> kCsvName = {
    "snr", "rssi", "noise", "evm",   "evmsnr",
    "ber", "ser",  "csi",   "dlcsi", "dlzf"};
const std::array<std::string, kMatLogs> kMatHeader = {
    "Frame,Subcarrier,BS-Ant,UE-Ant,Real,Imag",
    "Frame,Subcarrier,BS-Ant,UE-Ant,Real,Imag"};
#endif

class CsvLogger {
 public:
  CsvLogger(size_t log_id, const std::string& radio_name);

#if defined(ENABLE_CSV_LOG)
  inline void Write(size_t u1, size_t u2, size_t u3, size_t u4, float f1,
                    float f2) {
    logger_->info("{},{},{},{},{},{}", u1, u2, u3, u4, f1, f2);
  }
  inline void Write(const std::string& str) { logger_->info(str); }

 protected:
  std::shared_ptr<spdlog::logger> logger_;
#else
  inline void Write(size_t /*unused*/, size_t /*unused*/, size_t /*unused*/,
                    size_t /*unused*/, float /*unused*/, float /*unused*/) {}
  inline void Write(const std::string& /*unused*/) {}
#endif
};  // CsvLogger

}  //namespace CsvLog
#endif  //CSV_LOGGER_H_
