/**
 * @file csv_logger.h
 * @brief Declaration file for the CsvLogger class which records runtime
 * physical-layer performance into csv files. Enabled or disabled by cmake.
 */

#ifndef CSV_LOGGER_H_
#define CSV_LOGGER_H_

#include <array>
#include <memory>
#include "symbols.h"

#if defined(ENABLE_CSV_LOG)
#include "spdlog/async.h"
#include "spdlog/pattern_formatter.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/spdlog.h"
#endif

namespace CsvLog {

constexpr size_t kCsvIdStart = 0;
constexpr size_t kSNR = 0;
constexpr size_t kEVM = 1;
constexpr size_t kEVMSNR = 2;
constexpr size_t kBER = 3;
constexpr size_t kSER = 4;
constexpr size_t kCsvLogs = 5;
constexpr size_t kMatIdStart = kCsvLogs;
constexpr size_t kDLCSI = 0;
constexpr size_t kDLZF = 1;
constexpr size_t kMatLogs = 2;
constexpr size_t kAllLogs = kCsvLogs + kMatLogs;

#if defined(ENABLE_CSV_LOG)
const std::array<std::string, kAllLogs> kCsvName = {
    "snr", "evm", "evmsnr", "ber", "ser", "dlcsi", "dlzf"};
const std::array<std::string, kAllLogs> kMatHeader = {
    "Frame,Subcarrier,BS-Ant,UE-Ant,Real,Imag",
    "Frame,Subcarrier,BS-Ant,UE-Ant,Real,Imag"};
#endif

class CsvLogger {
 public:
  CsvLogger(size_t log_id, const std::string& radio_id, Direction dir);

#if defined(ENABLE_CSV_LOG)
  inline void Write(size_t u1, size_t u2, size_t u3, size_t u4, float f1,
                    float f2) {
    logger_->info("{},{},{},{},{},{}", u1, u2, u3, u4, f1, f2);
  }
  inline void Write(const std::string& str) { logger_->info(str); }

 protected:
  std::shared_ptr<spdlog::logger> logger_;
#else
  inline void Write(size_t, size_t, size_t, size_t, float, float) {}
  inline void Write(const std::string&) {}
#endif
};  // CsvLogger

}  //namespace CsvLog
#endif  //CSV_LOGGER_H_