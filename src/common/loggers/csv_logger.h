/**
 * @file csv_logger.h
 * @brief Declaration file for the CsvLogger class which records runtime
 * physical-layer performance into csv files. Enabled or disabled by cmake.
 */

#ifndef CSV_LOGGER_H_
#define CSV_LOGGER_H_

#include <array>
#include <string>

#include "config.h"
#include "symbols.h"

#if defined(ENABLE_CSV_LOG)
#include "spdlog/spdlog.h"
#endif

namespace CsvLog {

enum CsvLogId {
  kSNR,
  kRSSI,
  kNOISE,
  kEVM,
  kEVMSC,
  kEVMSNR,
  kBER,
  kSER,
  kCSI,
  kCalib,
  kCsvLogs
};

enum MatLogId { kULCSI, kDLCSI, kDlBeam, kMatLogs };

static constexpr size_t kAllLogs = kCsvLogs + kMatLogs;

#if defined(ENABLE_CSV_LOG)
static const std::array<std::string, kAllLogs> kCsvName = {
    "snr", "rssi", "noise", "evm",   "evmsc", "evmsnr", "ber",
    "ser", "csi",  "calib", "ulcsi", "dlcsi", "dlbeam"};
#endif

class CsvLogger {
 public:
  CsvLogger([[maybe_unused]] size_t log_id, [[maybe_unused]] Config* const cfg,
            [[maybe_unused]] Direction dir,
            [[maybe_unused]] bool bs_only = false);

#if defined(ENABLE_CSV_LOG)
  inline void Write(const std::string& str) {
    if (logger_) {
      logger_->info(str);
    }
  }

 protected:
  std::shared_ptr<spdlog::logger> logger_;
#else
  inline void Write(const std::string& /*unused*/) {}
#endif
};  // CsvLogger

}  //namespace CsvLog
#endif  //CSV_LOGGER_H_
