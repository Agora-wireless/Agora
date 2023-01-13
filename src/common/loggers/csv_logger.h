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
  kPltSnr,
  kPltRssi,
  kPltNoise,
  kBfSnr,
  kBfRssi,
  kBfNoise,
  kEvm,
  kEvmSc,
  kEvmSnr,
  kBer,
  kSer,
  kCsi,
  kCsvLogs
};

enum MatLogId { kCalib, kUlCsi, kDlCsi, kUlBeam, kDlBeam, kMatLogs };

static constexpr size_t kAllLogs = kCsvLogs + kMatLogs;

#if defined(ENABLE_CSV_LOG)
static const std::array<std::string, kAllLogs> kCsvName = {
    "plt-snr", "plt-rssi", "plt-noise", "bf-snr",  "bf-rssi", "bf-noise",
    "evm",     "evm-sc",   "evm-snr",   "ber",     "ser",     "csi",
    "calib",   "ul-csi",   "dl-csi",    "ul-beam", "dl-beam"};
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
