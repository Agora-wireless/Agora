/**
 * @file csv_logger.cc
 * @brief Implementation file for the CsvLogger class which records runtime
 * physical-layer performance into csv files. Enabled or disabled by cmake.
 */

#include "csv_logger.h"

#include "logger.h"
#include "utils.h"

namespace CsvLog {

CsvLogger::CsvLogger(size_t log_id, const std::string& radio_id,
                     Direction dir) {
#if defined(ENABLE_CSV_LOG)

  if (log_id >= kAllLogs) {
    AGORA_LOG_ERROR("Invalid log id %zu in CsvLogger\n", log_id);
  } else {
    constexpr size_t kShortIdLen = 3;
    const std::string short_id =
        radio_id.empty() ? (dir == Direction::kUplink ? "BS" : "UE")
            : (radio_id.length() > kShortIdLen
                ? radio_id.substr(radio_id.length() - kShortIdLen)
                : radio_id);
    std::string filename = "log-" + kCsvName.at(log_id)
                           + (dir == Direction::kUplink ? "-ul-" : "-dl-")
                           + short_id + ".csv";
    std::remove(filename.c_str());
    logger_ = spdlog::create_async_nb<spdlog::sinks::basic_file_sink_mt>(
        kCsvName.at(log_id), filename);
    logger_->set_level(spdlog::level::info);
    logger_->set_pattern("%v");
  }
#else
  unused(radio_id);
  unused(log_id);
  unused(dir);
#endif  //ENABLE_CSV_LOG
}
}  //namespace CsvLog