/**
 * @file csv_logger.cc
 * @brief Implementation file for the CsvLogger class which records runtime
 * physical-layer performance into csv files. Enabled or disabled by cmake.
 */

#include "csv_logger.h"

#include "logger.h"
#include "utils.h"

#if defined(ENABLE_CSV_LOG)
#include "spdlog/async.h"
#include "spdlog/pattern_formatter.h"
#include "spdlog/sinks/basic_file_sink.h"
#endif

namespace CsvLog {

CsvLogger::CsvLogger([[maybe_unused]] size_t log_id,
                     [[maybe_unused]] Config* const cfg,
                     [[maybe_unused]] Direction dir,
                     [[maybe_unused]] bool bs_only) {
#if defined(ENABLE_CSV_LOG)
  if (log_id >= kAllLogs) {
    AGORA_LOG_ERROR("Invalid log id %zu in CsvLogger\n", log_id);
  } else {
    std::string radio_name;
    bool data_avail;
    if (dir == Direction::kUplink) {
      radio_name = "BS";
      data_avail = (bs_only == true || cfg->Frame().NumULSyms() > 0);
    } else {
      radio_name = cfg->UeRadioName().at(0);
      data_avail = (bs_only == false && cfg->Frame().NumDLSyms() > 0);
    }
    if (data_avail) {
      const std::string filename = "files/log/" + cfg->Timestamp() + "/log-" +
                                   kCsvName.at(log_id) + "-" + radio_name +
                                   ".csv";
      logger_ = spdlog::create_async_nb<spdlog::sinks::basic_file_sink_mt>(
          kCsvName.at(log_id), filename);
      logger_->set_level(spdlog::level::info);
      logger_->set_pattern("%v");
    }
  }
#endif  //ENABLE_CSV_LOG
}

}  //namespace CsvLog