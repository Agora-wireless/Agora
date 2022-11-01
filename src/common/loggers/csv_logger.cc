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
#include "spdlog/sinks/udp_sink.h"
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
      radio_name = "UE";
      data_avail = (bs_only == false && cfg->Frame().NumDLSyms() > 0);
    }
    if (data_avail) {
      const std::string filename = "files/log/" + cfg->Timestamp() + "/log-" +
                                   kCsvName.at(log_id) + "-" + radio_name +
                                   ".csv";
      auto file_sink =
          std::make_shared<spdlog::sinks::basic_file_sink_mt>(filename, true);
      file_sink->set_level(spdlog::level::info);
      std::shared_ptr<spdlog::sinks::udp_sink_mt> udp_sink = {};
      if (cfg->LogListenerAddr().empty() == false) {
        udp_sink = std::make_shared<spdlog::sinks::udp_sink_mt>(
            spdlog::sinks::udp_sink_config(cfg->LogListenerAddr(),
                                           cfg->LogListenerPort() + log_id));
        udp_sink->set_level(spdlog::level::info);
      }
      const spdlog::sinks_init_list sink_file = {file_sink};
      const spdlog::sinks_init_list sink_file_udp = {file_sink, udp_sink};
      logger_ = std::make_shared<spdlog::async_logger>(
          kCsvName.at(log_id), udp_sink ? sink_file_udp : sink_file,
          spdlog::thread_pool(), spdlog::async_overflow_policy::overrun_oldest);
      logger_->set_level(spdlog::level::info);
      logger_->set_pattern("%v");
    }
  }
#endif  //ENABLE_CSV_LOG
}

}  //namespace CsvLog