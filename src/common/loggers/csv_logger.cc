/**
 * @file csv_logger.cc
 * @brief Implementation file for the CsvLogger class which records runtime
 * physical-layer performance into csv files. Enabled or disabled by cmake.
 */

#include "csv_logger.h"

#include "logger.h"
#include "utils.h"

namespace CsvLog {

CsvLogger::CsvLogger(const std::string& radio_id, const size_t log_id,
                     const std::string& sink_ip_addr) {
#if defined(ENABLE_CSV_LOG)
  if (log_id >= kAllLogs) {
    AGORA_LOG_ERROR("Invalid log id %zu in CsvLogger\n", log_id);
  } else {
    constexpr size_t kShortIdLen = 3;
    const std::string short_id =
        radio_id.substr(radio_id.length() - kShortIdLen);
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
        kCsvName.at(log_id) + "-" + short_id + ".csv", true);
    file_sink->set_level(spdlog::level::info);
    std::shared_ptr<spdlog::sinks::udp_sink_mt> udp_sink = {};
    if (sink_ip_addr.empty() == false && kUdpPortNum.at(log_id) > 0) {
      udp_sink = std::make_shared<spdlog::sinks::udp_sink_mt>(
          spdlog::sinks::udp_sink_config(sink_ip_addr, kUdpPortNum.at(log_id)));
      udp_sink->set_level(spdlog::level::info);
    }
    spdlog::sinks_init_list list_file_udp = {file_sink, udp_sink};
    spdlog::sinks_init_list list_file_only = {file_sink};
    logger_ = std::make_shared<spdlog::async_logger>(kCsvName.at(log_id),
        udp_sink ? list_file_udp : list_file_only, spdlog::thread_pool(),
        spdlog::async_overflow_policy::overrun_oldest);  //non-blocking
    logger_->set_level(spdlog::level::info);
    logger_->set_pattern("%v");
    logger_->info(kCsvHeader.at(log_id));
  }
#else
  unused(radio_id);
  unused(log_id);
#endif  //ENABLE_CSV_LOG
}
}  //namespace CsvLog