/**
 * @file csv_logger.cc
 * @brief Implementation file for the CsvLogger class which records runtime
 * physical-layer performance into csv files. Enabled or disabled by cmake.
 */

#include "csv_logger.h"

#if defined(ENABLE_CSV_LOG)
namespace CsvLog {

std::shared_ptr<spdlog::logger> Create(const size_t dev_id,
                                       const size_t log_id) {
  std::string filename = kCsvName.at(log_id) + "-" + std::to_string(dev_id)
                                             + ".csv";
  std::remove(filename.c_str());
  auto logger = spdlog::create_async_nb<spdlog::sinks::basic_file_sink_mt>
                (kCsvName.at(log_id), filename);
  logger->set_level(spdlog::level::info);
  logger->set_pattern("%v");
  logger->info(kCsvHeader.at(log_id));
  return logger;
}

CsvLogger::CsvLogger(const std::string& name)
         : logger_(spdlog::get(name)) {
  if (logger_ == nullptr) {
    std::printf("logger not created\n");
  }
}

CsvLogger::CsvLogger(std::shared_ptr<spdlog::logger> logger)
         : logger_(logger) {
  if (logger_ == nullptr) {
    std::printf("logger not created\n");
  }
}

}      //namespace CsvLog
#endif //ENABLE_CSV_LOG