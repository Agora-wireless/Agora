#ifndef CSV_LOGGER_H_
#define CSV_LOGGER_H_

#include "logger.h"

enum CsvLogID {
  kCsvLogDLPSNR,
  kCsvLogEVMSNR,
  kCsvLogBERSER,
  kCsvLogCSI,
  kCsvLogDLZF,
  kCsvLogNUM
};

#if defined(CSV_LOG_LEVEL)

#include "spdlog/sinks/basic_file_sink.h"

extern std::shared_ptr<spdlog::logger> csv_logger[kCsvLogNUM];
extern void CsvLogInit(size_t dev_id, size_t log_id);

#define CSV_LOG_INIT(DEV_ID, LOG_ID) CsvLogInit(DEV_ID, LOG_ID)
#define CSV_LOG(LOG_ID, ...) \
  if (csv_logger[LOG_ID] != nullptr) { \
    csv_logger[LOG_ID]->CSV_LOG_LEVEL(fmt::sprintf(__VA_ARGS__)); \
  }
#define CSV_LOG_VAR(VAR, EXPR) VAR = EXPR

#else

#define CSV_LOG_INIT(DEV_ID, LOG_ID)
#define CSV_LOG(LOG_ID, ...)
#define CSV_LOG_VAR(VAR, EXPR)

#endif //CSV_LOG_LEVEL

#endif //CSV_LOGGER_H_