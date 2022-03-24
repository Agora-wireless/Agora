#ifndef CSV_LOGGER_H_
#define CSV_LOGGER_H_

#include <armadillo>
#include "logger.h"

enum CsvLogID {
  kCsvLogDLPSNR,
  kCsvLogEVMSNR,
  kCsvLogBERSER,
  kMatLogCSI,
  kMatLogDLZF
};

#if defined(ENABLE_CSV_LOG)

#include "spdlog/sinks/basic_file_sink.h"

class CsvLogger {
public:
  CsvLogger(size_t dev_id, enum CsvLogID log_id);
  inline void Write(const char *format, ...) {
    char str[128];
    va_list args;
    va_start(args, format);
    sprintf(str, format, args);
    logger_->info(str);
    va_end(args);
  }
private:
  std::shared_ptr<spdlog::logger> logger_;
};

#else

class CsvLogger {
public:
  CsvLogger(size_t, enum CsvLogID);
  inline void Write(const char *, ...) {}
};

#endif //ENABLE_CSV_LOG

class MatLogger : public CsvLogger {
public:
  MatLogger(size_t dev_id, enum CsvLogID log_id);
  void UpdateMatBuf(size_t frame_id, size_t sc_id, const arma::cx_fmat& mat_in);
  void SaveMatBuf();
private:
  size_t mat_idx_;
  bool is_active_;
};

#endif //CSV_LOGGER_H_