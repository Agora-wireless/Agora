#ifndef CSV_LOGGER_H_
#define CSV_LOGGER_H_

#include "armadillo"
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
  CsvLogger(int dev_id, enum CsvLogID log_id);
  inline void Write(size_t u1, size_t u2, size_t u3, float f1) {
    logger_->info(fmt::sprintf("%zu,%zu,%zu,%f", u1, u2, u3, f1));
  }
  inline void Write(size_t u1, size_t u2, size_t u3, float f1, float f2) {
    logger_->info(fmt::sprintf("%zu,%zu,%zu,%f,%f", u1, u2, u3, f1, f2));
  }
  inline void Write(size_t u1, size_t u2, size_t u3, size_t u4, float f1, float f2) {
    logger_->info(fmt::sprintf("%zu,%zu,%zu,%zu,%f,%f", u1, u2, u3, u4, f1, f2));
  }
  inline void Write(std::string str) {
    logger_->info(str);
  }
private:
  std::shared_ptr<spdlog::logger> logger_;
};

#else

class CsvLogger {
public:
  CsvLogger(int, enum CsvLogID);
  inline void Write(...) {}
};

#endif //ENABLE_CSV_LOG

class MatLogger : public CsvLogger {
public:
  MatLogger(int dev_id, enum CsvLogID log_id);
  void UpdateMatBuf(size_t frame_id, size_t sc_id, const arma::cx_fmat& mat_in);
  void SaveMatBuf();
private:
  int mat_idx_;
};

#endif //CSV_LOGGER_H_