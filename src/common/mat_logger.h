/**
 * @file mat_logger.h
 * @brief Declaration file for the MatLogger class which records runtime
 * zero-forcing matrices into csv files. Enabled or disabled by cmake.
 */

#ifndef MAT_LOGGER_H_
#define MAT_LOGGER_H_

#include "armadillo"
#include "csv_logger.h"

namespace CsvLog {

constexpr size_t kFrames = 1000;
constexpr size_t kSCs = 304;
constexpr size_t kBSAnts = 8;
constexpr size_t kUEAnts = 1;

#if defined(ENABLE_MAT_LOG)
using MatBuffer = std::array<
    std::array<std::array<std::array<arma::cx_float, kUEAnts>, kBSAnts>, kSCs>,
    kFrames>;
class MatLogger : public CsvLogger {
 public:
  MatLogger(const std::string& name, MatBuffer& mat_buffer);
  MatLogger(std::shared_ptr<spdlog::logger> logger, MatBuffer& mat_buffer);
  bool UpdateMatBuf(const size_t frame_id, const size_t sc_id,
                    const arma::cx_fmat& mat_in);
  void SaveMatBuf();

 private:
  MatBuffer& mat_buffer_;
};
#else
using MatBuffer = void*;
class MatLogger {
 public:
  MatLogger(const std::string&, MatBuffer&);
#if defined(ENABLE_CSV_LOG)
  MatLogger(std::shared_ptr<spdlog::logger>, MatBuffer&);
#else
  MatLogger(void*, MatBuffer&);
#endif
  bool UpdateMatBuf(const size_t, const size_t, const arma::cx_fmat&);
  void SaveMatBuf();
};
#endif  //ENABLE_MAT_LOG

using MatLoggerArray = std::array<std::unique_ptr<MatLogger>, kMatLogs>;

}  //namespace CsvLog
#endif  //MAT_LOGGER_H_