#ifndef CONC_LOGGER_H
#define CONC_LOGGER_H

#include <errno.h>
#include <string.h>
#include <unordered_set>
#include <unistd.h>

#include <iostream>

#include "concurrentqueue.h"
#include "nlohmann/json.hpp"
#include "utils.h"

using json = nlohmann::json;

class Logger {
 public:
  // Delete Copy constructor
  Logger(const Logger &) = delete;

  static void InitInstance(std::string);
  static inline void Log(std::string);
  ~Logger();

 private:
  Logger(std::string);
  inline bool IsViableLog_(std::string) const;
  void SetupOutstream_(std::string &);
  void WorkerFunc_();
  void ParseConfFile_(const json &);

  // Singleton static instance
  static Logger *instance_;

  // Volatile boolean to signal worker thread to stop
  volatile bool done_;

  // The concurrent queue to collect all the logs
  moodycamel::ConcurrentQueue<std::string> log_buffer_;

  // The worker thread
  std::thread worker_;

  // The logger output stream
  int fd_;

  // Save configuration flags
  std::unordered_set<std::string> debug_level_;
};

#endif /* CONC_LOGGER_H */