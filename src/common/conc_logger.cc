#include "conc_logger.h"

void Logger::InitInstance(std::string fname) {
  if (instance_ != nullptr) return;
  // Call the constructor of the static instance
  instance_ = new Logger(fname);

  // Instantiate the thread
  instance_->worker_ = std::thread(&Logger::WorkerFunc_, instance_);
}

void Logger::Log(std::string log_string) {
  instance_->log_buffer_.enqueue(log_string);
}

Logger::~Logger() {
  // Wait for thread to exit
  instance_->done_ = true;
  instance_->worker_.join();
}

Logger::Logger(std::string fname) {
  // Borrow Json read from utils for now
  std::string json_config_str;
  Utils::LoadTddConfig(fname, json_config_str);
  const json json_config = json::parse(json_config_str, nullptr, true, true);
  ParseConfFile_(json_config);

  // Initiate the worker loop
  done_ = false;
}

bool Logger::IsViableLog_(std::string buf) const {
  // TODO: Check if it matched viability from config file
  return true;
}

void Logger::ParseConfFile_(const json &json_config) {
  // Read multiple flags
  for (auto conf : json_config.items()) {
    if (conf.key() == "kDebugOutstream") {
      // TODO: Instantiate the fd_
    } else if (conf.value()) {
      debug_level_.insert(conf.key());
    }
  }
}

void Logger::WorkerFunc_() {
  std::string buf;
  while (!done_) {
    if (!log_buffer_.try_dequeue(buf) || !IsViableLog_(buf)) continue;

    if (write(fd_, buf.c_str(), buf.length()) < 0) {
      fprintf(stderr, "[Logger] Error in writing to fd: %s\n", strerror(errno));
      _exit(1);
    }
  }

  if (close(fd_) < 0) {
    fprintf(stderr, "[Logger] Error in closing fd: %s\n", strerror(errno));
    _exit(1);
  }
}
