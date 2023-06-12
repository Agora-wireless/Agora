/**
 * @file logger.h
 * @brief Logging macros that can be optimized out by the compiler
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#define AGORA_LOG_LEVEL_OFF (0)
#define AGORA_LOG_LEVEL_ERROR (1)
#define AGORA_LOG_LEVEL_WARN (2)
#define AGORA_LOG_LEVEL_INFO (3)
#define AGORA_LOG_LEVEL_FRAME (4)
#define AGORA_LOG_LEVEL_SYMBOL (5)
#define AGORA_LOG_LEVEL_TRACE (6)

#if !defined(AGORA_LOG_LEVEL)
#define AGORA_LOG_LEVEL (AGORA_LOG_LEVEL_ERROR)
#endif

#define AGORA_LOG_DEFAULT_STREAM (stdout)
#define AGORA_LOG_STREAM (AGORA_LOG_DEFAULT_STREAM)

#if !defined(USE_SPDLOG)

#include <ctime>
#include <string>

// If AGORA_LOG_LEVEL is not defined, default to the highest level so that
// Log messages with "FRAME" or higher verbosity get written to
// mlpd_trace_file_or_default_stream. This can be stdout for basic debugging, or
// a file named "trace_file" for more involved debugging.
#define Agora_trace_file_or_default_stream (AGORA_LOG_DEFAULT_STREAM)

#define AGORA_LOG_INIT() ((void)0);
#define AGORA_LOG_SHUTDOWN() ((void)0);

#if AGORA_LOG_LEVEL >= AGORA_LOG_LEVEL_ERROR
#define AGORA_LOG_ERROR(...)                                             \
  AgoraOutputLogHeader(AGORA_LOG_DEFAULT_STREAM, AGORA_LOG_LEVEL_ERROR); \
  std::fprintf(AGORA_LOG_DEFAULT_STREAM, __VA_ARGS__);                   \
  std::fflush(AGORA_LOG_DEFAULT_STREAM)
#else
#define AGORA_LOG_ERROR(...) ((void)0)
#endif

#if AGORA_LOG_LEVEL >= AGORA_LOG_LEVEL_WARN
#define AGORA_LOG_WARN(...)                                             \
  AgoraOutputLogHeader(AGORA_LOG_DEFAULT_STREAM, AGORA_LOG_LEVEL_WARN); \
  std::fprintf(AGORA_LOG_DEFAULT_STREAM, __VA_ARGS__);                  \
  std::fflush(AGORA_LOG_DEFAULT_STREAM)
#else
#define AGORA_LOG_WARN(...) ((void)0)
#endif

#if AGORA_LOG_LEVEL >= AGORA_LOG_LEVEL_INFO
#define AGORA_LOG_INFO(...)                                             \
  AgoraOutputLogHeader(AGORA_LOG_DEFAULT_STREAM, AGORA_LOG_LEVEL_INFO); \
  std::fprintf(AGORA_LOG_DEFAULT_STREAM, __VA_ARGS__);                  \
  std::fflush(AGORA_LOG_DEFAULT_STREAM)
#else
#define AGORA_LOG_INFO(...) ((void)0)
#endif

#if AGORA_LOG_LEVEL >= AGORA_LOG_LEVEL_FRAME
#define AGORA_LOG_FRAME(...)                                     \
  AgoraOutputLogHeader(Agora_trace_file_or_default_stream,       \
                       AGORA_LOG_LEVEL_FRAME);                   \
  std::fprintf(Agora_trace_file_or_default_stream, __VA_ARGS__); \
  std::fflush(Agora_trace_file_or_default_stream)
#else
#define AGORA_LOG_FRAME(...) ((void)0)
#endif

#if AGORA_LOG_LEVEL >= AGORA_LOG_LEVEL_SYMBOL
#define AGORA_LOG_SYMBOL(...)                                    \
  AgoraOutputLogHeader(Agora_trace_file_or_default_stream,       \
                       AGORA_LOG_LEVEL_SYMBOL);                  \
  std::fprintf(Agora_trace_file_or_default_stream, __VA_ARGS__); \
  std::fflush(Agora_trace_file_or_default_stream)
#else
#define AGORA_LOG_SYMBOL(...) ((void)0)
#endif

#if AGORA_LOG_LEVEL >= AGORA_LOG_LEVEL_TRACE
#define AGORA_LOG_TRACE(...)                                     \
  AgoraOutputLogHeader(Agora_trace_file_or_default_stream,       \
                       AGORA_LOG_LEVEL_TRACE);                   \
  std::fprintf(Agora_trace_file_or_default_stream, __VA_ARGS__); \
  std::fflush(Agora_trace_file_or_default_stream)
#else
#define AGORA_LOG_TRACE(...) ((void)0)
#endif

#else

#include "spdlog/async.h"
#include "spdlog/fmt/bundled/printf.h"  // support for printf-style
#include "spdlog/pattern_formatter.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

constexpr size_t kLogThreadPoolQueueSize = 32768;
constexpr size_t kLogThreadCount = 1;

#define AGORA_LOG_INIT()                                              \
  spdlog::init_thread_pool(kLogThreadPoolQueueSize, kLogThreadCount); \
  spdlog::set_default_logger(                                         \
      spdlog::create_async_nb<spdlog::sinks::stdout_color_sink_mt>(   \
          "console"));                                                \
  auto f = std::make_unique<spdlog::pattern_formatter>(               \
      spdlog::pattern_time_type::utc, std::string(""));               \
  f->set_pattern("[%S:%f][%^%L%$] %v");                               \
  spdlog::set_formatter(std::move(f));                                \
  spdlog::set_level(SPDLOG_LEVEL);

#define AGORA_LOG_SHUTDOWN() spdlog::shutdown();

#if AGORA_LOG_LEVEL >= AGORA_LOG_LEVEL_ERROR
#define AGORA_LOG_ERROR(...) spdlog::error(fmt::sprintf(__VA_ARGS__));
#else
#define AGORA_LOG_ERROR(...) ((void)0)
#endif

#if AGORA_LOG_LEVEL >= AGORA_LOG_LEVEL_WARN
#define AGORA_LOG_WARN(...) spdlog::warn(fmt::sprintf(__VA_ARGS__));
#else
#define AGORA_LOG_WARN(...) ((void)0)
#endif

#if AGORA_LOG_LEVEL >= AGORA_LOG_LEVEL_INFO
#define AGORA_LOG_INFO(...) spdlog::info(fmt::sprintf(__VA_ARGS__));
#else
#define AGORA_LOG_INFO(...) ((void)0)
#endif

#if AGORA_LOG_LEVEL >= AGORA_LOG_LEVEL_FRAME
#define AGORA_LOG_FRAME(...) spdlog::debug(fmt::sprintf(__VA_ARGS__));
#else
#define AGORA_LOG_FRAME(...) ((void)0)
#endif

#if AGORA_LOG_LEVEL >= AGORA_LOG_LEVEL_SYMBOL
#define AGORA_LOG_SYMBOL(...) spdlog::debug(fmt::sprintf(__VA_ARGS__));
#else
#define AGORA_LOG_SYMBOL(...) ((void)0)
#endif

#if AGORA_LOG_LEVEL >= AGORA_LOG_LEVEL_TRACE
#define AGORA_LOG_TRACE(...) spdlog::trace(fmt::sprintf(__VA_ARGS__));
#else
#define AGORA_LOG_TRACE(...) ((void)0)
#endif

#endif

/// Return decent-precision time formatted as seconds:microseconds
static std::string AgoraGetFormattedTime() {
  struct timespec t;
  clock_gettime(CLOCK_REALTIME, &t);
  char buf[20];
  uint32_t seconds = t.tv_sec % 100;  // Rollover every 100 seconds
  uint32_t usec = t.tv_nsec / 1000;

  std::sprintf(buf, "%u:%06u", seconds, usec);
  return std::string(buf);
}
// Output log message header
static inline void AgoraOutputLogHeader(FILE* stream, int level) {
  std::string formatted_time = AgoraGetFormattedTime();

  const char* type;
  switch (level) {
    case AGORA_LOG_LEVEL_ERROR:
      type = "ERROR";
      break;
    case AGORA_LOG_LEVEL_WARN:
      type = "WARNG";
      break;
    case AGORA_LOG_LEVEL_INFO:
      type = "INFOR";
      break;
    case AGORA_LOG_LEVEL_FRAME:
      type = "FRAME";
      break;
    case AGORA_LOG_LEVEL_SYMBOL:
      type = "SBFRM";
      break;
    case AGORA_LOG_LEVEL_TRACE:
      type = "TRACE";
      break;
    default:
      type = "UNKWN";
  }
  std::fprintf(stream, "%s %s: ", formatted_time.c_str(), type);
}

/// Return true if the logging verbosity is reasonable for non-developer users
/// of Agora
static inline bool IsLogLevelReasonable() {
  return AGORA_LOG_LEVEL <= AGORA_LOG_LEVEL_INFO;
}
#endif  // LOGGER_H_