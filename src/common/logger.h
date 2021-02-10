#pragma once

/***************************************************************************
 *   Copyright (C) 2008 by H-Store Project                                 *
 *   Brown University                                                      *
 *   Massachusetts Institute of Technology                                 *
 *   Yale University                                                       *
 *                                                                         *
 *   This software may be modified and distributed under the terms         *
 *   of the MIT license.  See the LICENSE file for details.                *
 *                                                                         *
 *   Copyright (C) 2018 by eRPC Project                                    *
 *   Carnegie Mellon University                                            *
 ***************************************************************************/

/**
 * @file logger.h
 * @brief Logging macros that can be optimized out by the compiler
 * @author Hideaki, modified by Anuj
 */

#include <ctime>
#include <string>

// Log levels: higher means more verbose
#define MLPD_LOG_LEVEL_OFF 0
#define MLPD_LOG_LEVEL_ERROR 1  // Only fatal conditions
#define MLPD_LOG_LEVEL_WARN 2  // Conditions from which it's possible to recover
#define MLPD_LOG_LEVEL_INFO 3  // Reasonable to log (e.g., management packets)
#define MLPD_LOG_LEVEL_FRAME 4   // Per-frame logging
#define MLPD_LOG_LEVEL_SYMBOL 5  // Per-symbol logging
#define MLPD_LOG_LEVEL_TRACE 6   // Reserved for very high verbosity

#define MLPD_LOG_DEFAULT_STREAM stdout

// Log messages with "FRAME" or higher verbosity get written to
// mlpd_trace_file_or_default_stream. This can be stdout for basic debugging, or
// a file named "trace_file" for more involved debugging.

//#define mlpd_trace_file_or_default_stream trace_file
#define mlpd_trace_file_or_default_stream MLPD_LOG_DEFAULT_STREAM

// If MLPD_LOG_LEVEL is not defined, default to the highest level so that
// YouCompleteMe does not report compilation errors
#ifndef MLPD_LOG_LEVEL
#define MLPD_LOG_LEVEL MLPD_LOG_LEVEL_TRACE
#endif

#if MLPD_LOG_LEVEL >= MLPD_LOG_LEVEL_ERROR
#define MLPD_ERROR(...)                                               \
  MlpdOutputLogHeader(MLPD_LOG_DEFAULT_STREAM, MLPD_LOG_LEVEL_ERROR); \
  std::fprintf(MLPD_LOG_DEFAULT_STREAM, __VA_ARGS__);                 \
  fflush(MLPD_LOG_DEFAULT_STREAM)
#else
#define MLPD_ERROR(...) ((void)0)
#endif

#if MLPD_LOG_LEVEL >= MLPD_LOG_LEVEL_WARN
#define MLPD_WARN(...)                                               \
  MlpdOutputLogHeader(MLPD_LOG_DEFAULT_STREAM, MLPD_LOG_LEVEL_WARN); \
  std::fprintf(MLPD_LOG_DEFAULT_STREAM, __VA_ARGS__);                \
  fflush(MLPD_LOG_DEFAULT_STREAM)
#else
#define MLPD_WARN(...) ((void)0)
#endif

#if MLPD_LOG_LEVEL >= MLPD_LOG_LEVEL_INFO
#define MLPD_INFO(...)                                               \
  MlpdOutputLogHeader(MLPD_LOG_DEFAULT_STREAM, MLPD_LOG_LEVEL_INFO); \
  std::fprintf(MLPD_LOG_DEFAULT_STREAM, __VA_ARGS__);                \
  fflush(MLPD_LOG_DEFAULT_STREAM)
#else
#define MLPD_INFO(...) ((void)0)
#endif

#if MLPD_LOG_LEVEL >= MLPD_LOG_LEVEL_FRAME
#define MLPD_FRAME(...)                                         \
  MlpdOutputLogHeader(mlpd_trace_file_or_default_stream,        \
                      MLPD_LOG_LEVEL_FRAME);                    \
  std::fprintf(mlpd_trace_file_or_default_stream, __VA_ARGS__); \
  fflush(mlpd_trace_file_or_default_stream)
#else
#define MLPD_FRAME(...) ((void)0)
#endif

#if MLPD_LOG_LEVEL >= MLPD_LOG_LEVEL_SYMBOL
#define MLPD_SYMBOL(...)                                        \
  MlpdOutputLogHeader(mlpd_trace_file_or_default_stream,        \
                      MLPD_LOG_LEVEL_SYMBOL);                   \
  std::fprintf(mlpd_trace_file_or_default_stream, __VA_ARGS__); \
  fflush(mlpd_trace_file_or_default_stream)
#else
#define MLPD_SYMBOL(...) ((void)0)
#endif

#if MLPD_LOG_LEVEL >= MLPD_LOG_LEVEL_TRACE
#define MLPD_TRACE(...)                                         \
  MlpdOutputLogHeader(mlpd_trace_file_or_default_stream,        \
                      MLPD_LOG_LEVEL_TRACE);                    \
  std::fprintf(mlpd_trace_file_or_default_stream, __VA_ARGS__); \
  fflush(mlpd_trace_file_or_default_stream)
#else
#define MLPD_TRACE(...) ((void)0)
#endif

/// Return decent-precision time formatted as seconds:microseconds
static std::string MlpdGetFormattedTime() {
  struct timespec t;
  clock_gettime(CLOCK_REALTIME, &t);
  char buf[20];
  uint32_t seconds = t.tv_sec % 100;  // Rollover every 100 seconds
  uint32_t usec = t.tv_nsec / 1000;

  sprintf(buf, "%u:%06u", seconds, usec);
  return std::string(buf);
}

// Output log message header
static inline void MlpdOutputLogHeader(FILE* stream, int level) {
  std::string formatted_time = MlpdGetFormattedTime();

  const char* type;
  switch (level) {
    case MLPD_LOG_LEVEL_ERROR:
      type = "ERROR";
      break;
    case MLPD_LOG_LEVEL_WARN:
      type = "WARNG";
      break;
    case MLPD_LOG_LEVEL_INFO:
      type = "INFOR";
      break;
    case MLPD_LOG_LEVEL_FRAME:
      type = "FRAME";
      break;
    case MLPD_LOG_LEVEL_SYMBOL:
      type = "SBFRM";
      break;
    case MLPD_LOG_LEVEL_TRACE:
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
  return MLPD_LOG_LEVEL <= MLPD_LOG_LEVEL_INFO;
}
