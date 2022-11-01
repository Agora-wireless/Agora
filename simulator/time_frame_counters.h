/**
 * @file time_frame_counters.h
 * @brief Declaration file for the TimeFrameCounters class
 */
#ifndef TIME_FRAME_COUNTERS_H_
#define TIME_FRAME_COUNTERS_H_

#include <array>
#include <cstddef>

#include "gettime.h"
#include "message.h"
#include "symbols.h"

class TimeFrameCounters {
 public:
  TimeFrameCounters() = default;

  inline void Init(size_t max_symbol_count, size_t max_task_count = 0) {
    counter_.Init(max_symbol_count, max_task_count);
  }
  inline void Reset(size_t frame_id) { counter_.Reset(frame_id); }
  inline bool CompleteSymbol(size_t frame_id) {
    if (counter_.GetSymbolCount(frame_id) == 0) {
      const size_t frame_idx = frame_id % kFrameWnd;
      symbol_times_.at(frame_idx).first = GetTime::GetTimeUs();
    }
    const bool complete = counter_.CompleteSymbol(frame_id);
    if (complete) {
      const size_t frame_idx = frame_id % kFrameWnd;
      symbol_times_.at(frame_idx).second = GetTime::GetTimeUs();
    }
    return complete;
  }
  inline bool CompleteTask(size_t frame_id, size_t symbol_id) {
    if (counter_.GetTaskCount(frame_id, symbol_id) == 0) {
      const size_t frame_idx = frame_id % kFrameWnd;
      task_times_.at(frame_idx).at(symbol_id).first = GetTime::GetTimeUs();
    }
    const bool complete = counter_.CompleteTask(frame_id, symbol_id);
    if (complete) {
      const size_t frame_idx = frame_id % kFrameWnd;
      task_times_.at(frame_idx).at(symbol_id).second = GetTime::GetTimeUs();
    }
    return complete;
  }
  inline bool CompleteTask(size_t frame_id) {
    if (counter_.GetTaskCount(frame_id) == 0) {
      const size_t frame_idx = frame_id % kFrameWnd;
      symbol_times_.at(frame_idx).first = GetTime::GetTimeUs();
    }
    const bool complete = counter_.CompleteTask(frame_id);
    if (complete) {
      const size_t frame_idx = frame_id % kFrameWnd;
      symbol_times_.at(frame_idx).second = GetTime::GetTimeUs();
    }
    return complete;
  }
  inline double GetTaskTotalTimeMs(size_t frame_id, size_t symbol_id) const {
    const size_t frame_idx = frame_id % kFrameWnd;
    return (task_times_.at(frame_idx).at(symbol_id).second -
            task_times_.at(frame_idx).at(symbol_id).first) /
           1000.0f;
  }
  inline double GetTaskTotalTimeMs(size_t frame_id) const {
    const size_t frame_idx = frame_id % kFrameWnd;
    return (symbol_times_.at(frame_idx).second -
            symbol_times_.at(frame_idx).first) /
           1000.0f;
  };

  inline double GetTaskStartTimeUs(size_t frame_id, size_t symbol_id) const {
    const size_t frame_idx = frame_id % kFrameWnd;
    return task_times_.at(frame_idx).at(symbol_id).first;
  }
  inline double GetTaskStartTimeUs(size_t frame_id) const {
    const size_t frame_idx = frame_id % kFrameWnd;
    return symbol_times_.at(frame_idx).first;
  };

  inline double GetTaskEndTimeUs(size_t frame_id, size_t symbol_id) const {
    const size_t frame_idx = frame_id % kFrameWnd;
    return task_times_.at(frame_idx).at(symbol_id).second;
  }
  inline double GetTaskEndTimeUs(size_t frame_id) const {
    const size_t frame_idx = frame_id % kFrameWnd;
    return symbol_times_.at(frame_idx).second;
  };

 private:
  FrameCounters counter_;
  //First = Start, Second = End
  std::array<std::array<std::pair<double, double>, kMaxSymbols>, kFrameWnd>
      task_times_;
  std::array<std::pair<double, double>, kMaxSymbols> symbol_times_;
};

#endif  // TIME_FRAME_COUNTERS_H_
