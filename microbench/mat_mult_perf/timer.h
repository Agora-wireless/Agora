#pragma once

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

/// Return the TSC
static inline size_t rdtsc() {
  uint64_t rax;
  uint64_t rdx;
  asm volatile("rdtsc" : "=a"(rax), "=d"(rdx));
  return static_cast<size_t>((rdx << 32) | rax);
}

/// An alias for rdtsc() to distinguish calls on the critical path
static const auto& dpath_rdtsc = rdtsc;

static void nano_sleep(size_t ns, double freq_ghz) {
  size_t start = rdtsc();
  size_t end = start;
  size_t upp = static_cast<size_t>(freq_ghz * ns);
  while (end - start < upp) end = rdtsc();
}

static double measure_rdtsc_freq() {
  struct timespec start, end;
  clock_gettime(CLOCK_REALTIME, &start);
  uint64_t rdtsc_start = rdtsc();

  // Do not change this loop! The hardcoded value below depends on this loop
  // and prevents it from being optimized out.
  uint64_t sum = 5;
  for (uint64_t i = 0; i < 1000000; i++) {
    sum += i + (sum + i) * (i % sum);
  }

  if (sum != 13580802877818827968ull) {
    std::exit(-1);
  }

  clock_gettime(CLOCK_REALTIME, &end);
  uint64_t clock_ns =
      static_cast<uint64_t>(end.tv_sec - start.tv_sec) * 1000000000 +
      static_cast<uint64_t>(end.tv_nsec - start.tv_nsec);
  uint64_t rdtsc_cycles = rdtsc() - rdtsc_start;

  double _freq_ghz = rdtsc_cycles * 1.0 / clock_ns;
  return _freq_ghz;
}

/// Convert cycles measured by rdtsc with frequence \p freq_ghz to seconds
static double to_sec(size_t cycles, double freq_ghz) {
  return (cycles / (freq_ghz * 1000000000));
}

/// Convert cycles measured by rdtsc with frequence \p freq_ghz to msec
static double to_msec(size_t cycles, double freq_ghz) {
  return (cycles / (freq_ghz * 1000000));
}

/// Convert cycles measured by rdtsc with frequence \p freq_ghz to usec
static double to_usec(size_t cycles, double freq_ghz) {
  return (cycles / (freq_ghz * 1000));
}

static size_t us_to_cycles(double us, double freq_ghz) {
  return static_cast<size_t>(us * 1000 * freq_ghz);
}

static size_t ns_to_cycles(double ns, double freq_ghz) {
  return static_cast<size_t>(ns * freq_ghz);
}

/// Convert cycles measured by rdtsc with frequence \p freq_ghz to nsec
static double to_nsec(size_t cycles, double freq_ghz) {
  return (cycles / freq_ghz);
}

/// Return seconds elapsed since timestamp \p t0
static double sec_since(const struct timespec& t0) {
  struct timespec t1;
  clock_gettime(CLOCK_REALTIME, &t1);
  return (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) / 1000000000.0;
}

/// Return nanoseconds elapsed since timestamp \p t0
static double ns_since(const struct timespec& t0) {
  struct timespec t1;
  clock_gettime(CLOCK_REALTIME, &t1);
  return (t1.tv_sec - t0.tv_sec) * 1000000000.0 + (t1.tv_nsec - t0.tv_nsec);
}

static double stddev(const std::vector<double> in_vec) {
  if (in_vec.size() == 0) return 0.0;
  double sum = std::accumulate(in_vec.begin(), in_vec.end(), 0.0);
  double mean = sum * 1.0 / in_vec.size();
  double sq_sum =
      std::inner_product(in_vec.begin(), in_vec.end(), in_vec.begin(), 0.0);
  return std::sqrt((sq_sum / in_vec.size()) - (mean * mean));
}

static double mean(const std::vector<double> in_vec) {
  if (in_vec.empty()) return 0.0;
  double sum = std::accumulate(in_vec.begin(), in_vec.end(), 0.0);
  return sum * 1.0 / in_vec.size();
}

/// Simple time that uses RDTSC
class TscTimer {
 public:
  size_t start_tsc = 0;
  double freq_ghz;
  std::vector<double> ms_duration_vec;

  TscTimer(size_t n_timestamps, double freq_ghz) : freq_ghz(freq_ghz) {
    ms_duration_vec.reserve(n_timestamps);
  }

  inline void start() { start_tsc = rdtsc(); }
  inline void stop() {
    ms_duration_vec.push_back(to_msec(rdtsc() - start_tsc, freq_ghz));
  }

  void reset() { ms_duration_vec.clear(); }
  double stddev_msec() { return stddev(ms_duration_vec); }
  double avg_msec() { return mean(ms_duration_vec); }
  double avg_usec() { return 1000 * mean(ms_duration_vec); }
};
