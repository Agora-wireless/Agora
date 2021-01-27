// Copyright (c) 2018-2020, Rice University
// RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

/**
 * @file utils.h
 * @brief Utility functions for file and text processing.
 * @author Rahman Doost-Mohamamdy: doost@rice.edu
 */

#ifndef UTILS_HEADER_
#define UTILS_HEADER_

#define UNUSED __attribute__((unused))
#define unused(x) ((void)(x))
#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

#include <numa.h>
#include <pthread.h>
#include <unistd.h>

#include <armadillo>
#include <atomic>
#include <chrono>
#include <complex>
#include <condition_variable>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>  // std::ifstream
#include <iomanip>
#include <iostream>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "Symbols.hpp"

#define MAX_CORE_NUM (200)

void SetCpuLayoutOnNumaNodes(bool verbose = false);

size_t GetPhysicalCoreId(size_t core_id);

/* Pin this thread to core with global index = core_id */
int PinToCore(int core_id);

/* Pin this thread to core (base_core_offset + thread_id) */
void PinToCoreWithOffset(ThreadType thread, int base_core_offset, int thread_id,
                         bool verbose = true);

template <class T>
struct EventHandlerContext {
  T* obj_ptr_;
  int id_;
};

template <class C, void* (C::*run_thread)(int)>
void* PthreadFunWrapper(void* context) {
  auto* eh_context = static_cast<EventHandlerContext<C>*>(context);
  C* obj = reinterpret_cast<C*>(eh_context->obj_ptr_);
  int id = eh_context->id_;
  delete eh_context;
  return (obj->*run_thread)(id);
}

template <class C, void (C::*run_thread)(int)>
void PthreadFunWrapper(void* context) {
  auto* eh_context = static_cast<EventHandlerContext<C>*>(context);
  C* obj = reinterpret_cast<C*>(eh_context->obj_ptr_);
  int id = eh_context->id_;
  delete eh_context;
  return (obj->*run_thread)(id);
}

class Utils {
 public:
  Utils();
  ~Utils();

  static std::vector<size_t> StrToChannels(const std::string& channel);
  static std::vector<std::complex<int16_t>> DoubleToCint16(
      std::vector<std::vector<double>> in);
  static std::vector<std::complex<float>> DoubleToCfloat(
      std::vector<std::vector<double>> in);
  static std::vector<std::complex<float>> Uint32tocfloat(
      std::vector<uint32_t> in, const std::string& order);
  static std::vector<uint32_t> Cint16ToUint32(
      std::vector<std::complex<int16_t>> in, bool conj,
      const std::string& order);
  static std::vector<uint32_t> Cfloat32ToUint32(
      std::vector<std::complex<float>> in, bool conj, const std::string& order);
  static std::vector<std::vector<size_t>> LoadSymbols(
      std::vector<std::string> const& frames, char sym);
  static void LoadDevices(std::string filename, std::vector<std::string>& data);
  static void LoadData(const char* filename,
                       std::vector<std::complex<int16_t>>& data, int samples);
  static void LoadData(const char* filename, std::vector<unsigned>& data,
                       int samples);
  static void LoadTddConfig(const std::string& filename, std::string& jconfig);
  static std::vector<std::string> Split(const std::string& s, char delimiter);
  static void PrintVector(std::vector<std::complex<int16_t>>& data);
  static void WriteBinaryFile(const std::string& name, size_t elem_size,
                              size_t buffer_size, void* buff);
  static void PrintVec(arma::cx_fvec /*c*/, const std::string& /*ss*/);
  static void PrintMat(arma::cx_fmat /*c*/);
};

/// roundup<N>(x) returns x rounded up to the next multiple of N. N must be
/// a power of two.
template <uint64_t PowerOfTwoNumber, typename T>
static constexpr T Roundup(T x) {
  static_assert(IsPowerOfTwo(PowerOfTwoNumber),
                "PowerOfTwoNumber must be a power of 2");
  return ((x) + T(PowerOfTwoNumber - 1)) & (~T(PowerOfTwoNumber - 1));
}

/// Check a condition at runtime. If the condition is false, throw exception.
/// This is faster than rt_assert(cond, std::string) as it avoids string
/// construction.
static inline void RtAssert(bool condition, const char* throw_str) {
  if (unlikely(!condition)) {
    throw std::runtime_error(throw_str);
  }
}

/// Check a condition at runtime. If the condition is false, throw exception.
/// This is faster than rt_assert(cond, std::string) as it avoids string
/// construction.
static inline void RtAssert(bool condition) {
  if (unlikely(!condition)) {
    throw std::runtime_error("Error");
  }
}

/// Check a condition at runtime. If the condition is false, throw exception.
static inline void RtAssert(bool condition, const std::string& throw_str) {
  if (unlikely(!condition)) {
    throw std::runtime_error(throw_str);
  }
}

/// Check a condition at runtime. If the condition is false, throw exception.
static inline void RtAssert(bool condition, const std::string& throw_str,
                            char* s) {
  if (unlikely(!condition)) {
    throw std::runtime_error(throw_str + std::string(s));
  }
}

/// Returns the greatest common divisor of `a` and `b`.
inline size_t Gcd(size_t a, size_t b) {
  if (a == 0) {
    return b;
  }
  return Gcd(b % a, a);
}

/// Returns the least common multiple of `a` and `b`.
inline size_t Lcm(size_t a, size_t b) { return (a * b) / Gcd(a, b); }

/// A range type with an inclusive start bound and an exclusive end bound.
struct Range {
  const size_t kStart;
  const size_t kEnd;

  /// Create a new Range with the given `start` and `end` values.
  /// `end` must be greater than or equal to `start`.
  Range(size_t start, size_t end) : kStart(start), kEnd(end) {
    RtAssert(end >= start, "Invalid range, end must be >= start");
  }

  /// Returns `true` if this range contains the given `value`.
  bool Contains(size_t value) const {
    return (value >= kStart) && (value < kEnd);
  }

  std::string ToString() const {
    std::ostringstream ret;
    ret << "[" << kStart << ":" << kEnd << ")";
    return ret.str();
  }
};

class SlowRand {
  std::random_device rand_dev_;  // Non-pseudorandom seed for twister
  std::mt19937_64 mt_;
  std::uniform_int_distribution<uint64_t> dist_;

 public:
  SlowRand() : mt_(rand_dev_()), dist_(0, UINT64_MAX) {}

  inline uint64_t NextU64() { return dist_(mt_); }
};

class FastRand {
 public:
  uint64_t seed_;

  /// Create a FastRand using a seed from SlowRand
  FastRand() {
    SlowRand slow_rand;
    seed_ = slow_rand.NextU64();
  }

  inline uint32_t NextU32() {
    seed_ = seed_ * 1103515245 + 12345;
    return static_cast<uint32_t>(seed_ >> 32);
  }
};
#endif  // UTILS_HEADER_
