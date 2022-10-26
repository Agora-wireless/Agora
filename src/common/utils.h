// Copyright (c) 2018-2021, Rice University
// RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

/**
 * @file utils.h
 * @brief Utility functions for file and text processing.
 * @author Rahman Doost-Mohamamdy: doost@rice.edu
 */

#ifndef UTILS_H_
#define UTILS_H_

#define UNUSED __attribute__((unused))
#define unused(x) ((void)(x))
#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

#include <complex>
#include <cstddef>
#include <cstdint>
#include <fstream>  // std::ifstream
#include <random>
#include <string>
#include <vector>

#include "armadillo"
#include "symbols.h"

// Default argument is to exclude core 0 from the list
void SetCpuLayoutOnNumaNodes(
    bool verbose = false,
    const std::vector<size_t>& cores_to_exclude = std::vector<size_t>(1, 0));

size_t GetPhysicalCoreId(size_t core_id);

/* Pin this thread to core with global index = core_id */
int PinToCore(size_t core_id);

/* Pin this thread to core (base_core_offset + thread_id) */
void PinToCoreWithOffset(ThreadType thread, size_t base_core_offset,
                         size_t thread_id, bool allow_reuse = false,
                         bool verbose = false);

void PrintCoreAssignmentSummary();

template <class T>
struct EventHandlerContext {
  T* obj_ptr_;
  int id_;
};

class Utils {
 public:
  Utils();
  ~Utils();

  static std::vector<size_t> StrToChannels(const std::string& channel);
  static std::vector<std::complex<int16_t>> DoubleToCint16(
      const std::vector<std::vector<double>>& in);
  static std::vector<std::complex<float>> DoubleToCfloat(
      const std::vector<std::vector<double>>& in);
  static std::vector<std::complex<float>> Uint32tocfloat(
      const std::vector<uint32_t>& in, const std::string& order);
  static std::vector<std::complex<float>> Cint16ToCfloat32(
      const std::vector<std::complex<int16_t>>& in);
  static std::vector<uint32_t> Cint16ToUint32(
      const std::vector<std::complex<int16_t>>& in, bool conj,
      const std::string& order);
  static std::vector<uint32_t> Cfloat32ToUint32(
      const std::vector<std::complex<float>>& in, bool conj,
      const std::string& order);
  static std::vector<std::vector<size_t>> LoadSymbols(
      std::vector<std::string> const& frames, char sym);
  static void LoadDevices(std::string filename, std::vector<std::string>& data);
  static void LoadData(const char* filename,
                       std::vector<std::complex<int16_t>>& data, int samples);
  static void LoadData(const char* filename, std::vector<unsigned>& data,
                       int samples);
  static void LoadTddConfig(const std::string& filename, std::string& jconfig);
  static std::vector<std::string> Split(const std::string& s, char delimiter);
  static void PrintVector(const std::vector<std::complex<int16_t>>& data);
  static void WriteBinaryFile(const std::string& name, size_t elem_size,
                              size_t buffer_size, void* buff);
  static void PrintVec(const arma::cx_fvec& c, const std::string& ss);
  static void SaveVec(const arma::cx_fvec& c, const std::string& filename,
                      const std::string& /*ss*/, const bool /*append*/);
  static void PrintMat(const arma::cx_fmat& c, const std::string& ss);
  static void SaveMat(const arma::cx_fmat& c, const std::string& filename,
                      const std::string& ss, const bool append);
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
  const size_t start_;
  const size_t end_;

  /// Create a new Range with the given `start` and `end` values.
  /// `end` must be greater than or equal to `start`.
  Range(size_t start, size_t end) : start_(start), end_(end) {
    RtAssert(end >= start, "Invalid range, end must be >= start");
  }

  /// Returns `true` if this range contains the given `value`.
  bool Contains(size_t value) const {
    return (value >= start_) && (value < end_);
  }

  std::string ToString() const {
    std::ostringstream ret;
    ret << "[" << start_ << ":" << end_ << ")";
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
#endif  // UTILS_H_
