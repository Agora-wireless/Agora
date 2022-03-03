// Copyright (c) 2018-2020, Rice University
// RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

/**
 * @file utils.h
 * @brief Utility functions for file and text processing.
 * @author Rahman Doost-Mohamamdy: doost@rice.edu
 */

#ifndef UTILS_HEADER
#define UTILS_HEADER

#define UNUSED __attribute__((unused))
#define _unused(x) ((void)(x))
#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

#include "Symbols.hpp"
#include <atomic>
#include <chrono>
#include <complex>
#include <condition_variable>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream> // std::ifstream
#include <iostream>
#include <mutex>
#include <pthread.h>
#include <random>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <vector>

/* Pin this thread to core with global index = core_id */
int pin_to_core(int core_id);

/* Pin this thread to core (base_core_offset + thread_id) */
void pin_to_core_with_offset(ThreadType thread, int base_core_offset,
    int thread_id, bool verbose = true, bool hyper = false, int phy_core_num = 0);

template <class T> struct EventHandlerContext {
    T* obj_ptr;
    int id;
};

template <class C, void* (C::*run_thread)(int)>
void* pthread_fun_wrapper(void* context)
{
    EventHandlerContext<C>* eh_context = (EventHandlerContext<C>*)context;
    C* obj = reinterpret_cast<C*>(eh_context->obj_ptr);
    int id = eh_context->id;
    delete eh_context;
    return (obj->*run_thread)(id);
}

class Utils {
public:
    Utils();
    ~Utils();

    static std::vector<size_t> strToChannels(const std::string& channel);
    static std::vector<std::complex<int16_t>> double_to_cint16(
        std::vector<std::vector<double>> in);
    static std::vector<std::complex<float>> double_to_cfloat(
        std::vector<std::vector<double>> in);
    static std::vector<std::complex<float>> uint32tocfloat(
        std::vector<uint32_t> in, const std::string& order);
    static std::vector<uint32_t> cint16_to_uint32(
        std::vector<std::complex<int16_t>> in, bool conj, std::string order);
    static std::vector<uint32_t> cfloat32_to_uint32(
        std::vector<std::complex<float>> in, bool conj, std::string order);
    static std::vector<std::vector<size_t>> loadSymbols(
        std::vector<std::string> frames, char sym);
    static void loadDevices(
        std::string filename, std::vector<std::string>& data);
    static void loadData(const char* filename,
        std::vector<std::complex<int16_t>>& data, int samples);
    static void loadData(
        const char* filename, std::vector<unsigned>& data, int samples);
    static void loadTDDConfig(const std::string filename, std::string& jconfig);
    static std::vector<std::string> split(const std::string& s, char delimiter);
    static void printVector(std::vector<std::complex<int16_t>>& data);
    static void writeBinaryFile(
        std::string name, size_t elem_size, size_t buffer_size, void* buff);
};

/// roundup<N>(x) returns x rounded up to the next multiple of N. N must be
/// a power of two.
template <uint64_t PowerOfTwoNumber, typename T> static constexpr T roundup(T x)
{
    static_assert(is_power_of_two(PowerOfTwoNumber),
        "PowerOfTwoNumber must be a power of 2");
    return ((x) + T(PowerOfTwoNumber - 1)) & (~T(PowerOfTwoNumber - 1));
}

/// Check a condition at runtime. If the condition is false, throw exception.
/// This is faster than rt_assert(cond, std::string) as it avoids string
/// construction.
static inline void rt_assert(bool condition, const char* throw_str)
{
    if (unlikely(!condition))
        throw std::runtime_error(throw_str);
}

/// Check a condition at runtime. If the condition is false, throw exception.
/// This is faster than rt_assert(cond, std::string) as it avoids string
/// construction.
static inline void rt_assert(bool condition)
{
    if (unlikely(!condition))
        throw std::runtime_error("Error");
}

/// Check a condition at runtime. If the condition is false, throw exception.
static inline void rt_assert(bool condition, std::string throw_str)
{
    if (unlikely(!condition))
        throw std::runtime_error(throw_str);
}

/// Check a condition at runtime. If the condition is false, throw exception.
static inline void rt_assert(bool condition, std::string throw_str, char* s)
{
    if (unlikely(!condition)) {
        throw std::runtime_error(throw_str + std::string(s));
    }
}

/// Returns the greatest common divisor of `a` and `b`.
inline size_t gcd(size_t a, size_t b)
{
    if (a == 0)
        return b;
    return gcd(b % a, a);
}

/// Returns the least common multiple of `a` and `b`.
inline size_t lcm(size_t a, size_t b) { return (a * b) / gcd(a, b); }

/// A range type with an inclusive start bound and an exclusive end bound.
struct Range {
    const size_t start;
    const size_t end;

    /// Create a new Range with the given `start` and `end` values.
    /// `end` must be greater than or equal to `start`.
    Range(size_t start, size_t end)
        : start(start)
        , end(end)
    {
        rt_assert(end >= start, "Invalid range, end must be >= start");
    }

    /// Returns `true` if this range contains the given `value`.
    bool contains(size_t value) const
    {
        return (value >= start) && (value < end);
    }

    std::string to_string() const
    {
        std::ostringstream ret;
        ret << "[" << start << ":" << end << ")";
        return ret.str();
    }
};

class SlowRand {
    std::random_device rand_dev; // Non-pseudorandom seed for twister
    std::mt19937_64 mt;
    std::uniform_int_distribution<uint64_t> dist;

public:
    SlowRand()
        : mt(rand_dev())
        , dist(0, UINT64_MAX)
    {
    }

    inline uint64_t next_u64() { return dist(mt); }
};

class FastRand {
public:
    uint64_t seed;

    /// Create a FastRand using a seed from SlowRand
    FastRand()
    {
        SlowRand slow_rand;
        seed = slow_rand.next_u64();
    }

    inline uint32_t next_u32()
    {
        seed = seed * 1103515245 + 12345;
        return static_cast<uint32_t>(seed >> 32);
    }
};

inline size_t ceil_divide(size_t a, size_t b)
{
    return (a + b - 1) / b;
}
#endif
