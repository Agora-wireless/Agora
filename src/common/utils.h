/*
 Copyright (c) 2018-2019, Rice University
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Rahman Doost-Mohamamdy: doost@rice.edu

---------------------------------------------------------------------
 Utils functions
---------------------------------------------------------------------
*/

#ifndef UTILS_HEADER
#define UTILS_HEADER

#define UNUSED __attribute__((unused))
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
#include <emmintrin.h>
#include <fstream> // std::ifstream
#include <immintrin.h>
#include <iostream>
#include <mutex>
#include <pthread.h>
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
void pin_to_core_with_offset(
    ThreadType thread, int base_core_offset, int thread_id);

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
    static void cvtShortToFloatSIMD(
        short* in_buf, float*& out_buf, size_t length);
    static std::vector<size_t> strToChannels(const std::string& channel);
    static std::vector<std::complex<int16_t>> double_to_cint16(
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

#endif
