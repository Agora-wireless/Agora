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
#include <sstream>
#include <stdlib.h>
#include <string>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <vector>

int pin_to_core(int core_id);
void pin_to_core_with_offset(thread_type thread, int offset, int core_id);

template <class T>
struct EventHandlerContext {
    T* obj_ptr;
    int id;
#ifdef USE_ARGOS
    int radios;
#endif
};

template <class C, void* (C::*run_thread)(int)>
void* pthread_fun_wrapper(void* context)
{
    C* obj = reinterpret_cast<C*>(((EventHandlerContext<C>*)context)->obj_ptr);
    return (obj->*run_thread)(((EventHandlerContext<C>*)context)->id);
}

class Utils {
public:
    Utils();
    ~Utils();

    static std::vector<std::complex<int16_t>> double_to_int16(std::vector<std::vector<double>> in);
    static std::vector<std::complex<float>> uint32tocfloat(std::vector<uint32_t> in, const std::string& order);
    static std::vector<uint32_t> cint16_to_uint32(std::vector<std::complex<int16_t>> in, bool conj, std::string order);
    static std::vector<uint32_t> cfloat32_to_uint32(std::vector<std::complex<float>> in, bool conj, std::string order);
    static std::vector<std::vector<size_t>> loadSymbols(std::vector<std::string> frames, char sym);
    static void loadDevices(std::string filename, std::vector<std::string>& data);
    static void loadData(const char* filename, std::vector<std::complex<int16_t>>& data, int samples);
    static void loadData(const char* filename, std::vector<unsigned>& data, int samples);
    static void loadTDDConfig(const std::string filename, std::string& jconfig);
    static std::vector<std::string> split(const std::string& s, char delimiter);
    static void printVector(std::vector<std::complex<int16_t>>& data);
    static void writeBinaryFile(std::string name, size_t elem_size, size_t buffer_size, void *buff);
};
#endif
