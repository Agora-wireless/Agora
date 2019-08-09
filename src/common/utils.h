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

#include <iostream>
#include <fstream>      // std::ifstream
#include <sstream>
#include <stdlib.h>
#include <cstdlib>
#include <cstddef>
#include <chrono>
#include <string>
#include <cstdint>
#include <complex>
#include <csignal>
#include <thread>
#include <pthread.h>
#include <unistd.h>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <vector>
#include <time.h>

int pin_to_core(int core_id);

class Utils
{
public:

    Utils();
    ~Utils();

    static std::vector<std::complex<int16_t>> double_to_int16(std::vector<std::vector<double>> in);
    static std::vector<uint32_t> cint16_to_uint32(std::vector<std::complex<int16_t>> in, bool conj, std::string order);
    static std::vector<uint32_t> cfloat32_to_uint32(std::vector<std::complex<float>> in, bool conj, std::string order);
    static std::vector<std::vector<size_t>> loadSymbols(std::vector<std::string> frames, char sym);
    static void loadDevices(std::string filename, std::vector<std::string> &data);
    static void loadData(const char* filename, std::vector<std::complex<int16_t>> &data, int samples);
    static void loadData(const char* filename, std::vector<unsigned> &data, int samples);
    static void loadTDDConfig(const std::string filename, std::string &jconfig);
    static std::vector<std::string> split(const std::string& s, char delimiter);
    static void printVector(std::vector<std::complex<int16_t>> &data);
};
#endif
