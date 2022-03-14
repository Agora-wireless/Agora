// Copyright (c) 2018-2020, Rice University
// RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

/**
 * @file utils.cpp
 * @brief Utility functions for file and text processing.
 */

#include "utils.h"

int pin_to_core(int core_id)
{
    int num_cores = sysconf(_SC_NPROCESSORS_ONLN);
    if (core_id < 0 || core_id >= num_cores)
        return -1;

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);

    pthread_t current_thread = pthread_self();
    return pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
}

void pin_to_core_with_offset(
    ThreadType thread_type, int core_offset, int thread_id, bool verbose, bool hyper, int phy_core_num)
{
    if (!kEnableThreadPinning)
        return;

    // int actual_core_id = core_offset + thread_id;
    int actual_core_id;
    if (hyper) {
        actual_core_id = thread_id % 2 == 0 ? core_offset + thread_id / 2 : 
            core_offset + thread_id / 2 + phy_core_num;
    } else {
        actual_core_id = core_offset + thread_id;
    }
    int num_cores = sysconf(_SC_NPROCESSORS_ONLN);

    /* Reserve core 0 for kernel threads */
    if (actual_core_id >= num_cores) {
        actual_core_id = (actual_core_id % num_cores) + 1;
    }

    if (pin_to_core(actual_core_id) != 0) {
        fprintf(stderr,
            "%s thread %d: failed to pin to core %d. Exiting. "
            "This can happen if the machine has insufficient cores. "
            "Set kEnableThreadPinning to false to run Agora to run despite "
            "this - performance will be low.\n",
            thread_type_str(thread_type).c_str(), thread_id, actual_core_id);
        exit(0);
    } else {
        if (verbose) {
            printf("%s thread %d: pinned to core %d\n",
                thread_type_str(thread_type).c_str(), thread_id,
                actual_core_id);
        }
    }
}

std::vector<size_t> Utils::strToChannels(const std::string& channel)
{
    std::vector<size_t> channels;
    if (channel == "A")
        channels = { 0 };
    else if (channel == "B")
        channels = { 1 };
    else
        channels = { 0, 1 };
    return (channels);
}

std::vector<std::complex<int16_t>> Utils::double_to_cint16(
    std::vector<std::vector<double>> in)
{
    int len = in[0].size();
    std::vector<std::complex<int16_t>> out(len, 0);
    for (int i = 0; i < len; i++)
        out[i] = std::complex<int16_t>(
            (int16_t)(in[0][i] * 32768), (int16_t)(in[1][i] * 32768));
    return out;
}

std::vector<std::complex<float>> Utils::double_to_cfloat(
    std::vector<std::vector<double>> in)
{
    int len = in[0].size();
    std::vector<std::complex<float>> out(len, 0);
    for (int i = 0; i < len; i++)
        out[i] = std::complex<float>(in[0][i], in[1][i]);
    return out;
}

std::vector<std::complex<float>> Utils::uint32tocfloat(
    std::vector<uint32_t> in, const std::string& order)
{
    int len = in.size();
    std::vector<std::complex<float>> out(len, 0);
    for (size_t i = 0; i < in.size(); i++) {
        int16_t arr_hi_int = (int16_t)(in[i] >> 16);
        int16_t arr_lo_int = (int16_t)(in[i] & 0x0FFFF);

        float arr_hi = (float)arr_hi_int / 32768.0;
        float arr_lo = (float)arr_lo_int / 32768.0;

        if (order == "IQ") {
            std::complex<float> csamp(arr_hi, arr_lo);
            out[i] = csamp;
        } else if (order == "QI") {
            std::complex<float> csamp(arr_lo, arr_hi);
            out[i] = csamp;
        }
    }
    return out;
}

std::vector<uint32_t> Utils::cint16_to_uint32(
    std::vector<std::complex<int16_t>> in, bool conj, std::string order)
{
    std::vector<uint32_t> out(in.size(), 0);
    for (size_t i = 0; i < in.size(); i++) {
        uint16_t re = (uint16_t)in[i].real();
        uint16_t im = (uint16_t)(conj ? -in[i].imag() : in[i].imag());
        if (order == "IQ")
            out[i] = (uint32_t)re << 16 | im;
        else if (order == "QI")
            out[i] = (uint32_t)im << 16 | re;
    }
    return out;
}

std::vector<uint32_t> Utils::cfloat32_to_uint32(
    std::vector<std::complex<float>> in, bool conj, std::string order)
{
    std::vector<uint32_t> out(in.size(), 0);
    for (size_t i = 0; i < in.size(); i++) {
        uint16_t re = (uint16_t)(int16_t(in[i].real() * 32768.0));
        uint16_t im = (uint16_t)(
            int16_t((conj ? -in[i].imag() : in[i].imag()) * 32768));
        if (order == "IQ")
            out[i] = (uint32_t)re << 16 | im;
        else if (order == "QI")
            out[i] = (uint32_t)im << 16 | re;
    }
    return out;
}

std::vector<std::vector<size_t>> Utils::loadSymbols(
    std::vector<std::string> frames, char sym)
{
    std::vector<std::vector<size_t>> symId;
    size_t frameSize = frames.size();
    symId.resize(frameSize);
    for (size_t f = 0; f < frameSize; f++) {
        std::string fr = frames[f];
        for (size_t g = 0; g < fr.size(); g++) {
            if (fr[g] == sym) {
                symId[f].push_back(g);
            }
        }
    }
    return symId;
}

void Utils::loadDevices(std::string filename, std::vector<std::string>& data)
{
    std::string line;
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    filename = cur_directory + "/" + filename;
    std::ifstream myfile(filename, std::ifstream::in);
    if (myfile.is_open()) {
        while (getline(myfile, line)) {
            // line.erase( std::remove (line.begin(), line.end(), ' '),
            // line.end());
            if (line.at(0) == '#')
                continue;
            data.push_back(line);
            std::cout << line << '\n';
        }
        myfile.close();
    }

    else
        printf("Unable to open device file %s\n", filename.c_str());
}

void Utils::loadData(
    const char* filename, std::vector<std::complex<int16_t>>& data, int samples)
{
    FILE* fp = fopen(filename, "r");
    data.resize(samples);
    float real, imag;
    for (int i = 0; i < samples; i++) {
        int ret = fscanf(fp, "%f %f", &real, &imag);
        if (ret < 0)
            break;
        data[i] = std::complex<int16_t>(
            int16_t(real * 32768), int16_t(imag * 32768));
    }

    fclose(fp);
}

void Utils::loadData(
    const char* filename, std::vector<unsigned>& data, int samples)
{
    FILE* fp = fopen(filename, "r");
    data.resize(samples);
    for (int i = 0; i < samples; i++) {
        int ret = fscanf(fp, "%u", &data[i]);
        if (ret < 0)
            break;
    }

    fclose(fp);
}

void Utils::loadTDDConfig(const std::string filename, std::string& jconfig)
{
    std::string line;
    std::ifstream configFile(filename);
    if (configFile.is_open()) {
        while (getline(configFile, line)) {
            jconfig += line;
        }
        configFile.close();
    }

    else
        printf("Unable to open config file %s\n", filename.c_str());
}

std::vector<std::string> Utils::split(const std::string& s, char delimiter)
{
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

void Utils::printVector(std::vector<std::complex<int16_t>>& data)
{
    for (size_t i = 0; i < data.size(); i++) {
        std::cout << real(data.at(i)) << " " << imag(data.at(i)) << std::endl;
    }
}

void Utils::writeBinaryFile(
    std::string name, size_t elem_size, size_t buffer_size, void* buff)
{
    FILE* f_handle = fopen(name.c_str(), "wb");
    fwrite(buff, elem_size, buffer_size, f_handle);
    fclose(f_handle);
}
