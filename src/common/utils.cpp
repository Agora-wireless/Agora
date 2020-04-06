/*

 utility functions for file and text processing.

---------------------------------------------------------------------
 Copyright (c) 2018-2019, Rice University
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
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
    ThreadType thread_type, int core_offset, int thread_id)
{
#ifdef ENABLE_CPU_ATTACH
    int actual_core_id = core_offset + thread_id;
    int num_cores = sysconf(_SC_NPROCESSORS_ONLN);

    /* Reserve core 0 for kernel threads */
    if (actual_core_id >= num_cores) {
        actual_core_id = (actual_core_id % num_cores) + 1;
    }

    if (pin_to_core(actual_core_id) != 0) {
        printf("%s thread %d: failed to pin to core %d\n",
            thread_type_str(thread_type).c_str(), thread_id, actual_core_id);
        exit(0);
    } else {
        printf("%s thread %d: pinned to core %d\n",
            thread_type_str(thread_type).c_str(), thread_id, actual_core_id);
    }
#endif
}

/**
 * Use SIMD to vectorize data type conversion from short to float
 * reference:
 * https://stackoverflow.com/questions/50597764/convert-signed-short-to-float-in-c-simd
 * 0x4380'8000
 */
void Utils::cvtShortToFloatSIMD(short* in_buf, float*& out_buf, size_t length)
{
#ifdef __AVX512F__
    const __m512 magic = _mm512_set1_ps(float((1 << 23) + (1 << 15)) / 32768.f);
    const __m512i magic_i = _mm512_castps_si512(magic);
    for (size_t i = 0; i < length; i += 16) {
        /* get input */
        __m256i val = _mm256_load_si256((__m256i*)(in_buf + i)); // port 2,3
        /* interleave with 0x0000 */
        __m512i val_unpacked = _mm512_cvtepu16_epi32(val); // port 5
        /* convert by xor-ing and subtracting magic value:
         * VPXOR avoids port5 bottlenecks on Intel CPUs before SKL */
        __m512i val_f_int
            = _mm512_xor_si512(val_unpacked, magic_i); // port 0,1,5
        __m512 val_f = _mm512_castsi512_ps(val_f_int); // no instruction
        __m512 converted = _mm512_sub_ps(val_f, magic); // port 1,5 ?
        _mm512_store_ps(out_buf + i, converted); // port 2,3,4,7
    }
#else
    const __m256 magic = _mm256_set1_ps(float((1 << 23) + (1 << 15)) / 32768.f);
    const __m256i magic_i = _mm256_castps_si256(magic);
    for (size_t i = 0; i < length; i += 16) {
        /* get input */
        __m128i val = _mm_load_si128((__m128i*)(in_buf + i)); // port 2,3

        __m128i val1 = _mm_load_si128((__m128i*)(in_buf + i + 8));
        /* interleave with 0x0000 */
        __m256i val_unpacked = _mm256_cvtepu16_epi32(val); // port 5
        /* convert by xor-ing and subtracting magic value:
         * VPXOR avoids port5 bottlenecks on Intel CPUs before SKL */
        __m256i val_f_int
            = _mm256_xor_si256(val_unpacked, magic_i); // port 0,1,5
        __m256 val_f = _mm256_castsi256_ps(val_f_int); // no instruction
        __m256 converted = _mm256_sub_ps(val_f, magic); // port 1,5 ?
        _mm256_store_ps(out_buf + i, converted); // port 2,3,4,7

        __m256i val_unpacked1 = _mm256_cvtepu16_epi32(val1); // port 5
        __m256i val_f_int1
            = _mm256_xor_si256(val_unpacked1, magic_i); // port 0,1,5
        __m256 val_f1 = _mm256_castsi256_ps(val_f_int1); // no instruction
        __m256 converted1 = _mm256_sub_ps(val_f1, magic); // port 1,5 ?
        _mm256_store_ps(out_buf + i + 8, converted1); // port 2,3,4,7
    }
#endif
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
