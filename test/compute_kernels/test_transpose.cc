/**
 * @file test_transpose.cc
 * @brief Testing functions for benchmarking transpose computations
 */
#include <immintrin.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <iostream>
#include <vector>

#include "armadillo"
#include "simd_types.h"

#define OFDM (1024)
#define BS_ANT (96)
#define K (4)
#define LOOP_NUM (1e4)

struct complex_float {
  float real;
  float imag;
};

typedef arma::cx_float COMPLEX;

int flushCache() {
  const size_t bigger_than_cachesize = 100 * 1024 * 1024;
  auto* p = new long[bigger_than_cachesize];
  // When you want to "flush" cache.
  for (int i = 0; i < bigger_than_cachesize; i++) {
    p[i] = rand();
  }
  delete p;
}

void saveData(char* filename, complex_float* ptr, int row, int col) {
  FILE* fp = std::fopen(filename, "w");
  for (int i = 0; i < row; i++) {
    for (int j = 0; j < col; j++) {
      std::fprintf(fp, "%6.5f+%6.5fi  ", ptr[i * col + j].real,
                   ptr[i * col + j].imag);
    }
    std::fprintf(fp, "\n");
  }
  std::fclose(fp);
}

int main(int argc, char** argv) {
  srand(0);
  std::printf("test\n");
  SimdAlignCxFltVector buffer;
  SimdAlignCxFltVector buffer_trans;
  buffer.resize(BS_ANT * OFDM);
  buffer_trans.resize(BS_ANT * OFDM);
  for (int i = 0; i < BS_ANT; i++) {
    for (int j = 0; j < OFDM; j++) {
      buffer[i * OFDM + j].real = (rand() % 65536) / (float)65536;
      buffer[i * OFDM + j].imag = (rand() % 65536) / (float)65536;
      // buffer[i * OFDM + j].real = 1.f;
      // buffer[i * OFDM + j].imag = 0.f;
    }
  }

  saveData("data.txt", buffer.data(), BS_ANT, OFDM);

  SimdAlignCxFltVector precoder;
  precoder.resize(K * BS_ANT);
  for (int i = 0; i < K * BS_ANT; i++) {
    precoder[i].real = (rand() % 65536) / (float)65536;
    precoder[i].imag = (rand() % 65536) / (float)65536;
    // precoder[i].real = 1.f;
    // precoder[i].imag = 0.f;
  }
  SimdAlignCxFltVector result;
  result.resize(K);

  saveData("precoder.txt", precoder.data(), K, BS_ANT);

  flushCache();
  auto begin = std::chrono::system_clock::now();
  for (int i = 0; i < LOOP_NUM; ++i) {
    // just copy
    for (int j = 0; j < BS_ANT; j++) {
      // std::memcpy(buffer_trans.data() + j * OFDM, buffer.data() + j * OFDM,
      // sizeof(complex_float) * OFDM);
      std::copy(buffer.data() + j * OFDM, buffer.data() + (j + 1) * OFDM,
                buffer_trans.data() + j * OFDM);
    }
  }
  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end - begin;
  std::printf("std::memcpy copy time %f\n", diff.count());

  flushCache();
  begin = std::chrono::system_clock::now();
  for (int i = 0; i < LOOP_NUM; ++i) {
    // just copy
    for (int c1 = 0; c1 < BS_ANT; c1++) {
      for (int c2 = 0; c2 < OFDM; c2++) {
        buffer_trans[c1 * OFDM + c2] = buffer[c1 * OFDM + c2];
      }
    }
  }
  end = std::chrono::system_clock::now();
  diff = end - begin;
  std::printf("naive copy time %f\n", diff.count());

  flushCache();
  begin = std::chrono::system_clock::now();
  for (int i = 0; i < LOOP_NUM; ++i) {
    float* src_ptr = (float*)buffer.data();
    float* tar_ptr = (float*)buffer_trans.data();
    for (int i = 0; i < BS_ANT * OFDM / 4; i++) {
      __m256 data = _mm256_load_ps(src_ptr);
      _mm256_store_ps(tar_ptr, data);
      src_ptr += 8;
      tar_ptr += 8;
    }
  }
  end = std::chrono::system_clock::now();
  diff = end - begin;
  std::printf("avx2 __m256 copy time %f\n", diff.count());

  flushCache();
  begin = std::chrono::system_clock::now();
  for (int i = 0; i < LOOP_NUM; ++i) {
    // just copy
    for (int c1 = 0; c1 < BS_ANT; c1++) {
      for (int c2 = 0; c2 < OFDM; c2++) {
        buffer_trans[c2 * BS_ANT + c1] = buffer[c1 * OFDM + c2];
      }
    }
  }
  end = std::chrono::system_clock::now();
  diff = end - begin;
  std::printf("naive trans time %f\n", diff.count());

  flushCache();
  begin = std::chrono::system_clock::now();
  for (int i = 0; i < LOOP_NUM; ++i) {
    // just copy
    for (int j = 0; j < BS_ANT; j++) {
      std::memcpy(buffer_trans.data() + j * OFDM, buffer.data() + j * OFDM,
                  sizeof(complex_float) * OFDM);
    }
    arma::cx_float* mat_ptr = (arma::cx_float*)buffer_trans.data();
    arma::cx_fmat mat_data(mat_ptr, BS_ANT, OFDM, false);
    inplace_trans(mat_data);
  }
  end = std::chrono::system_clock::now();
  diff = end - begin;
  std::printf("armadillo trans time %f\n", diff.count());

  flushCache();
  begin = std::chrono::system_clock::now();
  for (int i = 0; i < LOOP_NUM; ++i) {
    for (int c1 = 0; c1 < OFDM; c1++) {
      arma::cx_float* data_ptr = (arma::cx_float*)(&buffer[c1 * BS_ANT]);
      arma::cx_fmat mat_data(data_ptr, BS_ANT, 1, false);

      arma::cx_float* precoder_ptr = (arma::cx_float*)precoder.data();
      arma::cx_fmat mat_precoder(precoder_ptr, K, BS_ANT, false);

      arma::cx_float* result_ptr = (arma::cx_float*)result.data();
      arma::cx_fmat mat_result(result_ptr, K, 1, false);

      mat_result = mat_precoder * mat_data;
    }
  }
  end = std::chrono::system_clock::now();
  diff = end - begin;
  std::printf("only precoding time %f\n", diff.count());

  flushCache();
  begin = std::chrono::system_clock::now();
  for (int i = 0; i < LOOP_NUM; ++i) {
    // just copy
    for (int c1 = 0; c1 < BS_ANT; c1++) {
      for (int c2 = 0; c2 < OFDM; c2++) {
        buffer_trans[c2 * BS_ANT + c1] = buffer[c1 * OFDM + c2];
      }
    }
    for (int c1 = 0; c1 < OFDM; c1++) {
      arma::cx_float* data_ptr = (arma::cx_float*)(&buffer_trans[c1 * BS_ANT]);
      arma::cx_fmat mat_data(data_ptr, BS_ANT, 1, false);

      arma::cx_float* precoder_ptr = (arma::cx_float*)precoder.data();
      arma::cx_fmat mat_precoder(precoder_ptr, K, BS_ANT, false);

      arma::cx_float* result_ptr = (arma::cx_float*)result.data();
      arma::cx_fmat mat_result(result_ptr, K, 1, false);

      mat_result = mat_precoder * mat_data;
      /*
                              if(i == 0 && c1 == 0)
                              {
                                      saveData("demul_ca_0_baseline.txt",
         (complex_float*)result_ptr, K, 1);
         saveData("data_ca_0_baseline.txt", (complex_float*)data_ptr,
         BS_ANT, 1); saveData("precoder_ca_0_baseline.txt",
         (complex_float*)precoder_ptr, K, BS_ANT);
                              }
                              */
    }
  }
  end = std::chrono::system_clock::now();
  diff = end - begin;
  std::printf("naive trans and precoding time %f\n", diff.count());

  flushCache();
  begin = std::chrono::system_clock::now();
  SimdAlignCxFltVector temp_buffer;
  temp_buffer.resize(BS_ANT);
  for (int i = 0; i < LOOP_NUM; ++i) {
    for (int c1 = 0; c1 < OFDM; c1++) {
      for (int c2 = 0; c2 < BS_ANT; c2++)
        temp_buffer[c2] = buffer[c2 * OFDM + c1];

      arma::cx_float* data_ptr = (arma::cx_float*)(&temp_buffer[0]);
      arma::cx_fmat mat_data(data_ptr, BS_ANT, 1, false);

      arma::cx_float* precoder_ptr = (arma::cx_float*)precoder.data();
      arma::cx_fmat mat_precoder(precoder_ptr, K, BS_ANT, false);

      arma::cx_float* result_ptr = (arma::cx_float*)result.data();
      arma::cx_fmat mat_result(result_ptr, K, 1, false);

      mat_result = mat_precoder * mat_data;
    }
  }
  end = std::chrono::system_clock::now();
  diff = end - begin;
  std::printf("no copy and (read trans) precoding time %f\n", diff.count());

  flushCache();
  begin = std::chrono::system_clock::now();
  float* temp_buffer_ptr = (float*)(temp_buffer.data());
  for (int i = 0; i < LOOP_NUM; ++i) {
    float* src_ptr = (float*)buffer.data();
    __m256i index = _mm256_setr_epi64x(0, 1024, 2048, 3072);
    for (int c1 = 0; c1 < OFDM; c1++) {
      for (int c2 = 0; c2 < BS_ANT / 4; c2++) {
        float* base_ptr = src_ptr + c2 * 4 * 1024 * 2 + c1 * 2;
        __m256d t_data = _mm256_i64gather_pd((double*)base_ptr, index, 8);
        _mm256_store_pd((double*)(temp_buffer_ptr + c2 * 8), t_data);

        //__m256d t_data = _mm256_i64gather_pd((double*)tar_ptr, index,
        // 8); _mm256_store_pd((double*)temp_buffer_ptr, t_data);
      }

      arma::cx_float* data_ptr = (arma::cx_float*)(&temp_buffer[0]);
      arma::cx_fmat mat_data(data_ptr, BS_ANT, 1, false);

      arma::cx_float* precoder_ptr = (arma::cx_float*)precoder.data();
      arma::cx_fmat mat_precoder(precoder_ptr, K, BS_ANT, false);

      arma::cx_float* result_ptr = (arma::cx_float*)result.data();
      arma::cx_fmat mat_result(result_ptr, K, 1, false);

      mat_result = mat_precoder * mat_data;
    }
  }
  end = std::chrono::system_clock::now();
  diff = end - begin;
  std::printf("no copy and (SIMD read) precoding time %f\n", diff.count());

  flushCache();
  begin = std::chrono::system_clock::now();
  for (int i = 0; i < LOOP_NUM; ++i) {
    int BLOCK = 64;  // 8 float per row

    // save buffer_trans as 8 column blocks
    float* src_ptr = (float*)buffer.data();
    float* tar_ptr = (float*)buffer_trans.data();
    for (int c1 = 0; c1 < BS_ANT; c1++) {
      for (int c2 = 0; c2 < OFDM / BLOCK * 2; c2++) {
        for (int c3 = 0; c3 < BLOCK / 8; c3++) {
          __m256 data =
              _mm256_load_ps(src_ptr + c1 * OFDM * 2 + c2 * BLOCK + c3 * 8);
          _mm256_store_ps(tar_ptr + c2 * BS_ANT * BLOCK + BLOCK * c1 + c3 * 8,
                          data);
        }
      }
    }
  }
  end = std::chrono::system_clock::now();
  diff = end - begin;
  std::printf("block trans time %f\n", diff.count());

  flushCache();
  begin = std::chrono::system_clock::now();
  // float* temp_buffer_ptr = (float*)(temp_buffer.data());
  for (int i = 0; i < LOOP_NUM; ++i) {
    int BLOCK = 64;  // 8 float per row

    // save buffer_trans as 8 column blocks
    float* src_ptr = (float*)buffer.data();
    float* tar_ptr = (float*)buffer_trans.data();
    for (int c1 = 0; c1 < BS_ANT; c1++) {
      for (int c2 = 0; c2 < OFDM / BLOCK * 2; c2++) {
        for (int c3 = 0; c3 < BLOCK / 8; c3++) {
          __m256 data =
              _mm256_load_ps(src_ptr + c1 * OFDM * 2 + c2 * BLOCK + c3 * 8);
          _mm256_store_ps(tar_ptr + c2 * BS_ANT * BLOCK + BLOCK * c1 + c3 * 8,
                          data);
        }
      }
    }

    __m256i index =
        _mm256_setr_epi64x(0, BLOCK / 2, BLOCK / 2 * 2, BLOCK / 2 * 3);
    for (int c1 = 0; c1 < OFDM; c1++) {
      for (int c2 = 0; c2 < BS_ANT / 4; c2++) {
        int c1_base = c1 * 2 / BLOCK;
        int c1_offset = c1 * 2 % BLOCK;
        float* base_ptr =
            tar_ptr + c1_base * BLOCK * BS_ANT + c1_offset + c2 * BLOCK * 4;
        __m256d t_data = _mm256_i64gather_pd((double*)base_ptr, index, 8);
        _mm256_store_pd((double*)(temp_buffer_ptr + c2 * 8), t_data);

        //__m256d t_data = _mm256_i64gather_pd((double*)tar_ptr, index,
        // 8); _mm256_store_pd((double*)temp_buffer_ptr, t_data);
      }

      arma::cx_float* data_ptr = (arma::cx_float*)(&temp_buffer[0]);
      arma::cx_fmat mat_data(data_ptr, BS_ANT, 1, false);

      arma::cx_float* precoder_ptr = (arma::cx_float*)precoder.data();
      arma::cx_fmat mat_precoder(precoder_ptr, K, BS_ANT, false);

      arma::cx_float* result_ptr = (arma::cx_float*)result.data();
      arma::cx_fmat mat_result(result_ptr, K, 1, false);

      mat_result = mat_precoder * mat_data;
      /*
                              if(i == 0 && c1 == 0)
                              {
                                      saveData("demul_ca_0_SIMD.txt",
         (complex_float*)result_ptr, K, 1); saveData("data_ca_0_SIMD.txt",
         (complex_float*)data_ptr, BS_ANT, 1);
                                      saveData("precoder_ca_0_SIMD.txt",
         (complex_float*)precoder_ptr, K, BS_ANT);
                              }
      */
    }
  }
  end = std::chrono::system_clock::now();
  diff = end - begin;
  std::printf("no copy and (SIMD read trans) precoding time %f\n",
              diff.count());
  saveData("data_trans_block.txt", buffer_trans.data(), OFDM * BS_ANT / 4, 4);
}
