/**
 * @file test_modulation.cc
 * @brief Testing functions for benchmarking modulation routines
 */

#include <iostream>

#include "gettime.h"
#include "memory_manage.h"
#include "message.h"
#include "modulation.h"
#include "utils.h"

void flushCache() {
  const size_t bigger_than_cachesize = 2 * 1024;  // 100 * 1024 * 1024;
  auto* p = new long[bigger_than_cachesize];
  // When you want to "flush" cache.
  for (size_t i = 0; i < bigger_than_cachesize; i++) {
    p[i] = rand();
  }
  delete[] p;
}

static double get_time() { return ((double)clock()) / CLOCKS_PER_SEC; }

static double bench_mod_16qam(unsigned iterations, unsigned mode) {
  int* input;
  complex_float* output_mod;
  Table<complex_float> mod_table;
  InitModulationTable(mod_table, 16);
  uint8_t* output_demod_loop;
  uint8_t* output_demod_sse;
  uint8_t* output_demod_avx2;

  unsigned int num = 40;
  AllocBuffer1d(&input, num, Agora_memory::Alignment_t::kAlign32, 1);
  AllocBuffer1d(&output_mod, num, Agora_memory::Alignment_t::kAlign32, 1);
  if (mode == 0) {
    AllocBuffer1d(&output_demod_loop, num, Agora_memory::Alignment_t::kAlign32,
                  1);
    AllocBuffer1d(&output_demod_sse, num, Agora_memory::Alignment_t::kAlign32,
                  1);
    AllocBuffer1d(&output_demod_avx2, num, Agora_memory::Alignment_t::kAlign32,
                  1);
  } else {
    AllocBuffer1d(&output_demod_loop, num * 4,
                  Agora_memory::Alignment_t::kAlign32, 1);
    AllocBuffer1d(&output_demod_sse, num * 4,
                  Agora_memory::Alignment_t::kAlign32, 1);
    AllocBuffer1d(&output_demod_avx2, num * 4,
                  Agora_memory::Alignment_t::kAlign32, 1);
  }

  srand(0);
  for (unsigned i = 0; i < num; i++) input[i] = rand() % 16;

  std::printf("input: \n");
  for (unsigned i = 0; i < num; i++) {
    std::printf("%d ", input[i]);
  }
  std::printf("\n");

  double start_time = get_time();

  for (unsigned i = 0; i < iterations; i++) {
    for (unsigned j = 0; j < num; j++)
      output_mod[j] = ModSingle(input[j], mod_table);
    // std::printf("modulated: \n");
    // for (unsigned i = 0; i < num; i++) {
    //     std::printf("(%.3f, %.3f) ", output_mod[i].re, output_mod[i].im);
    // }
    // std::printf("\n");
    if (mode == 0) {
      Demod16qamHardLoop((float*)output_mod, output_demod_loop, num);
      Demod16qamHardSse((float*)output_mod, output_demod_sse, num);
      Demod16qamHardAvx2((float*)output_mod, output_demod_avx2, num);
    } else {
      Demod16qamSoftLoop((float*)output_mod, (int8_t*)output_demod_loop, num);
      Demod16qamSoftSse((float*)output_mod, (int8_t*)output_demod_sse, num);
      Demod16qamSoftAvx2((float*)output_mod, (int8_t*)output_demod_avx2, num);
    }
  }
  double end_time = get_time();

  if (mode == 1) {
    std::printf("\noutput loop: \n");
    for (unsigned i = 0; i < num * 4; i++) {
      std::printf("%i ", (int8_t)output_demod_loop[i]);
    }
    std::printf("\n");

    std::printf("output sse: \n");
    for (unsigned i = 0; i < num * 4; i++) {
      std::printf("%i ", (int8_t)output_demod_sse[i]);
    }
    std::printf("\n");

    std::printf("output avx2: \n");
    for (unsigned i = 0; i < num * 4; i++) {
      std::printf("%i ", (int8_t)output_demod_avx2[i]);
    }
    std::printf("\n");

    int num_error = 0;
    for (unsigned i = 0; i < num * 4; i++) {
      if (output_demod_sse[i] != output_demod_avx2[i]) {
        num_error++;
        // std::printf("error at %d: %i, %i\n", i,
        // (int8_t)output_demod_sse[i], (int8_t)output_demod_avx2[i]);
      }
    }
    std::printf("error rate %d/%d\n", num_error, num * 4);
  } else {
    std::printf("\noutput loop: \n");
    for (unsigned i = 0; i < num; i++) {
      std::printf("%i ", output_demod_loop[i]);
    }
    std::printf("\n");

    std::printf("output sse: \n");
    for (unsigned i = 0; i < num; i++) {
      std::printf("%i ", output_demod_sse[i]);
    }
    std::printf("\n");

    std::printf("output avx2: \n");
    for (unsigned i = 0; i < num; i++) {
      std::printf("%i ", output_demod_avx2[i]);
    }
    std::printf("\n");
  }

  return end_time - start_time;
}

static double bench_mod_64qam(unsigned iterations, unsigned mode) {
  int* input;
  complex_float* output_mod;

  Table<complex_float> mod_table;
  InitModulationTable(mod_table, 64);
  unsigned int num = 100;
  uint8_t* output_demod_loop;
  uint8_t* output_demod_sse;
  uint8_t* output_demod_avx2;

  AllocBuffer1d(&input, num, Agora_memory::Alignment_t::kAlign32, 1);
  AllocBuffer1d(&output_mod, num, Agora_memory::Alignment_t::kAlign32, 1);
  if (mode == 0) {
    Demod64qamHardSse((float*)output_mod, output_demod_sse, num);
    AllocBuffer1d(&output_demod_loop, num, Agora_memory::Alignment_t::kAlign32,
                  1);
    AllocBuffer1d(&output_demod_sse, num, Agora_memory::Alignment_t::kAlign32,
                  1);
    AllocBuffer1d(&output_demod_avx2, num, Agora_memory::Alignment_t::kAlign32,
                  1);
  } else {
    AllocBuffer1d(&output_demod_loop, num * 6,
                  Agora_memory::Alignment_t::kAlign32, 1);
    AllocBuffer1d(&output_demod_sse, num * 6,
                  Agora_memory::Alignment_t::kAlign32, 1);
    AllocBuffer1d(&output_demod_avx2, num * 6,
                  Agora_memory::Alignment_t::kAlign32, 1);
  }

  srand(0);
  for (unsigned i = 0; i < num; i++) input[i] = i % 64;
  // input[i] = rand() % 64;

  std::printf("input: \n");
  for (unsigned i = 0; i < num; i++) {
    std::printf("%d ", input[i]);
  }
  std::printf("\n");

  double start_time = get_time();
  for (unsigned i = 0; i < iterations; i++) {
    for (unsigned j = 0; j < num; j++)
      output_mod[j] = ModSingle(input[j], mod_table);

    // for (unsigned i = 0; i < num; i++) {
    //     std::printf("(%.3f, %.3f) ", output_mod[i].re, output_mod[i].im);
    // }

    if (mode == 0) {
      Demod64qamHardLoop((float*)output_mod, output_demod_loop, num);
      Demod64qamHardSse((float*)output_mod, output_demod_sse, num);
      Demod64qamHardAvx2((float*)output_mod, output_demod_avx2, num);
    } else {
      Demod64qamSoftLoop((float*)output_mod, (int8_t*)output_demod_loop, num);
      Demod64qamSoftSse((float*)output_mod, (int8_t*)output_demod_sse, num);
      Demod64qamSoftAvx2((float*)output_mod, (int8_t*)output_demod_avx2, num);
    }
  }
  double end_time = get_time();

  if (mode == 1) {
    std::printf("\noutput loop: \n");
    for (unsigned i = 0; i < num * 6; i++) {
      std::printf("%i ", (int8_t)output_demod_loop[i]);
    }
    std::printf("\n");

    std::printf("output sse: \n");
    for (unsigned i = 0; i < num * 6; i++) {
      std::printf("%i ", (int8_t)output_demod_sse[i]);
    }
    std::printf("\n");

    std::printf("output avx2: \n");
    for (unsigned i = 0; i < num * 6; i++) {
      std::printf("%i ", (int8_t)output_demod_avx2[i]);
    }
    std::printf("\n");

    int num_error = 0;
    for (unsigned i = 0; i < num * 6; i++) {
      if (output_demod_sse[i] != output_demod_avx2[i]) {
        num_error++;
        // std::printf("error at %d: %i, %i\n", i,
        // (int8_t)output_demod_sse[i], (int8_t)output_demod_avx2[i]);
      }
    }
    std::printf("error rate %d/%d\n", num_error, num * 6);

  } else {
    std::printf("\noutput loop: \n");
    for (unsigned i = 0; i < num; i++) {
      std::printf("%i ", output_demod_loop[i]);
    }
    std::printf("\n");

    std::printf("output sse: \n");
    for (unsigned i = 0; i < num; i++) {
      std::printf("%i ", output_demod_sse[i]);
    }
    std::printf("\n");

    std::printf("output avx2: \n");
    for (unsigned i = 0; i < num; i++) {
      std::printf("%i ", output_demod_avx2[i]);
    }
    std::printf("\n");
  }

  return end_time - start_time;
}

int hammingdist(uint8_t x, uint8_t y) {
  uint8_t mask = 0x80, rshift = 7, hammingdist = 0;
  while (mask) {
    // Calculate hamming dist
    hammingdist += abs(((x & mask) >> rshift) - ((y & mask) >> rshift));
    rshift--;
    mask >>= 1;
  }
  return hammingdist;
}

void printbits(uint8_t x) {
  // Print MSB first
  uint8_t mask = 0x80, rshift = 7;
  while (mask) {
    printf("%d", (x & mask) >> rshift);
    rshift--;
    mask >>= 1;
  }
  printf(" ");
}

static double bench_mod_256qam(unsigned iterations, unsigned mode) {
  Table<complex_float> mod_table;
  complex_float* output_mod;
  double start_time, total_time, elapsed_time;
  unsigned int num = 1000;
  uint8_t* input;
  uint8_t *output_demod_loop, *output_demod_sse, *output_demod_avx2;
#ifdef __AVX512F__
  uint8_t* output_demod_avx512;
#endif
  int gray_mapping[16][16];

  InitModulationTable(mod_table, 256);
  for (int i = 0; i < 256; i++) {
    int re = (int)(mod_table[0][i].re * sqrt(170) / 2 + 8);
    int im = (int)(mod_table[0][i].im * sqrt(170) / 2 + 8);
    gray_mapping[im][re] = i;
  }
  for (int i = 15; i >= 0; i--) {
    if (i == 7)
      printf(
          "-----------------------------------------------------------------"
          "------------"
          "------------------------------------------------------------------"
          "\n");
    for (int j = 0; j < 16; j++) {
      printbits(gray_mapping[i][j]);
      /*
       * Validate the hamming distance of adjacent entries
       */
      if (i > 0) {
        // Check left
        if (hammingdist(gray_mapping[i - 1][j], gray_mapping[i][j]) != 1) {
          printf("Bad west hamming dist at (%d,%d)\n", i, j);
        }
      }
      if (i < 15) {
        // Check right
        if (hammingdist(gray_mapping[i + 1][j], gray_mapping[i][j]) != 1) {
          printf("Bad east hamming dist at (%d,%d)\n", i, j);
        }
      }
      if (j > 0) {
        // Check above
        if (hammingdist(gray_mapping[i][j - 1], gray_mapping[i][j]) != 1) {
          printf("Bad north hamming dist at (%d,%d)\n", i, j);
        }
      }
      if (j < 15) {
        // Check below
        if (hammingdist(gray_mapping[i][j + 1], gray_mapping[i][j]) != 1) {
          printf("Bad south hamming dist at (%d,%d)\n", i, j);
        }
      }
      if (j == 7) {
        // Print divider
        printf("| ");
      }
    }
    printf("\n");
  }
  AllocBuffer1d(&input, num, Agora_memory::Alignment_t::kAlign64, 1);
  AllocBuffer1d(&output_mod, num, Agora_memory::Alignment_t::kAlign64, 1);
  AllocBuffer1d(&output_demod_loop, num, Agora_memory::Alignment_t::kAlign64,
                1);
  AllocBuffer1d(&output_demod_sse, num, Agora_memory::Alignment_t::kAlign64, 1);
  AllocBuffer1d(&output_demod_avx2, num, Agora_memory::Alignment_t::kAlign64,
                1);
#ifdef __AVX512F__
  AllocBuffer1d(&output_demod_avx512, num, Agora_memory::Alignment_t::kAlign64,
                1);
#endif
  // Build the input data from random bytes
  std::printf("Input: ");
  for (int i = 0; i < num; i++) {
    input[i] = rand() % 256;
    std::printf("%d ", input[i]);
  }
  std::printf("\n");

  start_time = GetTime::GetTime();
  elapsed_time = GetTime::GetTime();
  for (unsigned i = 0; i < iterations; i++) {
    for (unsigned j = 0; j < num; j++) {
      output_mod[j] = ModSingle(input[j], mod_table);
    }
    // Demodulate input
    Demod256qamHardLoop((float*)output_mod, output_demod_loop, num);
  }
  elapsed_time = GetTime::GetTime() - elapsed_time;
  std::printf("Avg Loop time: %f us\n", (elapsed_time / iterations));
  elapsed_time = GetTime::GetTime();
  for (unsigned i = 0; i < iterations; i++) {
    // Build the input data from random bytes
    for (unsigned j = 0; j < num; j++) {
      output_mod[j] = ModSingle(input[j], mod_table);
    }
    // Demodulate input
    Demod256qamHardSse((float*)output_mod, output_demod_sse, num);
  }
  elapsed_time = GetTime::GetTime() - elapsed_time;
  std::printf("Avg SSE time: %f us\n", (elapsed_time / iterations));
  elapsed_time = GetTime::GetTime();
  for (unsigned i = 0; i < iterations; i++) {
    // Build the input data from random bytes
    for (unsigned j = 0; j < num; j++) {
      output_mod[j] = ModSingle(input[j], mod_table);
    }
    // Demodulate input
    Demod256qamHardAvx2((float*)output_mod, output_demod_avx2, num);
  }
  elapsed_time = GetTime::GetTime() - elapsed_time;
  std::printf("Avg AVX2 time: %f us\n", (elapsed_time / iterations));
#ifdef __AVX512F__
  elapsed_time = GetTime::GetTime();
  for (unsigned i = 0; i < iterations; i++) {
    for (unsigned j = 0; j < num; j++) {
      output_mod[j] = ModSingle(input[j], mod_table);
    }
    // Demodulate input
    Demod256qamHardAvx512((float*)output_mod, output_demod_avx512, num);
  }
  elapsed_time = GetTime::GetTime() - elapsed_time;
  std::printf("Avg AVX512 time: %f us\n", (elapsed_time / iterations));
#endif
  total_time = GetTime::GetTime() - start_time;
  if (mode == 0) {
    // Check results.
    unsigned i;
    std::cout << "Results checking enabled\n";
    for (i = 0; i < num; i++) {
      if (input[i] != output_demod_loop[i]) {
        std::cout << "Loop Results differed at index " << i << "\n";
        std::printf("Expected %d Actual %d\n", input[i], output_demod_loop[i]);
        break;
      }
      if (input[i] != output_demod_sse[i]) {
        std::cout << "SSE Results differed at index " << i << "\n";
        std::printf("Expected %d Actual %d\n", input[i], output_demod_sse[i]);
        break;
      }
      if (input[i] != output_demod_avx2[i]) {
        std::cout << "AXV2 Results differed at index " << i << "\n";
        std::printf("Expected %d Actual %d\n", input[i], output_demod_avx2[i]);
        break;
      }
#ifdef __AVX512F__
      if (input[i] != output_demod_avx512[i]) {
        std::cout << "AXV512 Results differed at index " << i << "\n";
        std::printf("Expected %d Actual %d\n", input[i],
                    output_demod_avx512[i]);
        break;
      }
#endif
    }
    if (i != num) {
      std::cout << "Correctness check failed\n";
    } else {
      std::cout << "Correctness check passed\n Output: ";
      for (i = 0; i < num; i++) {
        std::printf("%d ", output_demod_loop[i]);
      }
      std::printf("\n");
    }
  }
  return total_time * (1000000 / iterations);
}

static void run_benchmark_16qam(unsigned iterations, unsigned mode) {
  double time = bench_mod_16qam(iterations, mode);
  std::printf("time: %.2f us per iteration\n", time / iterations);
}

static void run_benchmark_64qam(unsigned iterations, unsigned mode) {
  double time = bench_mod_64qam(iterations, mode);
  std::printf("time: %.2f us per iteration\n", time / iterations);
}

static void run_benchmark_256qam(unsigned iterations, unsigned mode) {
  double time = bench_mod_256qam(iterations, mode);
str:
  printf("time: %.2f us per iteration\n", time / iterations);
}

int main(int argc, char* argv[]) {
  if (argc != 4) {
    std::fprintf(
        stderr,
        "Usage: %s [modulation order 4/16/64/256] [mode hard(0)/soft(1)] "
        "[iterations]\n",
        argv[0]);
    return 1;
  }

  // int main_core_id = 2;
  // if(stick_this_thread_to_core(main_core_id) != 0) {
  //     std::printf("Main thread: stitch main thread to core %d failed\n",
  //     main_core_id); std::exit(0);
  // }
  // else {
  //     std::printf("Main thread: stitch main thread to core %d succeeded\n",
  //     main_core_id);
  // }

  if (argc == 4) {
    unsigned mod_order = strtoul(argv[1], NULL, 0);
    unsigned mode = strtoul(argv[2], NULL, 0);
    unsigned iterations = strtoul(argv[3], NULL, 0);
    // if (mod_order == 4)
    //     run_benchmark_qpsk(mod_order, iterations);
    if (mod_order == 16)
      run_benchmark_16qam(iterations, mode);
    else if (mod_order == 64)
      run_benchmark_64qam(iterations, mode);
    else if (mod_order == 256)
      run_benchmark_256qam(iterations, mode);
    else
      std::printf("Error: modulation order not supported!\n");
  }
}
