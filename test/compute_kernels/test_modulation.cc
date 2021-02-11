#include "buffer.h"
#include "gettime.h"
#include "memory_manage.h"
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

static double bench_mod_16qam(unsigned iterations, unsigned mode) {
  int* input;
  complex_float* output_mod;
  Table<complex_float> mod_table;
  init_modulation_table(mod_table, 16);
  uint8_t* output_demod_loop;
  uint8_t* output_demod_sse;
  uint8_t* output_demod_avx2;

  unsigned int num = 40;
  alloc_buffer_1d(&input, num, 32, 1);
  alloc_buffer_1d(&output_mod, num, 32, 1);
  if (mode == 0) {
    alloc_buffer_1d(&output_demod_loop, num, 32, 1);
    alloc_buffer_1d(&output_demod_sse, num, 32, 1);
    alloc_buffer_1d(&output_demod_avx2, num, 32, 1);
  } else {
    alloc_buffer_1d(&output_demod_loop, num * 4, 32, 1);
    alloc_buffer_1d(&output_demod_sse, num * 4, 32, 1);
    alloc_buffer_1d(&output_demod_avx2, num * 4, 32, 1);
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
      output_mod[j] = mod_single(input[j], mod_table);
    // std::printf("modulated: \n");
    // for (unsigned i = 0; i < num; i++) {
    //     std::printf("(%.3f, %.3f) ", output_mod[i].re, output_mod[i].im);
    // }
    // std::printf("\n");
    if (mode == 0) {
      demod_16qam_hard_loop((float*)output_mod, output_demod_loop, num);
      demod_16qam_hard_sse((float*)output_mod, output_demod_sse, num);
      demod_16qam_hard_avx2((float*)output_mod, output_demod_avx2, num);
    } else {
      demod_16qam_soft_loop((float*)output_mod, (int8_t*)output_demod_loop,
                            num);
      demod_16qam_soft_sse((float*)output_mod, (int8_t*)output_demod_sse, num);
      demod_16qam_soft_avx2((float*)output_mod, (int8_t*)output_demod_avx2,
                            num);
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
  init_modulation_table(mod_table, 64);
  unsigned int num = 100;
  uint8_t* output_demod_loop;
  uint8_t* output_demod_sse;
  uint8_t* output_demod_avx2;

  alloc_buffer_1d(&input, num, 32, 1);
  alloc_buffer_1d(&output_mod, num, 32, 1);
  if (mode == 0) {
    alloc_buffer_1d(&output_demod_loop, num, 32, 1);
    alloc_buffer_1d(&output_demod_sse, num, 32, 1);
    alloc_buffer_1d(&output_demod_avx2, num, 32, 1);
  } else {
    alloc_buffer_1d(&output_demod_loop, num * 6, 32, 1);
    alloc_buffer_1d(&output_demod_sse, num * 6, 32, 1);
    alloc_buffer_1d(&output_demod_avx2, num * 6, 32, 1);
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
      output_mod[j] = mod_single(input[j], mod_table);

    // for (unsigned i = 0; i < num; i++) {
    //     std::printf("(%.3f, %.3f) ", output_mod[i].re, output_mod[i].im);
    // }

    if (mode == 0) {
      demod_64qam_hard_loop((float*)output_mod, output_demod_loop, num);
      demod_64qam_hard_sse((float*)output_mod, output_demod_sse, num);
      demod_64qam_hard_avx2((float*)output_mod, output_demod_avx2, num);
    } else {
      demod_64qam_soft_loop((float*)output_mod, (int8_t*)output_demod_loop,
                            num);
      demod_64qam_soft_sse((float*)output_mod, (int8_t*)output_demod_sse, num);
      demod_64qam_soft_avx2((float*)output_mod, (int8_t*)output_demod_avx2,
                            num);
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

static void run_benchmark_16qam(unsigned iterations, unsigned mode) {
  double time = bench_mod_16qam(iterations, mode);
  std::printf("time: %.2f us per iteration\n", time / iterations);
}

static void run_benchmark_64qam(unsigned iterations, unsigned mode) {
  double time = bench_mod_64qam(iterations, mode);
  std::printf("time: %.2f us per iteration\n", time / iterations);
}

int main(int argc, char* argv[]) {
  if (argc != 4) {
    std::fprintf(stderr,
                 "Usage: %s [modulation order 4/16/64] [mode hard(0)/soft(1)] "
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
    else
      std::printf("Error: modulation order not supported!\n");
  }
}