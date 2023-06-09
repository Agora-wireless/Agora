#include <gtest/gtest.h>

#include <random>

#include "gettime.h"
#include "memory_manage.h"
#include "message.h"
#include "modulation.h"

#define NUM_SYMBOLS 1000   // number of symbols to modulate and demodulate
#define NUM_ITERATIONS 50  // number of iterations to run tests

/**
 * Adds additive white gaussian noise to the supplied signal
 * @param signal: input signal
 * @param output: output signal
 * @param len: length of input and output signal (number of symbols)
 * @param snr: desired SNR in dB
 */
static void ApplyAwgn(complex_float *signal, complex_float *output, int len,
                      float snr) {
  float gamma;
  float power;
  float n0;
  int i;
  std::default_random_engine generator;
  // Normal distribution
  std::normal_distribution<float> distribution(0.0, 1.0);
  // Convert SNR to linear scale
  gamma = powf(10, (snr / 10.0));
  // Calculate power in signal
  power = 0.0;
  for (i = 0; i < len; i++) {
    power += (signal[i].re * signal[i].re) + (signal[i].im * signal[i].im);
  }
  power = power / ((float)len);
  n0 = power / gamma;  // Noise spectral density
  for (i = 0; i < len; i++) {
    output[i].re = signal[i].re + sqrtf(n0 / 2) * distribution(generator);
    output[i].im = signal[i].im + sqrtf(n0 / 2) * distribution(generator);
  }
}

/**
 * Runs 256 QAM soft demodulation across several SNR values,
 * using the provided function for demodulation.
 * Provided to simplify testing.
 * @param demod_func: Function to use for demodulation
 * @param func_desc: string describing function
 */
static void Run256QamSoftDemod(void (*demod_func)(const float *, int8_t *, int),
                               const char *func_desc) {
  uint8_t *input_symbols;
  uint8_t *output_symbols;
  complex_float *channel_input;
  Table<complex_float> mod_table;
  complex_float *channel_output;
  int8_t *output_demod;
  int shift_offset = 7;
  unsigned int i;
  unsigned int j;
  unsigned int snr_idx;
  unsigned int num = NUM_SYMBOLS;
  double runtime;
  double start_time;
  float snr_vals[] = {10.0, 25.0, 50.0, 100.0};
  float snr;
  float err_rate;

  // Allocate storage buffers
  AllocBuffer1d(&input_symbols, num, Agora_memory::Alignment_t::kAlign64, 1);
  AllocBuffer1d(&channel_input, num, Agora_memory::Alignment_t::kAlign64, 1);
  AllocBuffer1d(&channel_output, num, Agora_memory::Alignment_t::kAlign64, 1);
  AllocBuffer1d(&output_demod, num * 8, Agora_memory::Alignment_t::kAlign64, 1);
  AllocBuffer1d(&output_symbols, num, Agora_memory::Alignment_t::kAlign64, 1);
  InitModulationTable(mod_table, 256);
  srand(0);
  runtime = 0.0;
  /*
   * Test demodulation at 3 SNR levels:
   * 10 db
   * 25 db
   * 50 db
   */
  for (snr_idx = 0; snr_idx < sizeof(snr_vals) / sizeof(float); snr_idx++) {
    snr = snr_vals[snr_idx];
    err_rate = 0.0;
    for (i = 0; i < NUM_ITERATIONS; i++) {
      for (j = 0; j < num; j++) {
        input_symbols[j] = rand() % 256;
      }
      // Modulate symbols
      for (j = 0; j < num; j++) {
        channel_input[j] = ModSingle(input_symbols[j], mod_table);
      }
      // Add noise to symbols
      ApplyAwgn(channel_input, channel_output, num, snr);
      // Demodulate Symbols
      start_time = GetTime::GetTimeUs();
      demod_func((float *)channel_output, output_demod, num);
      runtime += (GetTime::GetTimeUs() - start_time);
      // Decode Symbols
      for (j = 0; j < num * 8; j++) {
        /*
         * If llr value is negative, bit will be zero.
         * If it is positive, bit will be 1
         */
        if (output_demod[j] > 0) {
          output_symbols[j / 8] |= 0x1 << shift_offset;
        } else {
          output_symbols[j / 8] &= ~(0x1 << shift_offset);
        }
        shift_offset--;
        if (shift_offset < 0) {
          // Wrap shift offset back to 7
          shift_offset = 7;
        }
      }
      // Check error rate
      for (j = 0; j < num; j++) {
        if (output_symbols[j] != input_symbols[j]) {
          // Error was encountered. Raise error rate.
          err_rate += 1.0;
        }
      }
    }
    std::printf(
        "256 QAM soft demod of %i symbols completed with average "
        "runtime of %f us over %i iterations\n",
        num, runtime / NUM_ITERATIONS, NUM_ITERATIONS);
    err_rate = (err_rate * 100) / (NUM_SYMBOLS * NUM_ITERATIONS);
    std::printf("Soft Demod Error Rate for 256 QAM was %.2f%% at %f db SNR\n",
                err_rate, snr);
  }
  /*
   * For the last SNR, assert that the error rate is zero. Although a
   * 100 db SNR does not guarantee no errors, they should be less likely
   */
  if (err_rate > FLT_MIN) {
    std::fprintf(stderr,
                 "Highest SNR error rate was nonzero, raise SNR or fix "
                 "implementation\n");
    ASSERT_EQ(err_rate, 0.0);
  }
  std::printf("Function utilized was %s\n", func_desc);
}

TEST(TestDemod256QAM, SoftLoop) {
  Run256QamSoftDemod(Demod256qamSoftLoop, "Demod256qamSoftLoop");
}

TEST(TestDemod256QAM, SoftSSE) {
  Run256QamSoftDemod(Demod256qamSoftSse, "Demod256qamSoftSse");
}

TEST(TestDemod256QAM, SoftAVX2) {
  Run256QamSoftDemod(Demod256qamSoftAvx2, "Demod256qamSoftAvx2");
}

#ifdef __AVX512F__
TEST(TestDemod256QAM, SoftAVX512) {
  Run256QamSoftDemod(Demod256qamSoftAvx512, "Demod256qamSoftAvx512");
}
#endif

/**
 * Unlike the rest of the testing suite, this test verifies that
 * all AVX implementations of 256 QAM demodulation produce the EXACT
 * same LLR results. It treats the SSE implementation as a ground truth.
 *
 * Note: the SSE implementation was verified against the loop one during
 * testing. The loop implementation rounds floats differently, resulting in
 * a slightly different LLR. This is why the SSE implementation is used as a
 * ground truth.
 *
 */
TEST(TestDemod256QAM, VerifyCorrectness) {
  uint8_t *input_symbols;
  complex_float *channel_input;
  Table<complex_float> mod_table;
  int8_t *output_demod_truth;
  int8_t *output_demod_check;
  unsigned int i;
  unsigned int num = NUM_SYMBOLS;

  // Allocate storage buffers
  AllocBuffer1d(&input_symbols, num, Agora_memory::Alignment_t::kAlign64, 1);
  AllocBuffer1d(&channel_input, num, Agora_memory::Alignment_t::kAlign64, 1);
  AllocBuffer1d(&output_demod_truth, num * 8,
                Agora_memory::Alignment_t::kAlign64, 1);
  AllocBuffer1d(&output_demod_check, num * 8,
                Agora_memory::Alignment_t::kAlign64, 1);
  InitModulationTable(mod_table, 256);
  srand(0);
  for (i = 0; i < num; i++) {
    input_symbols[i] = rand() % 256;
  }
  // Modulate symbols
  for (i = 0; i < num; i++) {
    channel_input[i] = ModSingle(input_symbols[i], mod_table);
  }
  // Generate ground truth
  Demod256qamSoftSse((float *)channel_input, output_demod_truth, num);
  // Test AVX2 implementation
  Demod256qamSoftAvx2((float *)channel_input, output_demod_check, num);
  ASSERT_EQ(memcmp(output_demod_check, output_demod_truth, num * 8), 0);

#ifdef __AVX512F__
  // Test AVX512 implementation
  Demod256qamSoftAvx512((float *)channel_input, output_demod_check, num);
  ASSERT_EQ(memcmp(output_demod_check, output_demod_truth, num * 8), 0);
#endif
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
