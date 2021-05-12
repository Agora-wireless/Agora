#include <random>

#include <gtest/gtest.h>

#include "buffer.h"
#include "gettime.h"
#include "memory_manage.h"
#include "modulation.h"

#define NUM_SYMBOLS 1000   // number of symbols to modulate and demodulate
#define NUM_ITERATIONS 50 // number of iterations to run tests

/**
 * Adds additive white gaussian noise to the supplied signal
 * @param signal: input signal
 * @param output: output signal
 * @param len: length of input and output signal (number of symbols)
 * @param snr: desired SNR in dB
 */
static void apply_awgn(complex_float *signal, complex_float *output, int len, float snr) {
    float gamma, power, N0;
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
    N0 = power / gamma; // Noise spectral density
    for (i = 0; i < len; i++) {
      output[i].re = signal[i].re + sqrtf(N0/2) * distribution(generator);
      output[i].im = signal[i].im + sqrtf(N0/2) * distribution(generator);
    }
}

/**
 * Runs 256 QAM soft demodulation across several SNR values,
 * using the provided function for demodulation.
 * Provided to simplify testing.
 * @param demod_func: Function to use for demodulation
 * @param func_desc: string describing function
 */
static void run_256QAM_soft_demod(void (*demod_func)(const float*, int8_t*, int),
                                  const char *func_desc) {
  uint8_t *input_symbols, *output_symbols;
  complex_float *channel_input;
  Table<complex_float> mod_table;
  complex_float *channel_output;
  int8_t *output_demod_loop;
  int shift_offset = 7;
  unsigned int i, j, snr_idx;
  unsigned int num = NUM_SYMBOLS;
  double runtime, start_time;
  float SNR_vals[] = {10.0, 25.0, 50.0};
  float SNR, err_rate;

  // Allocate storage buffers
  AllocBuffer1d(&input_symbols, num, Agora_memory::Alignment_t::kAlign32, 1);
  AllocBuffer1d(&channel_input, num, Agora_memory::Alignment_t::kAlign32, 1);
  AllocBuffer1d(&channel_output, num, Agora_memory::Alignment_t::kAlign32, 1);
  AllocBuffer1d(&output_demod_loop, num * 8,
                Agora_memory::Alignment_t::kAlign32, 1);
  AllocBuffer1d(&output_symbols, num, Agora_memory::Alignment_t::kAlign32, 1);
  InitModulationTable(mod_table, 256);
  srand(0);
  runtime = 0.0;
  /*
   * Test demodulation at 3 SNR levels:
   * 10 db
   * 25 db
   * 50 db
   */
  for (snr_idx = 0; snr_idx < sizeof(SNR_vals) / sizeof(float); snr_idx++) {
    SNR = SNR_vals[snr_idx];
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
      apply_awgn(channel_input, channel_output, num, SNR);
      // Demodulate Symbols
      start_time = GetTime::GetTimeUs();
      demod_func((float*)channel_output, output_demod_loop, num);
      runtime += (GetTime::GetTimeUs() - start_time);
      // Decode Symbols
      for (j = 0; j < num * 8; j++) {
        /*
        * If llr value is negative, bit will be zero. 
        * If it is positive, bit will be 1
        */
        if (output_demod_loop[j] > 0) {
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
    std::printf("256 QAM soft demod of %i symbols completed with average "
      "runtime of %f us over %i iterations\n", num,
      runtime / NUM_ITERATIONS, NUM_ITERATIONS);
    err_rate = (err_rate * 100) / (NUM_SYMBOLS * NUM_ITERATIONS);
    std::printf("Soft Demod Error Rate for 256 QAM was %.2f%% at %f db SNR\n",
      err_rate, SNR);
  }
  /*
   * For the last SNR, assert that the error rate is zero. Although a
   * 100 db SNR does not guarantee no errors, they should be less likely
   */
  if (err_rate > FLT_MIN) {
    std::fprintf(stderr, 
      "Highest SNR error rate was nonzero, raise SNR or fix implementation\n");
    ASSERT_EQ(err_rate, 0.0);
  }
  std::printf("Function utilized was %s\n", func_desc);
}

TEST(TestDemod256QAM, SoftLoop) {
  run_256QAM_soft_demod(Demod256qamSoftLoop, "Demod256qamSoftLoop"); 
}

TEST(TestDemod256QAM, SoftSSE) {
  run_256QAM_soft_demod(Demod256qamSoftSse, "Demod256qamSoftSse");
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}