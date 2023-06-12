/**
 * @file test_ldpc.cc
 *
 * @brief Accuracy and performance test for LDPC. The encoder is Agora's
 * avx2enc - unlike FlexRAN's encoder, avx2enc works with AVX2 (i.e., unlike
 * FlexRAN's encoder, avx2enc does not require AVX-512). The decoder is
 * FlexRAN's decoder, which supports AVX2.
 */

#include <algorithm>
#include <bitset>
#include <fstream>
#include <vector>

#include "encoder.h"
#include "gettime.h"
#include "memory_manage.h"
#include "phy_ldpc_decoder_5gnr.h"
#include "symbols.h"
#include "utils_ldpc.h"

static constexpr size_t kNumCodeBlocks = 2;
static constexpr size_t kBaseGraph = 1;
static constexpr bool kEnableEarlyTermination = false;
static constexpr size_t kNumFillerBits = 0;
static constexpr size_t kMaxDecoderIters = 8;
static constexpr size_t kK5GnrNumPunctured = 2;
static constexpr size_t kNumRows = 46;

int main() {
  double freq_ghz = GetTime::MeasureRdtscFreq();
  std::printf("Spinning for one second for Turbo Boost\n");
  GetTime::NanoSleep(1000 * 1000 * 1000, freq_ghz);
  int8_t* input[kNumCodeBlocks];
  int8_t* parity[kNumCodeBlocks];
  int8_t* encoded[kNumCodeBlocks];
  uint8_t* decoded[kNumCodeBlocks];

  std::printf("Code rate: %.3f (nRows = %zu)\n", 22.f / (20 + kNumRows),
              kNumRows);

  std::vector<size_t> zc_vec = {
      2,   4,   8,   16, 32, 64,  128, 256, 3,   6,   12,  24, 48,
      96,  192, 384, 5,  10, 20,  40,  80,  160, 320, 7,   14, 28,
      56,  112, 224, 9,  18, 36,  72,  144, 288, 11,  22,  44, 88,
      176, 352, 13,  26, 52, 104, 208, 15,  30,  60,  120, 240};
  std::sort(zc_vec.begin(), zc_vec.end());
  for (const size_t& zc : zc_vec) {
    if (zc < LdpcGetMinZc() || zc > LdpcGetMaxZc()) {
      std::fprintf(stderr, "Zc value %zu not supported. Skipping.\n", zc);
      continue;
    }
    const size_t num_input_bits = LdpcNumInputBits(kBaseGraph, zc);
    const size_t num_encoded_bits =
        LdpcNumEncodedBits(kBaseGraph, zc, kNumRows);

    for (size_t i = 0; i < kNumCodeBlocks; i++) {
      input[i] = new int8_t[LdpcEncodingInputBufSize(kBaseGraph, zc)];
      parity[i] = new int8_t[LdpcEncodingParityBufSize(kBaseGraph, zc)];
      encoded[i] = new int8_t[LdpcEncodingEncodedBufSize(kBaseGraph, zc)];
      decoded[i] = new uint8_t[LdpcEncodingEncodedBufSize(kBaseGraph, zc)];
    }

    // Randomly generate input
    srand(time(nullptr));
    for (auto& n : input) {
      for (size_t i = 0; i < BitsToBytes(num_input_bits); i++) {
        n[i] = static_cast<int8_t>(rand());
      }
    }

    const size_t encoding_start_tsc = GetTime::Rdtsc();
    for (size_t n = 0; n < kNumCodeBlocks; n++) {
      LdpcEncodeHelper(kBaseGraph, zc, kNumRows, encoded[n], parity[n],
                       input[n]);
    }

    const double encoding_us =
        GetTime::CyclesToUs(GetTime::Rdtsc() - encoding_start_tsc, freq_ghz);

    // For decoding, generate log-likelihood ratios, one byte per input bit
    int8_t* llrs[kNumCodeBlocks];
    for (size_t n = 0; n < kNumCodeBlocks; n++) {
      llrs[n] = static_cast<int8_t*>(Agora_memory::PaddedAlignedAlloc(
          Agora_memory::Alignment_t::kAlign32, num_encoded_bits));
      for (size_t i = 0; i < num_encoded_bits; i++) {
        uint8_t bit_i = (encoded[n][i / 8] >> (i % 8)) & 1;
        llrs[n][i] = (bit_i == 1 ? -127 : 127);
      }
    }

    // Decoder setup
    struct bblib_ldpc_decoder_5gnr_request ldpc_decoder_5gnr_request = {};
    struct bblib_ldpc_decoder_5gnr_response ldpc_decoder_5gnr_response = {};
    ldpc_decoder_5gnr_request.numChannelLlrs = num_encoded_bits;
    ldpc_decoder_5gnr_request.numFillerBits = kNumFillerBits;
    ldpc_decoder_5gnr_request.maxIterations = kMaxDecoderIters;
    ldpc_decoder_5gnr_request.enableEarlyTermination = kEnableEarlyTermination;
    ldpc_decoder_5gnr_request.Zc = zc;
    ldpc_decoder_5gnr_request.baseGraph = kBaseGraph;
    ldpc_decoder_5gnr_request.nRows = kNumRows;

    const size_t buffer_len = 1024 * 1024;
    const size_t num_msg_bits = num_input_bits - kNumFillerBits;
    ldpc_decoder_5gnr_response.numMsgBits = num_msg_bits;
    ldpc_decoder_5gnr_response.varNodes =
        static_cast<int16_t*>(Agora_memory::PaddedAlignedAlloc(
            Agora_memory::Alignment_t::kAlign32, buffer_len * sizeof(int16_t)));

    // Decoding
    const size_t decoding_start_tsc = GetTime::Rdtsc();
    for (size_t n = 0; n < kNumCodeBlocks; n++) {
      ldpc_decoder_5gnr_request.varNodes = llrs[n];
      ldpc_decoder_5gnr_response.compactedMessageBytes = decoded[n];
      bblib_ldpc_decoder_5gnr(&ldpc_decoder_5gnr_request,
                              &ldpc_decoder_5gnr_response);
    }

    const double decoding_us =
        GetTime::CyclesToUs(GetTime::Rdtsc() - decoding_start_tsc, freq_ghz);

    // Check for errors
    size_t err_cnt = 0;
    for (size_t n = 0; n < kNumCodeBlocks; n++) {
      auto* input_buffer = reinterpret_cast<uint8_t*>(input[n]);
      uint8_t* output_buffer = decoded[n];
      for (size_t i = 0; i < BitsToBytes(num_input_bits); i++) {
        // std::printf("input: %i, output: %i\n", input_buffer[i],
        // output_buffer[i]);
        uint8_t error = input_buffer[i] ^ output_buffer[i];
        for (size_t j = 0; j < 8; j++) {
          if (i * 8 + j >= num_input_bits) {
            continue;  // Don't compare beyond end of input bits
          }
          err_cnt += error & 1;
          error >>= 1;
        }
      }
    }

    std::printf(
        "Zc = %zu, {encoding, decoding}: {%.2f, %.2f} Mbps, {%.2f, "
        "%.2f} us per code block. Bit errors = %zu, BER = %.3f\n",
        zc, num_input_bits * kNumCodeBlocks / encoding_us,
        num_input_bits * kNumCodeBlocks / decoding_us,
        encoding_us / kNumCodeBlocks, decoding_us / kNumCodeBlocks, err_cnt,
        err_cnt * 1.0 / (kNumCodeBlocks * num_input_bits));

    for (size_t i = 0; i < kNumCodeBlocks; i++) {
      delete[] input[i];
      delete[] parity[i];
      delete[] encoded[i];
      delete[] decoded[i];
      std::free(llrs[i]);
    }
    std::free(ldpc_decoder_5gnr_response.varNodes);
  }

  return 0;
}
