/**
 * @file encoder_test.cc
 * @brief Test functions for the ldpc encoding routines
 */
#include "encoder.h"

#include <algorithm>
#include <fstream>
#include <vector>

#include "../common/utils_ldpc.h"
#include "gcc_phy_ldpc_encoder_5gnr_internal.h"

static constexpr size_t kNumCodeBlocks = 1;

char* read_binfile(std::string filename, int buffer_size) {
  std::ifstream infile;
  infile.open(filename, std::ios::binary | std::ios::in);

  if (!infile.is_open()) {
    std::fprintf(stderr, "Failed to open file %s\n", filename.c_str());
    throw std::runtime_error("Failed to open file");
  }

  auto* x = new char[buffer_size]();
  infile.read((char*)x, buffer_size * sizeof(char));
  infile.close();
  return x;
}

void run_test(size_t base_graph, size_t zc) {
  const std::string bg_string = base_graph == 1 ? "BG1" : "BG2";
  const std::string zc_string = std::string("Zc") + std::to_string(zc);
  const std::string input_filename =
      std::string("test_vectors/input_") + bg_string + "_" + zc_string + ".bin";
  const std::string reference_filename = std::string("test_vectors/output_") +
                                         bg_string + "_" + zc_string + ".bin";

  int8_t* input[kNumCodeBlocks];
  int8_t* parity[kNumCodeBlocks];
  int8_t* encoded[kNumCodeBlocks];
  int8_t* parity_reference[kNumCodeBlocks];
  for (size_t n = 0; n < kNumCodeBlocks; n++) {
    input[n] = (int8_t*)read_binfile(input_filename,
                                     LdpcEncodingInputBufSize(base_graph, zc));
    parity[n] = new int8_t[LdpcEncodingParityBufSize(base_graph, zc)]();
    encoded[n] = new int8_t[LdpcEncodingEncodedBufSize(base_graph, zc)]();
    parity_reference[n] = (int8_t*)read_binfile(
        reference_filename, LdpcEncodingParityBufSize(base_graph, zc));

    LdpcEncodeHelper(base_graph, zc, LdpcMaxNumRows(base_graph), encoded[n],
                     parity[n], input[n]);
  }

  for (size_t n = 0; n < kNumCodeBlocks; n++) {
    if (std::memcmp(parity[n], parity_reference[n],
                    BitsToBytes(LdpcMaxNumParityBits(base_graph, zc))) != 0) {
      std::fprintf(stderr, "Mismatch for Zc = %zu, base graph = %zu\n", zc,
                   base_graph);
    } else {
      std::printf("Passed for Zc = %zu, base graph = %zu\n", zc, base_graph);
    }
  }

  for (size_t n = 0; n < kNumCodeBlocks; n++) {
    delete[] input[n];
    delete[] parity[n];
    delete[] encoded[n];
    delete[] parity_reference[n];
  }
}

int main() {
  // All possible expansion factors Zc in 5G NR
  std::vector<size_t> zc_all_vec = {
      2,   4,   8,   16, 32, 64,  128, 256, 3,   6,   12,  24, 48,
      96,  192, 384, 5,  10, 20,  40,  80,  160, 320, 7,   14, 28,
      56,  112, 224, 9,  18, 36,  72,  144, 288, 11,  22,  44, 88,
      176, 352, 13,  26, 52, 104, 208, 15,  30,  60,  120, 240};
  std::sort(zc_all_vec.begin(), zc_all_vec.end());

  // For some expansion factors, we don't have input and reference files yet
  const std::vector<size_t> zc_nofiles_vec = {2, 3, 4, 5, 6, 9, 13};

  for (const size_t& zc : zc_all_vec) {
    if (zc < LdpcGetMinZc() || zc > LdpcGetMaxZc()) {
      std::fprintf(stderr, "Zc value %zu not supported. Skipping.\n", zc);
      continue;
    }

    const bool no_files =
        std::find(std::begin(zc_nofiles_vec), std::end(zc_nofiles_vec), zc) !=
        std::end(zc_nofiles_vec);

    if (!no_files) {
      std::printf("Running for zc = %zu\n", zc);
      run_test(1 /* base graph */, zc);
      run_test(2 /* base graph */, zc);
    }
  }
}