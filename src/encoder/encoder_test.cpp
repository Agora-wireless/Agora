#include "../common/utils_ldpc.hpp"
#include "gcc_phy_ldpc_encoder_5gnr_internal.h"
#include <algorithm>
#include <fstream>
#include <vector>

static constexpr size_t kNumCodeBlocks = 1;

char* read_binfile(std::string filename, int buffer_size)
{
    std::ifstream infile;
    infile.open(filename, std::ios::binary | std::ios::in);

    if (!infile.is_open()) {
        fprintf(stderr, "Failed to open file %s\n", filename.c_str());
        exit(-1);
    }

    auto* x = new char[buffer_size]();
    infile.read((char*)x, buffer_size * sizeof(char));
    infile.close();
    return x;
}

void run_test(size_t base_graph, size_t zc)
{
    const std::string bg_string = base_graph == 1 ? "BG1" : "BG2";
    const std::string zc_string = std::string("Zc") + std::to_string(zc);
    const std::string input_filename = std::string("test_vectors/input_")
        + bg_string + "_" + zc_string + ".bin";
    const std::string reference_filename = std::string("test_vectors/output_")
        + bg_string + "_" + zc_string + ".bin";

    int8_t* input[kNumCodeBlocks];
    int8_t* parity[kNumCodeBlocks];
    int8_t* encoded[kNumCodeBlocks];
    int8_t* parity_reference[kNumCodeBlocks];
    for (size_t n = 0; n < kNumCodeBlocks; n++) {
        // We add kMaxProcBytes as padding for the encoder's scatter (for
        // input) and gather (for output) functions.
        input[n] = (int8_t*)read_binfile(
            input_filename, ldpc_encoding_input_buf_size(base_graph, zc));
        parity[n] = new int8_t[ldpc_encoding_parity_buf_size(base_graph, zc)];
        encoded[n] = new int8_t[ldpc_encoding_encoded_buf_size(base_graph, zc)];
        parity_reference[n] = (int8_t*)read_binfile(
            reference_filename, ldpc_encoding_parity_buf_size(base_graph, zc));

        ldpc_encode_helper(base_graph, zc, encoded[n], parity[n], input[n]);
    }

    for (size_t n = 0; n < kNumCodeBlocks; n++) {
        if (memcmp(parity[n], parity_reference[n],
                bits_to_bytes(ldpc_num_parity_bits(base_graph, zc)))
            != 0) {
            fprintf(stderr, "Mismatch for Zc = %zu, base graph = %zu\n", zc,
                base_graph);
        } else {
            printf("Passed for Zc = %zu, base graph = %zu\n", zc, base_graph);
        }
    }

    for (size_t n = 0; n < kNumCodeBlocks; n++) {
        delete[] input[n];
        delete[] parity[n];
        delete[] encoded[n];
        delete[] parity_reference[n];
    }
}

int main()
{
    // All possible expansion factors Zc in 5G NR
    std::vector<size_t> zc_all_vec = { 2, 4, 8, 16, 32, 64, 128, 256, 3, 6, 12,
        24, 48, 96, 192, 384, 5, 10, 20, 40, 80, 160, 320, 7, 14, 28, 56, 112,
        224, 9, 18, 36, 72, 144, 288, 11, 22, 44, 88, 176, 352, 13, 26, 52, 104,
        208, 15, 30, 60, 120, 240 };
    std::sort(zc_all_vec.begin(), zc_all_vec.end());

    // For some expansion factors, we don't have input and reference files yet
    const std::vector<size_t> zc_nofiles_vec = { 2, 3, 4, 5, 6, 9, 13 };

    for (const size_t& zc : zc_all_vec) {
        if (zc > 255)
            continue;

        const bool no_files = std::find(std::begin(zc_nofiles_vec),
                                  std::end(zc_nofiles_vec), zc)
            != std::end(zc_nofiles_vec);

        if (zc <= ZC_MAX and !no_files) {
            printf("Running for zc = %zu\n", zc);
            run_test(1 /* base graph */, zc);
            run_test(2 /* base graph */, zc);
        }
    }
}