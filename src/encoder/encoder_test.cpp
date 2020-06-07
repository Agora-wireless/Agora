#include "encoder.hpp"
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

static size_t bits_to_bytes(size_t num_bits) { return (num_bits + 7) / 8; };

void run_test(size_t base_graph, size_t zc)
{
    const std::string bg_string = base_graph == 1 ? "BG1" : "BG2";
    const std::string zc_string = std::string("Zc") + std::to_string(zc);
    const std::string input_filename = std::string("test_vectors/input_")
        + bg_string + "_" + zc_string + ".bin";
    const std::string reference_filename = std::string("test_vectors/output_")
        + bg_string + "_" + zc_string + ".bin";

    avx2enc::bblib_ldpc_encoder_5gnr_request req;
    req.Zc = zc;
    req.baseGraph = base_graph;
    req.numberCodeblocks = kNumCodeBlocks;
    avx2enc::bblib_ldpc_encoder_5gnr_response resp;

    size_t num_information_bits = zc
        * (base_graph == 1 ? avx2enc::BG1_COL_INF_NUM
                           : avx2enc::BG2_COL_INF_NUM);
    size_t num_parity_bits = zc
        * (base_graph == 1 ? avx2enc::BG1_ROW_TOTAL : avx2enc::BG2_ROW_TOTAL);

    int8_t* reference[kNumCodeBlocks];
    for (int i = 0; i < kNumCodeBlocks; i++) {
        // We add avx2enc::PROC_BYTES as padding for the encoder's scatter (for
        // input) and gather (for output) functions.
        req.input[i] = (int8_t*)read_binfile(input_filename,
            bits_to_bytes(num_information_bits) + avx2enc::PROC_BYTES);
        resp.output[i]
            = new int8_t[bits_to_bytes(num_parity_bits) + avx2enc::PROC_BYTES];
        reference[i] = (int8_t*)read_binfile(
            reference_filename, bits_to_bytes(num_parity_bits));
    }

    avx2enc::ldpc_encoder_avx2(&req, &resp);

    for (size_t n = 0; n < kNumCodeBlocks; n++) {
        if (memcmp(resp.output[n], reference[n], bits_to_bytes(num_parity_bits))
            != 0) {
            fprintf(stderr, "Mismatch for Zc = %zu, base graph = %zu\n", zc,
                base_graph);
        } else {
            printf("Passed for Zc = %zu, base graph = %zu\n", zc, base_graph);
        }
    }

    for (int i = 0; i < kNumCodeBlocks; i++) {
        delete[] req.input[i];
        delete[] resp.output[i];
        delete[] reference[i];
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

    // For some expansion factors, we don't have input and reference files
    const std::vector<size_t> zc_nofiles_vec = { 2, 3, 4, 5, 6, 9, 13 };

    for (const auto& zc : zc_all_vec) {
        const bool no_files = std::find(std::begin(zc_nofiles_vec),
                                  std::end(zc_nofiles_vec), zc)
            != std::end(zc_nofiles_vec);

        if (zc <= avx2enc::ZC_MAX and !no_files) {
            run_test(1 /* base graph */, zc);
            run_test(2 /* base graph */, zc);
        }
    }
}