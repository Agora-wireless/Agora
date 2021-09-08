#include "control.hpp"
#include "Symbols.hpp"
#include "config.hpp"
#include <cstdio>
#include <string>
#include <set>
#include <gflags/gflags.h>

DEFINE_string(conf_file,
    TOSTRING(PROJECT_DIRECTORY) "/data/tddconfig-sim-ul.json",
    "Agora config filename");

size_t num_rb_gen(double avg_load, size_t total_rb) {
    double p_base = 1.0 / (1 + total_rb);
    double p_min = (total_rb + 1) * (2 * total_rb + 1) / 3.0 / total_rb * p_base - avg_load;
    p_min = p_min / ((total_rb + 1) * (2 * total_rb + 1) / 3.0 / total_rb - (total_rb + 1) / 2.0);
    double p_delta = (2 * p_base - 2 * p_min) / total_rb;
    double rdm = (double)rand() / RAND_MAX;
    double s = 0;
    double p = p_min;
    for (size_t i = 0; i <= total_rb; i ++) {
        s += p;
        if (s > rdm) {
            return i < 10 ? 10 : i;
        }
        p += p_delta;
    }
    return total_rb;
}

int main(int argc, char **argv)
{
    // Load the config file
    const std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    auto* cfg = new Config(FLAGS_conf_file.c_str());
    bool verbose = true;

    // 5G RAN configurations
    // const size_t num_ue = 16;
    const size_t num_ue = cfg->UE_NUM;
    // const size_t num_slots = 4 * 10;
    const size_t num_user_levels = cfg->user_level_list.size();
    const size_t num_load_levels = cfg->num_load_levels;
    const size_t num_slots = cfg->user_level_list.size() * cfg->num_load_levels;
    const size_t num_sc_per_rb = cfg->UE_NUM;
    const size_t num_rb = cfg->OFDM_DATA_NUM / num_sc_per_rb;
    const size_t max_ue_per_sc = cfg->UE_NUM;
    const double avg_load = 0.5;
    srand(time(NULL));

    // Open control file to write for all UEs
    FILE *file; 

    std::string filename = cur_directory + "/data/control_ue_template.bin";
    file = fopen(filename.c_str(), "wb");

    std::set<size_t> zc_list = {4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
        15, 16, 18, 20, 22, 24, 26, 28, 30, 32, 36, 40, 44, 48, 52, 56, 
        60, 64, 72, 80, 88, 96, 104};
    // std::set<size_t> zc_list = {4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
    //     15, 16, 18, 20, 22, 24, 26, 28, 30, 32, 36, 40, 44, 48, 52, 56, 
    //     60, 64, 72};

    // Assign control struct to all UEs
    for (size_t i = 0; i < num_slots; i ++) {
        // Basic settings
        // size_t num_ue_slot = (i / 10 + 1) * 4;
        size_t num_ue_slot = cfg->user_level_list[i / num_load_levels];
        // double load = 0.1 * (i % 10 + 1);
        double load = 1.0 / num_load_levels * (i % num_load_levels + 1);

        // Count UE num for each resource block
        size_t ue_counter[num_rb] = {0};
        size_t rb_start = 0;

        for (size_t j = 0; j < num_ue_slot; j ++) {
            // Generate the rb num for this ue
            // size_t n_rb = num_rb_gen(load, num_rb);
            size_t n_rb = load * num_rb;
            n_rb = MIN(n_rb, num_rb - rb_start);

            // Generate the MCS index and LDPC coding scheme
            size_t mod_order_bits = cfg->mod_order_bits;
            size_t num_bits_per_symbol = n_rb * num_sc_per_rb * mod_order_bits;
            size_t bg = cfg->LDPC_config.Bg;
            size_t n_rows = bg == 1 ? 46 : 42;
            size_t zc = num_bits_per_symbol / (ldpc_num_input_cols(bg) + n_rows - 2);
            zc = *(--zc_list.upper_bound(zc));

            ControlInfo info = {i, j, rb_start * num_sc_per_rb, (rb_start + n_rb) * num_sc_per_rb, mod_order_bits, bg, zc};
            fwrite(&info, sizeof(ControlInfo), 1, file);

            if (verbose) {
                printf("[Slot %u]: UE %u uses rb range (%u:%u), mod %u, Bg %u, Zc %u\n",
                    i, j, rb_start, rb_start + n_rb, mod_order_bits, bg, zc);
            }

            // Update rb usage info
            for (size_t k = rb_start; k < rb_start + n_rb; k ++) {
                ue_counter[k] ++;
            }
            while (rb_start < num_rb && ue_counter[rb_start] == max_ue_per_sc) {
                rb_start ++;
            }
        }
    }

    // Close files
    fclose(file);

    filename = cur_directory + "/data/control_ue.bin";
    file = fopen(filename.c_str(), "wb");

    for (size_t i = 0; i < cfg->frames_to_test; i ++) {
        // size_t control_idx = rand() % num_slots;
        // size_t control_idx = 1;
        size_t control_idx;
        if (cfg->fixed_control == -1) {
            control_idx = rand() % num_slots;
        } else {
            control_idx = cfg->fixed_control;
        }
        fwrite(&control_idx, sizeof(size_t), 1, file);
    }

    fclose(file);

    return 0;
}