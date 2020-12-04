#include "agora.hpp"

void read_from_file_ul(std::string filename, Table<uint8_t>& data,
    int num_bytes_per_ue, Config* cfg)
{
    int data_symbol_num_perframe = cfg->ul_data_symbol_num_perframe;
    size_t ue_num = cfg->ue_num();
    FILE* fp = std::fopen(filename.c_str(), "rb");
    if (fp == NULL) {
        std::printf("open file failed: %s\n", filename.c_str());
        std::cerr << "Error: " << strerror(errno) << std::endl;
    } else {
        std::printf("opening file %s\n", filename.c_str());
    }
    int expect_num_bytes = num_bytes_per_ue * ue_num;
    // std::printf("read data of %d byptes\n", expect_num_bytes);
    for (int i = 0; i < data_symbol_num_perframe; i++) {
        int num_bytes = std::fread(data[i], sizeof(uint8_t), expect_num_bytes, fp);
        // if (i == 0) {
        // std::printf("i: %d\n", i);
        // for (int j = 0; j < num_bytes; j++) {
        //     std::printf("%u ", data[i][j]);
        // }
        // std::printf("\n");
        // }
        if (expect_num_bytes != num_bytes) {
            std::printf("read file failed: %s, symbol %d, expect: %d, actual: %d "
                   "bytes\n",
                filename.c_str(), i, expect_num_bytes, num_bytes);
            std::cerr << "Error: " << strerror(errno) << std::endl;
        }
    }
}

void read_from_file_dl(
    std::string filename, Table<short>& data, int ofdm_size, Config* cfg)
{
    int data_symbol_num_perframe = cfg->dl_data_symbol_num_perframe;
    size_t bs_ant_num = cfg->bs_ant_num();
    FILE* fp = std::fopen(filename.c_str(), "rb");
    if (fp == NULL) {
        std::printf("open file failed: %s\n", filename.c_str());
        std::cerr << "Error: " << strerror(errno) << std::endl;
    }
    for (size_t i = 0; i < data_symbol_num_perframe * bs_ant_num; i++) {
        if ((unsigned)ofdm_size * 2
            != std::fread(data[i], sizeof(short), ofdm_size * 2, fp)) {
            std::printf("read file failed: %s\n", filename.c_str());
            std::cerr << "Error: " << strerror(errno) << std::endl;
        }
    }
}

void check_correctness_ul(Config* cfg)
{
    int ue_num = cfg->ue_num();
    int data_symbol_num_perframe = cfg->ul_data_symbol_num_perframe;
    int ofdm_data_num = cfg->ofdm_data_num();
    int ul_pilot_syms = cfg->ul_pilot_syms();

    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string raw_data_filename = cur_directory + "/data/LDPC_orig_data_"
        + std::to_string(cfg->ofdm_ca_num()) + "_ant"
        + std::to_string(cfg->ue_num()) + ".bin";
    std::string output_data_filename = cur_directory + "/data/decode_data.bin";

    Table<uint8_t> raw_data;
    Table<uint8_t> output_data;
    raw_data.calloc(data_symbol_num_perframe, ofdm_data_num * ue_num, Agora_memory::Alignment_t::k64Align);
    output_data.calloc(data_symbol_num_perframe, ofdm_data_num * ue_num, Agora_memory::Alignment_t::k64Align);

    int num_bytes_per_ue
        = (cfg->ldpc_config().num_cb_len() + 7) >> 3 * cfg->ldpc_config().num_blocks_in_symbol();
    read_from_file_ul(raw_data_filename, raw_data, num_bytes_per_ue, cfg);
    read_from_file_ul(output_data_filename, output_data, num_bytes_per_ue, cfg);

    int error_cnt = 0;
    int total_count = 0;
    for (int i = 0; i < data_symbol_num_perframe; i++) {
        if (i < ul_pilot_syms)
            continue;
        for (int ue = 0; ue < ue_num; ue++) {
            for (int j = 0; j < num_bytes_per_ue; j++) {
                total_count++;
                int offset_in_raw = num_bytes_per_ue * ue + j;
                int offset_in_output = num_bytes_per_ue * ue + j;
                if (raw_data[i][offset_in_raw]
                    != output_data[i][offset_in_output]) {
                    error_cnt++;
                    // std::printf("(%d, %d, %u, %u)\n", i, j,
                    //     raw_data[i][offset_in_raw],
                    //     output_data[i][offset_in_output]);
                }
            }
        }
    }
    std::printf("======================\n");
    std::printf("Uplink test: \n\n");
    if (error_cnt == 0)
        std::printf("Passed uplink test!\n");
    else
        std::printf(
            "Failed uplink test! Error rate: %d/%d\n", error_cnt, total_count);
    std::printf("======================\n\n");
    raw_data.free();
    output_data.free();
}

void check_correctness_dl(Config* cfg)
{
    int bs_ant_num = cfg->bs_ant_num();
    int data_symbol_num_perframe = cfg->dl_data_symbol_num_perframe;
    int ofdm_ca_num = cfg->ofdm_ca_num();
    int sampsPerSymbol = cfg->sampsPerSymbol;

    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string raw_data_filename = cur_directory + "/data/LDPC_dl_tx_data_"
        + std::to_string(ofdm_ca_num) + "_ant" + std::to_string(bs_ant_num)
        + ".bin";
    std::string tx_data_filename = cur_directory + "/data/tx_data.bin";
    Table<short> raw_data;
    Table<short> tx_data;
    raw_data.calloc(
        data_symbol_num_perframe * bs_ant_num, sampsPerSymbol * 2, Agora_memory::Alignment_t::k64Align);
    tx_data.calloc(
        data_symbol_num_perframe * bs_ant_num, sampsPerSymbol * 2, Agora_memory::Alignment_t::k64Align);

    read_from_file_dl(raw_data_filename, raw_data, sampsPerSymbol, cfg);
    read_from_file_dl(tx_data_filename, tx_data, sampsPerSymbol, cfg);

    int error_cnt = 0;
    int total_count = 0;
    float sum_diff = 0;
    for (int i = 0; i < data_symbol_num_perframe; i++) {
        for (int ant = 0; ant < bs_ant_num; ant++) {
            // std::printf("symbol %d, antenna %d\n", i, ant);
            sum_diff = 0;
            total_count++;
            for (int sc = 0; sc < sampsPerSymbol * 2; sc++) {
                int offset = bs_ant_num * i + ant;
                float diff = fabs(
                    (raw_data[offset][sc] - tx_data[offset][sc]) / 32768.0);
                sum_diff += diff;
                // if (i == 0)
                // std::printf("symbol %d ant %d sc %d, (%d, %d) diff: %.3f\n", i, ant,
                //     sc / 2, raw_data[offset][sc], tx_data[offset][sc], diff);
            }
            float avg_diff = sum_diff / sampsPerSymbol;
            std::printf("symbol %d, ant %d, mean per-sample diff %.3f\n", i, ant,
                avg_diff);
            if (avg_diff > 0.03)
                error_cnt++;
        }
    }
    std::printf("======================\n");
    std::printf("Downlink test: \n\n");
    if (error_cnt == 0)
        std::printf("Passed downlink test!\n");
    else
        std::printf("Failed downlink test! Error rate: %d/%d\n", error_cnt,
            total_count);
    std::printf("======================\n\n");
    raw_data.free();
    tx_data.free();
}

int main(int argc, char* argv[])
{
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string confFile
        = cur_directory + "/data/tddconfig-correctness-test-ul.json";
    if (argc == 2)
        confFile = std::string(argv[1]);

    auto* cfg = new Config(confFile.c_str());
    cfg->genData();

    int ret;
    try {
        SignalHandler signalHandler;
        signalHandler.setupSignalHandlers();
        auto* agora_cli = new Agora(cfg);
        agora_cli->flags.enable_save_decode_data_to_file = true;
        agora_cli->flags.enable_save_tx_data_to_file = true;
        agora_cli->start();

        std::printf("Start correctness check\n");
        if (cfg->downlink_mode()) {
            check_correctness_dl(cfg);
        }
        else {
            check_correctness_ul(cfg);
        }
        ret = EXIT_SUCCESS;
    } catch (SignalException& e) {
        std::cerr << "SignalException: " << e.what() << std::endl;
        ret = EXIT_FAILURE;
    }
    return ret;
}
