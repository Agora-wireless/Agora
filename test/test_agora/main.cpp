#include "agora.hpp"

void read_from_file_ul(std::string filename, Table<uint8_t>& data,
    int num_bytes_per_ue, Config* cfg)
{
    int data_symbol_num_perframe = cfg->ul_data_symbol_num_perframe;
    size_t UE_NUM = cfg->UE_NUM;
    FILE* fp = fopen(filename.c_str(), "rb");
    if (fp == NULL) {
        printf("open file failed: %s\n", filename.c_str());
        std::cerr << "Error: " << strerror(errno) << std::endl;
    } else {
        printf("opening file %s\n", filename.c_str());
    }
    int expect_num_bytes = num_bytes_per_ue * UE_NUM;
    // printf("read data of %d byptes\n", expect_num_bytes);
    for (int i = 0; i < data_symbol_num_perframe; i++) {
        int num_bytes = fread(data[i], sizeof(uint8_t), expect_num_bytes, fp);
        // if (i == 0) {
        // printf("i: %d\n", i);
        // for (int j = 0; j < num_bytes; j++) {
        //     printf("%u ", data[i][j]);
        // }
        // printf("\n");
        // }
        if (expect_num_bytes != num_bytes) {
            printf("read file failed: %s, symbol %d, expect: %d, actual: %d "
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
    size_t BS_ANT_NUM = cfg->BS_ANT_NUM;
    FILE* fp = fopen(filename.c_str(), "rb");
    if (fp == NULL) {
        printf("open file failed: %s\n", filename.c_str());
        std::cerr << "Error: " << strerror(errno) << std::endl;
    }
    for (size_t i = 0; i < data_symbol_num_perframe * BS_ANT_NUM; i++) {
        if ((unsigned)ofdm_size * 2
            != fread(data[i], sizeof(short), ofdm_size * 2, fp)) {
            printf("read file failed: %s\n", filename.c_str());
            std::cerr << "Error: " << strerror(errno) << std::endl;
        }
    }
}

void check_correctness_ul(Config* cfg)
{
    int UE_NUM = cfg->UE_NUM;
    int data_symbol_num_perframe = cfg->ul_data_symbol_num_perframe;
    int OFDM_DATA_NUM = cfg->OFDM_DATA_NUM;
    int UL_PILOT_SYMS = cfg->UL_PILOT_SYMS;

    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string raw_data_filename = cur_directory + "/data/LDPC_orig_data_"
        + std::to_string(cfg->OFDM_CA_NUM) + "_ant"
        + std::to_string(cfg->UE_NUM) + ".bin";
    std::string output_data_filename = cur_directory + "/data/decode_data.bin";

    Table<uint8_t> raw_data;
    Table<uint8_t> output_data;
    raw_data.calloc(data_symbol_num_perframe, OFDM_DATA_NUM * UE_NUM, 64);
    output_data.calloc(data_symbol_num_perframe, OFDM_DATA_NUM * UE_NUM, 64);

    int num_bytes_per_ue
        = (cfg->LDPC_config.cbLen + 7) >> 3 * cfg->LDPC_config.nblocksInSymbol;
    read_from_file_ul(raw_data_filename, raw_data, num_bytes_per_ue, cfg);
    read_from_file_ul(output_data_filename, output_data, num_bytes_per_ue, cfg);

    int error_cnt = 0;
    int total_count = 0;
    for (int i = 0; i < data_symbol_num_perframe; i++) {
        if (i < UL_PILOT_SYMS)
            continue;
        for (int ue = 0; ue < UE_NUM; ue++) {
            for (int j = 0; j < num_bytes_per_ue; j++) {
                total_count++;
                int offset_in_raw = num_bytes_per_ue * ue + j;
                int offset_in_output = num_bytes_per_ue * ue + j;
                if (raw_data[i][offset_in_raw]
                    != output_data[i][offset_in_output]) {
                    error_cnt++;
                    // printf("(%d, %d, %u, %u)\n", i, j,
                    //     raw_data[i][offset_in_raw],
                    //     output_data[i][offset_in_output]);
                }
            }
        }
    }
    printf("======================\n");
    printf("Uplink test: \n\n");
    if (error_cnt == 0)
        printf("Passed uplink test!\n");
    else
        printf(
            "Failed uplink test! Error rate: %d/%d\n", error_cnt, total_count);
    printf("======================\n\n");
    raw_data.free();
    output_data.free();
}

void check_correctness_dl(Config* cfg)
{
    int BS_ANT_NUM = cfg->BS_ANT_NUM;
    int data_symbol_num_perframe = cfg->dl_data_symbol_num_perframe;
    int OFDM_CA_NUM = cfg->OFDM_CA_NUM;
    int sampsPerSymbol = cfg->sampsPerSymbol;

    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string raw_data_filename = cur_directory + "/data/LDPC_dl_tx_data_"
        + std::to_string(OFDM_CA_NUM) + "_ant" + std::to_string(BS_ANT_NUM)
        + ".bin";
    std::string tx_data_filename = cur_directory + "/data/tx_data.bin";
    Table<short> raw_data;
    Table<short> tx_data;
    raw_data.calloc(
        data_symbol_num_perframe * BS_ANT_NUM, sampsPerSymbol * 2, 64);
    tx_data.calloc(
        data_symbol_num_perframe * BS_ANT_NUM, sampsPerSymbol * 2, 64);

    read_from_file_dl(raw_data_filename, raw_data, sampsPerSymbol, cfg);
    read_from_file_dl(tx_data_filename, tx_data, sampsPerSymbol, cfg);

    int error_cnt = 0;
    int total_count = 0;
    float sum_diff = 0;
    for (int i = 0; i < data_symbol_num_perframe; i++) {
        for (int ant = 0; ant < BS_ANT_NUM; ant++) {
            // printf("symbol %d, antenna %d\n", i, ant);
            sum_diff = 0;
            total_count++;
            for (int sc = 0; sc < sampsPerSymbol * 2; sc++) {
                int offset = BS_ANT_NUM * i + ant;
                float diff = fabs(
                    (raw_data[offset][sc] - tx_data[offset][sc]) / 32768.0);
                sum_diff += diff;
                // if (i == 0)
                // printf("symbol %d ant %d sc %d, (%d, %d) diff: %.3f\n", i, ant,
                //     sc / 2, raw_data[offset][sc], tx_data[offset][sc], diff);
            }
            float avg_diff = sum_diff / sampsPerSymbol;
            printf("symbol %d, ant %d, mean per-sample diff %.3f\n", i, ant,
                avg_diff);
            if (avg_diff > 0.03)
                error_cnt++;
        }
    }
    printf("======================\n");
    printf("Downlink test: \n\n");
    if (error_cnt == 0)
        printf("Passed downlink test!\n");
    else
        printf("Failed downlink test! Error rate: %d/%d\n", error_cnt,
            total_count);
    printf("======================\n\n");
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
        agora_cli->flags_.enable_save_decode_data_to_file_ = true;
        agora_cli->flags_.enable_save_tx_data_to_file_ = true;
        agora_cli->Start();

        printf("Start correctness check\n");
        if (cfg->downlink_mode)
            check_correctness_dl(cfg);
        else
            check_correctness_ul(cfg);

        ret = EXIT_SUCCESS;
    } catch (SignalException& e) {
        std::cerr << "SignalException: " << e.what() << std::endl;
        ret = EXIT_FAILURE;
    }
    return ret;
}
