/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */

#include "millipede.hpp"

void read_from_file_ul(std::string filename, Table<uint8_t>& data,
    int num_bytes_per_ue, Config* cfg)
{
    int data_symbol_num_perframe = cfg->data_symbol_num_perframe;
    size_t UE_NUM = cfg->UE_NUM;
    FILE* fp = fopen(filename.c_str(), "rb");
    if (fp == NULL) {
        printf("open file failed: %s\n", filename.c_str());
        std::cerr << "Error: " << strerror(errno) << std::endl;
    }
    int expect_num_bytes = num_bytes_per_ue * UE_NUM;
    // printf("read data of %d byptes\n", expect_num_bytes);
    for (int i = 0; i < data_symbol_num_perframe; i++) {
        int num_bytes = fread(data[i], sizeof(uint8_t), expect_num_bytes, fp);
        // if (i == 0) {
        //     for(int j = 0; j < num_bytes; j++) {
        //         printf("%u ", data[i][j]);
        //     }
        //     printf("\n");
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
    std::string filename, Table<float>& data, int ofdm_size, Config* cfg)
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
            != fread(data[i], sizeof(float), ofdm_size * 2, fp)) {
            printf("read file failed: %s\n", filename.c_str());
            std::cerr << "Error: " << strerror(errno) << std::endl;
        }
    }
}

void check_correctness_ul(Config* cfg)
{
    int BS_ANT_NUM = cfg->BS_ANT_NUM;
    int UE_NUM = cfg->UE_NUM;
    int data_symbol_num_perframe = cfg->ul_data_symbol_num_perframe;
    int OFDM_DATA_NUM = cfg->OFDM_DATA_NUM;

    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
#ifdef USE_LDPC
    std::string raw_data_filename = cur_directory
        + "/data/LDPC_orig_data_2048_ant" + std::to_string(BS_ANT_NUM) + ".bin";
    std::string output_data_filename = cur_directory + "/data/decode_data.bin";
#else
    std::string raw_data_filename = cur_directory + "/data/orig_data_2048_ant"
        + std::to_string(BS_ANT_NUM) + ".bin";
    std::string output_data_filename = cur_directory + "/data/demul_data.bin";
#endif

    Table<uint8_t> raw_data;
    Table<uint8_t> output_data;
    raw_data.calloc(data_symbol_num_perframe, OFDM_DATA_NUM * UE_NUM, 64);
    output_data.calloc(data_symbol_num_perframe, OFDM_DATA_NUM * UE_NUM, 64);

    int num_bytes_per_ue;
#ifdef USE_LDPC
    LDPCconfig LDPC_config = cfg->LDPC_config;
    num_bytes_per_ue
        = (LDPC_config.cbLen + 7) >> 3 * LDPC_config.nblocksInSymbol;
#else
    num_bytes_per_ue = OFDM_DATA_NUM;
#endif
    read_from_file_ul(raw_data_filename, raw_data, num_bytes_per_ue, cfg);
    read_from_file_ul(output_data_filename, output_data, num_bytes_per_ue, cfg);

    int error_cnt = 0;
    int total_count = 0;
    for (int i = 0; i < data_symbol_num_perframe; i++) {
#ifdef USE_LDPC
        for (int ue = 0; ue < UE_NUM; ue++) {
            for (int j = 0; j < num_bytes_per_ue; j++) {
                total_count++;
                int offset_in_raw = num_bytes_per_ue * ue + j;
                int offset_in_output = num_bytes_per_ue * ue + j;
#else
        for (int j = 0; j < num_bytes_per_ue; j++) {
            for (int ue = 0; ue < UE_NUM; ue++) {
                total_count++;
                int offset_in_raw = num_bytes_per_ue * ue + j;
                int offset_in_output = UE_NUM * j + ue;
#endif
                // if (i == 0)
                //     printf("(%d, %u, %u) ", j, raw_data[i][offset_in_raw],
                //     output_data[i][offset_in_output]);
                if (raw_data[i][offset_in_raw]
                    != output_data[i][offset_in_output])
                    error_cnt++;
            }
            // if (i == 0)
            //     printf("\n");
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

    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
#ifdef USE_LDPC
    std::string raw_data_filename = cur_directory
        + "/data/LDPC_dl_ifft_data_2048_ant" + std::to_string(BS_ANT_NUM)
        + ".bin";
#else
    std::string raw_data_filename = cur_directory
        + "/data/dl_ifft_data_2048_ant" + std::to_string(BS_ANT_NUM) + ".bin";
#endif
    std::string ifft_data_filename = cur_directory + "/data/ifft_data.bin";
    Table<float> raw_data;
    Table<float> ifft_data;
    raw_data.calloc(data_symbol_num_perframe * BS_ANT_NUM, OFDM_CA_NUM * 2, 64);
    ifft_data.calloc(
        data_symbol_num_perframe * BS_ANT_NUM, OFDM_CA_NUM * 2, 64);

    read_from_file_dl(raw_data_filename, raw_data, OFDM_CA_NUM, cfg);
    read_from_file_dl(ifft_data_filename, ifft_data, OFDM_CA_NUM, cfg);

    int error_cnt = 0;
    int total_count = 0;
    float sum_diff = 0;
    for (int i = 0; i < data_symbol_num_perframe; i++) {
        if (i != DL_PILOT_SYMS - 1) {
            for (int ant = 0; ant < BS_ANT_NUM; ant++) {
                // printf("symbol %d, antenna %d\n", i, ant);
                sum_diff = 0;
                total_count++;
                for (int sc = 0; sc < OFDM_CA_NUM * 2; sc++) {
                    int offset = BS_ANT_NUM * i + ant;
                    float diff
                        = fabs(raw_data[offset][sc] - ifft_data[offset][sc]);
                    sum_diff += diff;
                    // if (i == 0)
                    // printf("symbol %d ant %d sc %d, (%.3f, %.3f) diff:
                    // %.3f\n",
                    //     i, ant, sc / 2, raw_data[offset][sc],
                    //     ifft_data[offset][sc], diff);
                }
                printf(
                    "symbol %d, ant %d, total diff %.3f\n", i, ant, sum_diff);
                if (sum_diff > OFDM_CA_NUM * 10)
                    error_cnt++;
            }
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
    ifft_data.free();
}

int main(int argc, char* argv[])
{
    std::string confFile;
    if (argc == 2)
        confFile = std::string("/") + std::string(argv[1]);
    else
        confFile = "/data/tddconfig-correctness-test-ul.json";
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + confFile;
    auto* cfg = new Config(filename.c_str());
    cfg->genData();
    Millipede* millipede_cli;

    int ret;
    try {
        SignalHandler signalHandler;

        // Register signal handler to handle kill signal
        signalHandler.setupSignalHandlers();
        millipede_cli = new Millipede(cfg);
        millipede_cli->start();

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
