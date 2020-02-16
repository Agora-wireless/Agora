/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */

#include "millipede.hpp"

void read_from_file_ul(std::string filename, Table<uint8_t>& data, int ofdm_size, Config* cfg)
{
    int data_symbol_num_perframe = cfg->data_symbol_num_perframe;
    size_t UE_NUM = cfg->UE_NUM;
    FILE *fp = fopen(filename.c_str(), "rb");
    if (fp == NULL) {
        printf("open file failed: %s\n", filename.c_str());
        std::cerr << "Error: " << strerror(errno) << std::endl;
    }
    for (int i = 0; i < data_symbol_num_perframe; i++) {
        if (ofdm_size * UE_NUM != 
            fread(data[i], sizeof(uint8_t), ofdm_size * UE_NUM, fp)) {
            printf("read file failed: %s\n", filename.c_str());
            std::cerr << "Error: " << strerror(errno) << std::endl;
        }
    } 
}

void read_from_file_dl(std::string filename, Table<float>& data, int ofdm_size, Config* cfg)
{
    int data_symbol_num_perframe = cfg->dl_data_symbol_num_perframe;
    size_t BS_ANT_NUM = cfg->BS_ANT_NUM;
    FILE *fp = fopen(filename.c_str(), "rb");
    if (fp == NULL) {
        printf("open file failed: %s\n", filename.c_str());
        std::cerr << "Error: " << strerror(errno) << std::endl;
    }
    for (size_t i = 0; i < data_symbol_num_perframe * BS_ANT_NUM; i++) {
        if ((unsigned) ofdm_size * 2 != 
            fread(data[i], sizeof(float), ofdm_size * 2, fp)) {
            printf("read file failed: %s\n", filename.c_str());
            std::cerr << "Error: " << strerror(errno) << std::endl;
        }
    } 
}

void check_correctness_ul(Config *cfg)  
{
    int BS_ANT_NUM = cfg->BS_ANT_NUM;
    int UE_NUM = cfg->UE_NUM;
    int data_symbol_num_perframe = cfg->ul_data_symbol_num_perframe;
    int OFDM_DATA_NUM = cfg->OFDM_DATA_NUM;

    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string raw_data_filename = cur_directory + "/data/orig_data_2048_ant" +
                         std::to_string(BS_ANT_NUM) + ".bin";
    std::string demul_data_filename = cur_directory + "/data/demul_data.bin";
    Table<uint8_t> raw_data;
    Table<uint8_t> demul_data;
    raw_data.calloc(data_symbol_num_perframe, OFDM_DATA_NUM * UE_NUM, 64);
    demul_data.calloc(data_symbol_num_perframe, OFDM_DATA_NUM * UE_NUM, 64);
                    
    read_from_file_ul(raw_data_filename, raw_data, OFDM_DATA_NUM, cfg);    
    read_from_file_ul(demul_data_filename, demul_data, OFDM_DATA_NUM, cfg);   

    int error_cnt = 0;
    int total_count = 0;
    for (int i = 0; i < data_symbol_num_perframe; i++) {
        for (int sc = 0; sc < OFDM_DATA_NUM; sc++) {
            for (int ue = 0; ue < UE_NUM; ue++) {
                total_count++;
                int offset_in_raw = OFDM_DATA_NUM * ue + sc;
                int offset_in_demul = UE_NUM * sc + ue;
                // if (i == 0)
                //     printf("(%d, %u, %u) ", sc, raw_data[i][offset_in_raw], demul_data[i][offset_in_demul]);
                if (raw_data[i][offset_in_raw] != demul_data[i][offset_in_demul]) 
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
        printf("Failed uplink test! Error rate: %d/%d\n", error_cnt, total_count);
    printf("======================\n\n");
    raw_data.free();
    demul_data.free();
}

void check_correctness_dl(Config *cfg)  
{
    int BS_ANT_NUM = cfg->BS_ANT_NUM;
    int data_symbol_num_perframe = cfg->dl_data_symbol_num_perframe;
    int OFDM_CA_NUM = cfg->OFDM_CA_NUM;

    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string raw_data_filename = cur_directory + "/data/dl_ifft_data_2048_ant" +
                         std::to_string(BS_ANT_NUM) + ".bin";
    std::string ifft_data_filename = cur_directory + "/data/ifft_data.bin";
    Table<float> raw_data;
    Table<float> ifft_data;
    raw_data.calloc(data_symbol_num_perframe * BS_ANT_NUM, OFDM_CA_NUM * 2, 64);
    ifft_data.calloc(data_symbol_num_perframe * BS_ANT_NUM, OFDM_CA_NUM * 2, 64);
                    
    read_from_file_dl(raw_data_filename, raw_data, OFDM_CA_NUM, cfg);    
    read_from_file_dl(ifft_data_filename, ifft_data, OFDM_CA_NUM, cfg);   

    int error_cnt = 0;
    int total_count = 0;
    for (int i = 0; i < data_symbol_num_perframe; i++) {
        for (int ant = 0; ant < BS_ANT_NUM; ant++) {
            // printf("symbol %d, antenna %d\n", i, ant);
            for (int sc = 0; sc < OFDM_CA_NUM * 2; sc++) {
                total_count++;
                int offset = BS_ANT_NUM * i + ant;
                // if (i == 0)
                //     printf("sc :%d, (%.3f, %.3f) diff: %.3f\n", sc / 2, 
                //         raw_data[offset][sc], ifft_data[offset][sc], 
                //         fabs(raw_data[offset][sc] - ifft_data[offset][sc]));
                if (fabs(raw_data[offset][sc] - ifft_data[offset][sc]) > 2) {
                    error_cnt++;
                }
            }
            // if (i == 0) printf("\n");
        }
    }    
    printf("======================\n");
    printf("Downlink test: \n\n");
    if (error_cnt == 0) 
        printf("Passed downlink test!\n");
    else 
        printf("Failed downlink test! Error rate: %d/%d\n", error_cnt, total_count);
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
    Config* cfg = new Config(filename.c_str());
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
