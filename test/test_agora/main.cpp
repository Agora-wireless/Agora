#include "agora.hpp"


void read_from_file_ul(std::string filename, Table<uint8_t>& data,
    int num_bytes_per_ue, Config const * const cfg)
{
    int data_symbol_num_perframe = cfg->frame().NumULSyms();
    size_t ue_num = cfg->ue_num();
    FILE* fp = std::fopen(filename.c_str(), "rb");
    if (fp == NULL) {
        std::printf("open file failed: %s\n", filename.c_str());
        std::cerr << "Error: " << strerror(errno) << std::endl;
    } else {
        std::printf("opening file %s\n", filename.c_str());
    }
    const unsigned read_size = (num_bytes_per_ue * ue_num);
    for (int i = 0; i < data_symbol_num_perframe; i++) {
        size_t num_bytes = std::fread(data[i], sizeof(uint8_t), read_size, fp);
        if (read_size != num_bytes) {
            std::printf("read file failed: %s, symbol %d, expect: %d, actual: %zu "
                   "bytes\n",
                filename.c_str(), i, read_size, num_bytes);
            std::cerr << "Error: " << strerror(errno) << std::endl;
        }
    }
}

void read_from_file_dl(
    std::string filename, Table<short>& data, int ofdm_size, Config const * const cfg)
{
    int data_symbol_num_perframe = cfg->frame().NumDLSyms();
    size_t bs_ant_num = cfg->bs_ant_num();
    FILE* fp = std::fopen(filename.c_str(), "rb");
    if (fp == NULL) {
        std::printf("open file failed: %s\n", filename.c_str());
        std::cerr << "Error: " << strerror(errno) << std::endl;
    } else {
        std::printf("opening file %s\n", filename.c_str());
    }
    const unsigned read_size = static_cast<unsigned>(ofdm_size * 2u);
    for (size_t i = 0; i < (data_symbol_num_perframe * bs_ant_num); i++) {
        size_t num_bytes = std::fread(data[i], sizeof(short), read_size, fp);
        if (read_size != num_bytes) {
            std::printf("read file failed: %s\n", filename.c_str());
            std::cerr << "Error: " << strerror(errno) << std::endl;
        }
    }
}


void check_correctness_ul(Config const * const cfg)
{
    int ue_num = cfg->ue_num();
    int num_uplink_syms = cfg->frame().NumULSyms();
    int ofdm_data_num = cfg->ofdm_data_num();
    int ul_pilot_syms = cfg->frame().client_ul_pilot_symbols();

    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string raw_data_filename = cur_directory + "/data/LDPC_orig_data_"
        + std::to_string(cfg->ofdm_ca_num()) + "_ant"
        + std::to_string(cfg->ue_num()) + ".bin";
    std::string output_data_filename = cur_directory + "/data/decode_data.bin";

    Table<uint8_t> raw_data;
    Table<uint8_t> output_data;
    raw_data.calloc(num_uplink_syms, (ofdm_data_num * ue_num), Agora_memory::Alignment_t::k64Align);
    output_data.calloc(num_uplink_syms, (ofdm_data_num * ue_num), Agora_memory::Alignment_t::k64Align);

    int num_bytes_per_ue
        = (cfg->ldpc_config().num_cb_len() + 7) >> 3 * cfg->ldpc_config().num_blocks_in_symbol();
    read_from_file_ul(raw_data_filename, raw_data, num_bytes_per_ue, cfg);
    read_from_file_ul(output_data_filename, output_data, num_bytes_per_ue, cfg);

    std::printf("check_correctness_ul: ue %d, ul syms %d, ofdm %d, ul pilots %d, bytes per UE %d.\n", ue_num, num_uplink_syms, ofdm_data_num, ul_pilot_syms, num_bytes_per_ue);

    int error_cnt = 0;
    int total_count = 0;
    for (int i = 0; i < num_uplink_syms; i++) {
        if (i >= ul_pilot_syms) {
            for (int ue = 0; ue < ue_num; ue++) {
                for (int j = 0; j < num_bytes_per_ue; j++) {
                    total_count++;
                    int offset_in_raw = num_bytes_per_ue * ue + j;
                    int offset_in_output = num_bytes_per_ue * ue + j;
                    if (raw_data[i][offset_in_raw] != output_data[i][offset_in_output]) {
                        error_cnt++;
                        //std::printf("(%d, %d, %u, %u)\n", i, j,
                        //     raw_data[i][offset_in_raw],
                        //     output_data[i][offset_in_output]);
                    }
                }
            } //  for (int ue = 0; ue < ue_num; ue++) {
        } // if (i >= ul_pilot_syms) {
    } //for (int i = 0; i < num_uplink_syms; i++) {
    std::printf("======================\n");
    std::printf("Uplink test: \n\n");
    if (error_cnt == 0) {
        std::printf("Passed uplink test!\n");
    }
    else {
        std::printf(
            "Failed uplink test! Error rate: %d/%d\n", error_cnt, total_count);
    }
    std::printf("======================\n\n");
    raw_data.free();
    output_data.free();
}

void check_correctness_dl(Config const * const cfg)
{
    int bs_ant_num = cfg->bs_ant_num();
    int num_data_syms = cfg->frame().NumDLSyms();
    int ofdm_ca_num = cfg->ofdm_ca_num();
    int samps_per_symbol = cfg->samps_per_symbol();

    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string raw_data_filename = cur_directory + "/data/LDPC_dl_tx_data_"
        + std::to_string(ofdm_ca_num) + "_ant" + std::to_string(bs_ant_num)
        + ".bin";
    std::string tx_data_filename = cur_directory + "/data/tx_data.bin";
    Table<short> raw_data;
    Table<short> tx_data;
    raw_data.calloc(
        num_data_syms * bs_ant_num, samps_per_symbol * 2, Agora_memory::Alignment_t::k64Align);
    tx_data.calloc(
        num_data_syms * bs_ant_num, samps_per_symbol * 2, Agora_memory::Alignment_t::k64Align);

    read_from_file_dl(raw_data_filename, raw_data, samps_per_symbol, cfg);
    read_from_file_dl(tx_data_filename, tx_data, samps_per_symbol, cfg);
    std::printf("check_correctness_dl: bs ant %d, dl syms %d, ofdm %d, samps per %d. \n", bs_ant_num, num_data_syms, ofdm_ca_num, samps_per_symbol);

    int error_cnt = 0;
    int total_count = 0;
    float sum_diff = 0;
    for (int i = 0; i < num_data_syms; i++) {
        for (int ant = 0; ant < bs_ant_num; ant++) {
            // std::printf("symbol %d, antenna %d\n", i, ant);
            sum_diff = 0;
            total_count++;
            for (int sc = 0; sc < (samps_per_symbol * 2); sc++) {
                int offset = (bs_ant_num * i) + ant;
                float diff = fabs(
                    (raw_data[offset][sc] - tx_data[offset][sc]) / 32768.0);
                sum_diff += diff;
                // if (i == 0)
                // std::printf("symbol %d ant %d sc %d, (%d, %d) diff: %.3f\n", i, ant,
                //     sc / 2, raw_data[offset][sc], tx_data[offset][sc], diff);
            }
            float avg_diff = sum_diff / samps_per_symbol;
            std::printf("symbol %d, ant %d, mean per-sample diff %.3f\n", i, ant,
                avg_diff);
            if (avg_diff > 0.03) {
                error_cnt++;
            }
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


void check_correctness(Config const * const cfg) {
    check_correctness_ul(cfg);
    check_correctness_dl(cfg);
}


int main(int argc, char* argv[])
{
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string confFile
        = cur_directory + "/data/tddconfig-correctness-test-ul.json";
    if (argc == 2) {
        confFile = std::string(argv[1]);
    }

    std::unique_ptr<Config> cfg( new Config(confFile.c_str()) );
    cfg->GenData();

    int ret;
    try {
        SignalHandler signalHandler;
        signalHandler.setupSignalHandlers();
        std::unique_ptr<Agora> agora_cli( new Agora(cfg.get()) );
        agora_cli->flags.enable_save_decode_data_to_file = true;
        agora_cli->flags.enable_save_tx_data_to_file = true;
        agora_cli->start();

        std::printf("Start correctness check\n");

        if ((cfg->frame().NumDLSyms() > 0) && (cfg->frame().NumULSyms() > 0)) {
            check_correctness(cfg.get());
        } else if (cfg->frame().NumDLSyms() > 0) {
            check_correctness_dl(cfg.get());
        } else if (cfg->frame().NumULSyms() > 0) {
            check_correctness_ul(cfg.get());
        } else {
            //Should never happen
            assert(false);
        }

        ret = EXIT_SUCCESS;
    } catch (SignalException& e) {
        std::cerr << "SignalException: " << e.what() << std::endl;
        ret = EXIT_FAILURE;
    }
    return ret;
}
