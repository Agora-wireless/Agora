#include <string>

#include "agora.h"
#include "data_generator.h"
#include "datatype_conversion.h"
#include "gflags/gflags.h"
#include "logger.h"
#include "signal_handler.h"

static const bool kDebugPrintUlCorr = false;
static const bool kDebugPrintDlCorr = false;

static const std::string kUlCheckFilePrefix =
    kExperimentFilepath + kUlLdpcDataPrefix;
static const std::string kDlCheckFilePrefix = kExperimentFilepath + kDlTxPrefix;
static const std::string kTxFilename = kExperimentFilepath + "tx_data.bin";
static const std::string kDecodedFilename =
    kExperimentFilepath + "decode_data.bin";

template <class TableType>
static void ReadFromFile(const std::string& filename, Table<TableType>& data,
                         size_t seek_size, size_t num_reads,
                         size_t read_elements, size_t element_size) {
  FILE* fp = std::fopen(filename.c_str(), "rb");
  if (fp == nullptr) {
    AGORA_LOG_ERROR("Open file failed: %s, error %s\n", filename.c_str(),
                    strerror(errno));
  } else {
    AGORA_LOG_INFO("Opening file %s\n", filename.c_str());
  }
  if (seek_size != 0) {
    std::fseek(fp, seek_size, SEEK_SET);
  }
  for (size_t i = 0; i < num_reads; i++) {
    size_t elements = std::fread(data[i], element_size, read_elements, fp);
    if (read_elements != elements) {
      AGORA_LOG_ERROR(
          "Read file failed: %s, symbol %zu, expect: %zu, actual: %zu "
          "bytes, error %s\n",
          filename.c_str(), i, read_elements, elements, strerror(errno));
    }
  }
  fclose(fp);
}

static void ReadFromFileUl(const std::string& filename, Table<uint8_t>& data,
                           int ue_num, int num_bytes_per_ue,
                           Config const* const cfg) {
  ReadFromFile(filename, data, 0, cfg->Frame().NumULSyms(),
               (num_bytes_per_ue * ue_num), sizeof(uint8_t));
}

static void ReadFromFileDl(const std::string& filename, Table<short>& data,
                           size_t seek_size, int ofdm_size,
                           Config const* const cfg) {
  ReadFromFile(filename, data, seek_size,
               cfg->Frame().NumDLSyms() * cfg->BsAntNum(), (ofdm_size * 2),
               sizeof(short));
}

static unsigned int CheckCorrectnessUl(Config const* const cfg,
                                       arma::uvec spatial_streams) {
  size_t bs_ant_num = cfg->BsAntNum();
  size_t ue_num = cfg->UeAntNum();
  size_t num_uplink_syms = cfg->Frame().NumUlDataSyms();
  size_t ofdm_data_num = cfg->OfdmDataNum();
  size_t ul_pilot_syms = cfg->Frame().ClientUlPilotSymbols();
  size_t spatial_streams_num = spatial_streams.n_elem;

  const std::string raw_data_filename = kUlCheckFilePrefix +
                                        std::to_string(cfg->OfdmCaNum()) +
                                        "_ue" + std::to_string(ue_num) + ".bin";

  Table<uint8_t> raw_data;
  Table<uint8_t> output_data;
  raw_data.Calloc(num_uplink_syms, (ofdm_data_num * ue_num),
                  Agora_memory::Alignment_t::kAlign64);
  output_data.Calloc(num_uplink_syms, (ofdm_data_num * spatial_streams_num),
                     Agora_memory::Alignment_t::kAlign64);

  size_t num_bytes_per_ue =
      (cfg->LdpcConfig(Direction::kUplink).NumCbLen() + 7) >>
      3 * cfg->LdpcConfig(Direction::kUplink).NumBlocksInSymbol();
  ReadFromFileUl(raw_data_filename, raw_data, ue_num, num_bytes_per_ue, cfg);
  ReadFromFileUl(kDecodedFilename, output_data, spatial_streams_num,
                 num_bytes_per_ue, cfg);
  std::printf(
      "check_correctness_ul: bs ant %zu, ues %zu, spatial streams (last frame) "
      "%zu, "
      "ul syms %zu, ofdm %zu, ul pilots %zu, bytes per UE %zu.\n",
      bs_ant_num, ue_num, spatial_streams_num, num_uplink_syms, ofdm_data_num,
      ul_pilot_syms, num_bytes_per_ue);

  unsigned int error_cnt = 0;
  unsigned int total_count = 0;
  for (size_t i = 0; i < num_uplink_syms; i++) {
    for (size_t ue = 0; ue < spatial_streams_num; ue++) {
      for (size_t j = 0; j < num_bytes_per_ue; j++) {
        total_count++;
        size_t offset_in_raw = num_bytes_per_ue * spatial_streams(ue) + j;
        size_t offset_in_output = num_bytes_per_ue * ue + j;
        if (raw_data[i][offset_in_raw] != output_data[i][offset_in_output]) {
          error_cnt++;
          if (kDebugPrintUlCorr) {
            std::printf("(%zu, %zu, %zu, %u, %u)\n", i, ue, j,
                        raw_data[i][offset_in_raw],
                        output_data[i][offset_in_output]);
          }
        }
      }
    }  //  for (int ue = 0; ue < ue_num; ue++)
  }    // for (int i = 0; i < num_uplink_syms; i++)

  raw_data.Free();
  output_data.Free();

  return error_cnt;
}

unsigned int CheckCorrectnessDl(Config const* const cfg,
                                arma::uvec spatial_streams, size_t sched_id) {
  const size_t bs_ant_num = cfg->BsAntNum();
  const size_t ue_num = cfg->UeAntNum();
  const size_t num_data_syms = cfg->Frame().NumDLSyms();
  const size_t ofdm_ca_num = cfg->OfdmCaNum();
  const size_t samps_per_symbol = cfg->SampsPerSymbol();
  size_t spatial_streams_num = spatial_streams.n_elem;

  std::string raw_data_filename =
      kDlCheckFilePrefix + std::to_string(ofdm_ca_num) + "_bsant" +
      std::to_string(bs_ant_num) + "_ueant" + std::to_string(ue_num) + ".bin";

  Table<short> raw_data;
  Table<short> tx_data;
  raw_data.Calloc(num_data_syms * bs_ant_num, samps_per_symbol * 2,
                  Agora_memory::Alignment_t::kAlign64);
  tx_data.Calloc(num_data_syms * bs_ant_num, samps_per_symbol * 2,
                 Agora_memory::Alignment_t::kAlign64);

  size_t seek_size =
      sched_id * num_data_syms * bs_ant_num * samps_per_symbol * 2 * 2;
  ReadFromFileDl(raw_data_filename, raw_data, seek_size, samps_per_symbol, cfg);
  ReadFromFileDl(kTxFilename, tx_data, 0, samps_per_symbol, cfg);
  std::printf(
      "check_correctness_dl: bs ant %zu, ues %zu, spatial streams (last frame) "
      "%zu, "
      " dl syms %zu, ofdm %zu, samps per symb %zu. \n",
      bs_ant_num, ue_num, spatial_streams_num, num_data_syms, ofdm_ca_num,
      samps_per_symbol);

  unsigned int error_cnt = 0;
  float sum_diff = 0;
  for (size_t i = 0; i < num_data_syms; i++) {
    for (size_t ant = 0; ant < bs_ant_num; ant++) {
      sum_diff = 0;
      for (size_t sc = 0; sc < (samps_per_symbol * 2); sc++) {
        const size_t offset = (bs_ant_num * i) + ant;
        const float diff = std::fabs(
            (raw_data[offset][sc] - tx_data[offset][sc]) / kShrtFltConvFactor);
        sum_diff += diff;
        if (kDebugPrintDlCorr) {
          if (i == 0) {
            std::printf("dl symbol %zu ant %zu sc %zu, (%d, %d) diff: %.3f\n",
                        i, ant, sc / 2, raw_data[offset][sc],
                        tx_data[offset][sc], diff);
          }
        }
      }
      float avg_diff = sum_diff / samps_per_symbol;
      std::printf("dl symbol %zu, ant %zu, mean per-sample diff %.3f\n", i, ant,
                  avg_diff);
      if (avg_diff > 0.03) {
        error_cnt++;
      }
    }
  }
  raw_data.Free();
  tx_data.Free();

  return error_cnt;
}

static unsigned int CheckCorrectness(Config const* const cfg,
                                     arma::uvec spatial_streams,
                                     size_t sched_set_id) {
  unsigned int ul_error_count = 0;
  unsigned int dl_error_count = 0;
  ul_error_count = CheckCorrectnessUl(cfg, spatial_streams);
  std::printf("Uplink error count: %d\n", ul_error_count);
  dl_error_count = CheckCorrectnessDl(cfg, spatial_streams, sched_set_id);
  std::printf("Downlink error count: %d\n", dl_error_count);
  return ul_error_count + dl_error_count;
}

DEFINE_string(
    conf_file,
    TOSTRING(PROJECT_DIRECTORY) "/files/config/ci/tddconfig-sim-both.json",
    "Config filename");

int main(int argc, char* argv[]) {
  std::string conf_file;
  gflags::SetUsageMessage("conf_file : set the configuration filename");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  AGORA_LOG_INIT();

  // For backwards compatibility
  if (argc == 2) {
    conf_file = std::string(argv[1]);
    std::printf("User: Setting configuration filename to %s\n",
                conf_file.c_str());
  } else {
    conf_file = FLAGS_conf_file;
  }

  auto cfg = std::make_unique<Config>(conf_file.c_str());
  cfg->LoadTestVectors();
  auto mac_sched = std::make_unique<MacScheduler>(cfg.get());

  int ret;
  try {
    SignalHandler signal_handler;
    signal_handler.SetupSignalHandlers();
    auto agora_cli = std::make_unique<Agora>(cfg.get());
    agora_cli->flags_.enable_save_decode_data_to_file_ = true;
    agora_cli->flags_.enable_save_tx_data_to_file_ = true;
    agora_cli->Start();

    std::printf("Start correctness check\n");
    unsigned int error_count = 0;
    std::string test_name;

    auto ue_list = mac_sched->ScheduledUeList(cfg->FramesToTest() - 1, 0);
    size_t sched_set_id = 0;
    if (cfg->AdaptUes()) {
      sched_set_id = mac_sched->UeScheduleIndex(Utils::BitIndices2Int(ue_list));
    }
    if ((cfg->Frame().NumDLSyms() > 0) && (cfg->Frame().NumULSyms() > 0)) {
      test_name = "combined";
      error_count = CheckCorrectness(cfg.get(), ue_list, sched_set_id);
    } else if (cfg->Frame().NumDLSyms() > 0) {
      test_name = "downlink";
      error_count = CheckCorrectnessDl(cfg.get(), ue_list, sched_set_id);
    } else if (cfg->Frame().NumULSyms() > 0) {
      test_name = "uplink";
      error_count = CheckCorrectnessUl(cfg.get(), ue_list);
    } else {
      // Should never happen
      assert(false);
    }

    std::printf("======================\n");
    std::printf("%s test: \n", test_name.c_str());
    if (error_count == 0) {
      std::printf("Passed %s test!\n", test_name.c_str());
    } else {
      std::printf("Failed %s test! Error count: %d\n", test_name.c_str(),
                  error_count);
    }
    std::printf("======================\n\n");

    ret = EXIT_SUCCESS;
  } catch (SignalException& e) {
    std::cerr << "SignalException: " << e.what() << std::endl;
    ret = EXIT_FAILURE;
  }
  PrintCoreAssignmentSummary();
  gflags::ShutDownCommandLineFlags();
  AGORA_LOG_SHUTDOWN();
  return ret;
}
