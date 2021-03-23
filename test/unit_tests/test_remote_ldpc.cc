#include <gtest/gtest.h>
// For some reason, gtest include order matters
#include <algorithm>

#include "concurrentqueue.h"
#include "config.hpp"
#include "docoding.hpp"
#include "gettime.h"
#include "remote_ldpc.hpp"
#include "utils.h"

constexpr int kReqType = 2;
constexpr int kRpcPort = 31850;

static constexpr size_t kNumCodeBlocks = 2;
static constexpr size_t kBaseGraph = 1;
static constexpr bool kEnableEarlyTermination = true;
static constexpr size_t kNumFillerBits = 0;
static constexpr size_t kMaxDecoderIters = 20;
static constexpr size_t k5GNRNumPunctured = 2;

RemoteLDPC* remote;

void run_remote(Config* cfg, size_t tid, erpc::Nexus* nexus) {
  pin_to_core_with_offset(ThreadType::kWorker, cfg->remote_ldpc_core_offset,
                          tid);

  remote = new RemoteLDPC(cfg->LDPC_config, tid, nexus);
  remote->run_erpc_event_loop_forever();
}

class RemoteLDPCTest : public ::testing::Test {
 public:
  RemoteLDPCTest() {
    // Load config file
    cfg = new Config("data/tddconfig-sim-ul.json");
    cfg->genData();

    tid = 0;
    freq_ghz = measure_rdtsc_freq();

    // Prepare event queues for DoDecode object
    event_queue = moodycamel::ConcurrentQueue<Event_data>(2 * kNumIters);
    comp_queue = moodycamel::ConcurrentQueue<Event_data>(2 * kNumIters);
    ptok = new moodycamel::ProducerToken(comp_queue);

    // Prepare input and output buffer for DoDecode object
    const size_t task_buffer_symbol_num_ul =
        cfg->ul_data_symbol_num_perframe * TASK_BUFFER_FRAME_NUM;
    size_t mod_type = cfg->mod_type;
    demod_soft_buffer.malloc(task_buffer_symbol_num_ul,
                             mod_type * cfg->OFDM_DATA_NUM * cfg->UE_NUM, 64);
    size_t num_decoded_bytes =
        (cfg->LDPC_config.cbLen + 7) >> 3 * cfg->LDPC_config.nblocksInSymbol;
    decoded_buffer.calloc(task_buffer_symbol_num_ul,
                          num_decoded_bytes * cfg->UE_NUM, 64);

    // Prepare stats for DoDecode object
    stats = new Stats(cfg, kMaxStatBreakdown, freq_ghz);
    phy_stats = new PhyStats(cfg);

    // Create Dodecode object
    computeDecoding =
        new DoDecode(cfg, tid, freq_ghz, event_queue, comp_queue, ptok,
                     demod_soft_buffer, decoded_buffer, phy_stats, stats);

    // Prepare eRPC object for DoDecode object
    auto uri = cfg->bs_server_addr + ":" + std::to_string(kRpcPort);
    nexus = new erpc::Nexus(uri, 0, 0);
    rpc = new erpc::Rpc<erpc::CTransport>(
        nexus, static_cast<void*>(computeDecoding), tid, basic_sm_handler);
    rpc->retry_connect_on_invalid_rpc_id = true;

    // Run a different thread to launch a RemoteLDPC object
    uri = cfg->bs_server_addr + ":" + std::to_string(kRpcPort + 1);
    ldpc_nexus = new erpc::Nexus(uri, 0, 0);
    ldpc_nexus->register_req_func(kReqType, ldpc_req_handler);
    thread = new std::thread(run_remote, cfg, 0, ldpc_nexus);

    // Create session between local eRPC and RemoteLDPC,
    // and register local eRPC to DoDecode object
    session = rpc->create_session(uri, 0);
    rt_assert(session >= 0, "Connect failed!");
    while (!rpc->is_connected(session)) {
      rpc->run_event_loop_once();
    }
    static_cast<DoDecode*>(computeDecoding)->initialize_erpc(rpc, session);
  }

  ~RemoteLDPCTest() {
    remote->stop_loop();
    thread->join();
    rpc->destroy_session(session);
    delete remote;
    delete computeDecoding;
    delete rpc;
    delete nexus;
    delete ldpc_nexus;
  }

  // Test whether the local server receives responses from the remote LDPC
  void test_connectivity() {
    FastRand fast_rand;
    size_t start_tsc = rdtsc();

    // For each round, randomly generate frame_ud, symbol_id,
    // and cb_id, and launch a DoDecode task
    for (size_t i = 0; i < kNumIters; i++) {
      uint32_t frame_id = fast_rand.next_u32();
      size_t symbol_id =
          (fast_rand.next_u32() % cfg->ul_data_symbol_num_perframe);
      size_t cb_id = (fast_rand.next_u32() %
                      (cfg->UE_NUM * cfg->LDPC_config.nblocksInSymbol));
      computeDecoding->launch(
          gen_tag_t::frm_sym_cb(frame_id, symbol_id, cb_id)._tag);
      rpc->run_event_loop(10);
    }
    double ms = cycles_to_ms(rdtsc() - start_tsc, freq_ghz);

    // Wait for incoming responses
    rpc->run_event_loop(100);

    // Assert that # tasks launched = # responses
    ASSERT_EQ(computeDecoding->get_num_responses(), kNumIters);
  }

  // Test whether the remote LDPC reports the correct decoded results
  void test_correctness() {
    // Prepare Data
    int8_t* input[kNumCodeBlocks];
    int8_t* parity[kNumCodeBlocks];
    int8_t* encoded[kNumCodeBlocks];
    uint8_t* decoded[kNumCodeBlocks];

    std::vector<size_t> zc_vec = {72};
    std::sort(zc_vec.begin(), zc_vec.end());

    for (const size_t& zc : zc_vec) {
      if (zc > ZC_MAX) {
        fprintf(stderr, "Zc value %zu not supported by avx2enc. Skipping.\n",
                zc);
        continue;
      }
      const size_t num_input_bits = ldpc_num_input_bits(kBaseGraph, zc);
      const size_t num_parity_bits = ldpc_max_num_parity_bits(kBaseGraph, zc);
      const size_t num_encoded_bits = ldpc_max_num_encoded_bits(kBaseGraph, zc);

      for (size_t i = 0; i < kNumCodeBlocks; i++) {
        input[i] = new int8_t[ldpc_encoding_input_buf_size(kBaseGraph, zc)];
        parity[i] = new int8_t[ldpc_encoding_parity_buf_size(kBaseGraph, zc)];
        encoded[i] = new int8_t[ldpc_encoding_encoded_buf_size(kBaseGraph, zc)];
        decoded[i] =
            new uint8_t[ldpc_encoding_encoded_buf_size(kBaseGraph, zc)];
      }

      // Randomly generate input
      srand(time(NULL));
      for (size_t n = 0; n < kNumCodeBlocks; n++) {
        for (size_t i = 0; i < bits_to_bytes(num_input_bits); i++)
          input[n][i] = (int8_t)rand();
      }

      const size_t encoding_start_tsc = rdtsc();
      for (size_t n = 0; n < kNumCodeBlocks; n++) {
        ldpc_encode_helper(kBaseGraph, zc, ldpc_max_num_rows(kBaseGraph),
                           encoded[n], parity[n], input[n]);
      }

      // For decoding, generate log-likelihood ratios, one byte per input bit
      int8_t* llrs[kNumCodeBlocks];
      for (size_t n = 0; n < kNumCodeBlocks; n++) {
        llrs[n] = reinterpret_cast<int8_t*>(memalign(32, num_encoded_bits));
        for (size_t i = 0; i < num_encoded_bits; i++) {
          uint8_t bit_i = (encoded[n][i / 8] >> (i % 8)) & 1;
          llrs[n][i] = (bit_i == 1 ? -127 : 127);
        }
      }

      // Decoder setup
      for (size_t n = 0; n < kNumCodeBlocks; n++) {
        size_t symbol_offset = cfg->get_total_data_symbol_idx_ul(n, 0);
        size_t input_offset = cfg->get_ldpc_input_offset(0);
        size_t llr_buffer_offset = input_offset * cfg->mod_type;
        memcpy(demod_soft_buffer[symbol_offset] + llr_buffer_offset, llrs[n],
               num_encoded_bits);
      }

      // Decoding
      for (size_t n = 0; n < kNumCodeBlocks; n++) {
        computeDecoding->launch(gen_tag_t::frm_sym_cb(n, 0, 0)._tag);
      }

      // Wait for incoming responses
      rpc->run_event_loop(100);

      for (size_t n = 0; n < kNumCodeBlocks; n++) {
        size_t symbol_offset = cfg->get_total_data_symbol_idx_ul(n, 0);
        size_t output_offset = cfg->get_ldpc_output_offset(0);
        uint8_t* out_buf =
            static_cast<uint8_t*>(decoded_buffer[symbol_offset]) +
            output_offset;
        memcpy(decoded[n], out_buf,
               ldpc_encoding_input_buf_size(kBaseGraph, zc));
      }

      // Check for errors
      size_t err_cnt = 0;
      for (size_t n = 0; n < kNumCodeBlocks; n++) {
        uint8_t* input_buffer = (uint8_t*)input[n];
        uint8_t* output_buffer = decoded[n];
        for (size_t i = 0; i < bits_to_bytes(num_input_bits); i++) {
          // printf("input: %i, output: %i\n", input_buffer[i],
          // output_buffer[i]);
          uint8_t error = input_buffer[i] ^ output_buffer[i];
          for (size_t j = 0; j < 8; j++) {
            if (i * 8 + j >= num_input_bits) {
              continue;  // Don't compare beyond end of input bits
            }
            err_cnt += error & 1;
            error >>= 1;
          }
        }
      }

      printf("Zc = %zu. Bit errors = %zu, BER = %.3f\n", zc, err_cnt,
             err_cnt * 1.0 / (kNumCodeBlocks * num_input_bits));

      ASSERT_EQ(err_cnt, 0);

      for (size_t i = 0; i < kNumCodeBlocks; i++) {
        delete[] input[i];
        delete[] parity[i];
        delete[] encoded[i];
        delete[] decoded[i];
        free(llrs[i]);
      }
    }
  }

 private:
  size_t kNumIters = 5;
  Config* cfg;
  moodycamel::ConcurrentQueue<Event_data> event_queue;
  moodycamel::ConcurrentQueue<Event_data> comp_queue;
  moodycamel::ProducerToken* ptok;
  Table<int8_t> demod_soft_buffer;
  Table<uint8_t> decoded_buffer;
  Stats* stats;
  PhyStats* phy_stats;
  DoDecode* compute_decoding;
  erpc::Nexus* nexus;
  erpc::Nexus* ldpc_nexus;
  erpc::Rpc<erpc::CTransport>* rpc;
  std::thread* thread;
  int session;
  int tid;
  double freq_ghz;
};

TEST_F(RemoteLDPCTest, create) {}

TEST_F(RemoteLDPCTest, test_connectivity) { test_connectivity(); }

TEST_F(RemoteLDPCTest, test_correctness) { test_correctness(); }

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}