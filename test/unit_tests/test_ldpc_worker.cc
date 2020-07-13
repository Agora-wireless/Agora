#include <gtest/gtest.h>
// For some reason, gtest include order matters
#include "concurrentqueue.h"
#include "config.hpp"
#include "docoding.hpp"
#include "gettime.h"
#include "ldpc_worker.hpp"
#include "rpc.h"
#include "utils.h"

#include <algorithm>

constexpr int kReqType = 2;
constexpr int kRpcPort = 31850;
size_t count = 0;

static constexpr size_t kNumCodeBlocks = 2;
static constexpr size_t kBaseGraph = 1;
static constexpr bool kEnableEarlyTermination = true;
static constexpr size_t kNumFillerBits = 0;
static constexpr size_t kMaxDecoderIters = 20;
static constexpr size_t k5GNRNumPunctured = 2;

LDPCWorker* worker;

void basic_sm_handler_test(int session_num, erpc::SmEventType sm_event_type,
    erpc::SmErrType sm_err_type, void* _context)
{
    printf("Connected session: %d\n", session_num);
}

void ldpc_req_handler_test(erpc::ReqHandle* req_handle, void* _context)
{
    count++;
    auto* worker = static_cast<LDPCWorker*>(_context);
    auto* in_buf = reinterpret_cast<int8_t*>(req_handle->get_req_msgbuf()->buf);

    req_handle->dyn_resp_msgbuf
        = worker->rpc->alloc_msg_buffer(worker->decoded_bits);
    auto* out_buf = reinterpret_cast<uint8_t*>(req_handle->dyn_resp_msgbuf.buf);
    worker->decode(in_buf, out_buf);

    worker->rpc->enqueue_response(req_handle, &req_handle->dyn_resp_msgbuf);
}

void run_worker(Config* cfg, size_t tid, erpc::Nexus* nexus)
{
    pin_to_core_with_offset(
        ThreadType::kWorker, cfg->ldpc_worker_core_offset, tid);

    worker = new LDPCWorker(cfg->LDPC_config, tid, nexus);
    worker->run_erpc_event_loop_forever();
}

TEST(TestLDPCWorker, Connectivity)
{
    static constexpr size_t kNumIters = 1000;
    auto* cfg = new Config("data/tddconfig-sim-ul.json");
    cfg->genData();

    int tid = 0;
    double freq_ghz = measure_rdtsc_freq();

    auto event_queue = moodycamel::ConcurrentQueue<Event_data>(2 * kNumIters);
    auto comp_queue = moodycamel::ConcurrentQueue<Event_data>(2 * kNumIters);
    auto ptok = new moodycamel::ProducerToken(comp_queue);

    Table<int8_t> demod_soft_buffer;
    Table<uint8_t> decoded_buffer;
    const size_t task_buffer_symbol_num_ul
        = cfg->ul_data_symbol_num_perframe * TASK_BUFFER_FRAME_NUM;
    size_t mod_type = cfg->mod_type;
    demod_soft_buffer.malloc(task_buffer_symbol_num_ul,
        mod_type * cfg->OFDM_DATA_NUM * cfg->UE_NUM, 64);
    size_t num_decoded_bytes
        = (cfg->LDPC_config.cbLen + 7) >> 3 * cfg->LDPC_config.nblocksInSymbol;
    decoded_buffer.calloc(
        task_buffer_symbol_num_ul, num_decoded_bytes * cfg->UE_NUM, 64);

    auto stats = new Stats(cfg, kMaxStatBreakdown, freq_ghz);
    auto phy_stats = new PhyStats(cfg);

    auto computeDecoding = new DoDecode(cfg, tid, freq_ghz, event_queue,
        comp_queue, ptok, demod_soft_buffer, decoded_buffer, phy_stats, stats);

    auto uri = cfg->server_addr + ":" + std::to_string(kRpcPort);
    auto* nexus = new erpc::Nexus(uri, 0, 0);
    auto* rpc = new erpc::Rpc<erpc::CTransport>(
        nexus, static_cast<void*>(computeDecoding), tid, basic_sm_handler_test);
    rpc->retry_connect_on_invalid_rpc_id = true;

    uri = cfg->server_addr + ":" + std::to_string(kRpcPort + 1);
    auto* ldpc_nexus = new erpc::Nexus(uri, 0, 0);
    ldpc_nexus->register_req_func(kReqType, ldpc_req_handler_test);
    auto* thread = new std::thread(run_worker, cfg, 0, ldpc_nexus);

    int session = rpc->create_session(uri, 0);
    rt_assert(session >= 0, "Connect failed!");
    while (!rpc->is_connected(session)) {
        rpc->run_event_loop_once();
    }
    static_cast<DoDecode*>(computeDecoding)->initialize_erpc(rpc, session);

    FastRand fast_rand;
    size_t start_tsc = rdtsc();
    for (size_t i = 0; i < kNumIters; i++) {
        uint32_t frame_id = fast_rand.next_u32();
        size_t symbol_id
            = (fast_rand.next_u32() % cfg->ul_data_symbol_num_perframe);
        size_t cb_id = (fast_rand.next_u32()
            % (cfg->UE_NUM * cfg->LDPC_config.nblocksInSymbol));
        computeDecoding->launch(
            gen_tag_t::frm_sym_cb(frame_id, symbol_id, cb_id)._tag);
        rpc->run_event_loop(10);
    }
    double ms = cycles_to_ms(rdtsc() - start_tsc, freq_ghz);

    rpc->run_event_loop(5000);

    ASSERT_EQ(count, kNumIters);

    worker->stop_loop();
    thread->join();
    rpc->destroy_session(session);
    delete worker;
    delete computeDecoding;
    delete rpc;
    delete nexus;
    delete ldpc_nexus;

    printf("Time per zeroforcing iteration = %.4f ms\n", ms / kNumIters);
}

TEST(TestLDPCWorker, Corretness)
{
    static constexpr size_t kNumIters = 1000;
    auto* cfg = new Config("data/tddconfig-sim-ul.json");
    cfg->genData();

    int tid = 0;
    double freq_ghz = measure_rdtsc_freq();

    auto event_queue = moodycamel::ConcurrentQueue<Event_data>(2 * kNumIters);
    auto comp_queue = moodycamel::ConcurrentQueue<Event_data>(2 * kNumIters);
    auto ptok = new moodycamel::ProducerToken(comp_queue);

    Table<int8_t> demod_soft_buffer;
    Table<uint8_t> decoded_buffer;
    const size_t task_buffer_symbol_num_ul
        = cfg->ul_data_symbol_num_perframe * TASK_BUFFER_FRAME_NUM;
    size_t mod_type = cfg->mod_type;
    demod_soft_buffer.malloc(task_buffer_symbol_num_ul,
        mod_type * cfg->OFDM_DATA_NUM * cfg->UE_NUM, 64);
    size_t num_decoded_bytes
        = (cfg->LDPC_config.cbLen + 7) >> 3 * cfg->LDPC_config.nblocksInSymbol;
    decoded_buffer.calloc(
        task_buffer_symbol_num_ul, num_decoded_bytes * cfg->UE_NUM, 64);

    auto stats = new Stats(cfg, kMaxStatBreakdown, freq_ghz);
    auto phy_stats = new PhyStats(cfg);

    auto computeDecoding = new DoDecode(cfg, tid, freq_ghz, event_queue,
        comp_queue, ptok, demod_soft_buffer, decoded_buffer, phy_stats, stats);

    auto uri = cfg->server_addr + ":" + std::to_string(kRpcPort);
    auto* nexus = new erpc::Nexus(uri, 0, 0);
    auto* rpc = new erpc::Rpc<erpc::CTransport>(
        nexus, static_cast<void*>(computeDecoding), tid, basic_sm_handler_test);
    rpc->retry_connect_on_invalid_rpc_id = true;

    uri = cfg->server_addr + ":" + std::to_string(kRpcPort + 1);
    auto* ldpc_nexus = new erpc::Nexus(uri, 0, 0);
    ldpc_nexus->register_req_func(kReqType, ldpc_req_handler_test);
    auto* thread = new std::thread(run_worker, cfg, 0, ldpc_nexus);

    int session = rpc->create_session(uri, 0);
    rt_assert(session >= 0, "Connect failed!");
    while (!rpc->is_connected(session)) {
        rpc->run_event_loop_once();
    }
    static_cast<DoDecode*>(computeDecoding)->initialize_erpc(rpc, session);

    // Prepare Data
    int8_t* input[kNumCodeBlocks];
    int8_t* parity[kNumCodeBlocks];
    int8_t* encoded[kNumCodeBlocks];
    uint8_t* decoded[kNumCodeBlocks];

    // std::vector<size_t> zc_vec = { 2, 4, 8, 16, 32, 64, 128, 256, 3, 6, 12, 24,
    //     48, 96, 192, 384, 5, 10, 20, 40, 80, 160, 320, 7, 14, 28, 56, 112, 224,
    //     9, 18, 36, 72, 144, 288, 11, 22, 44, 88, 176, 352, 13, 26, 52, 104, 208,
    //     15, 30, 60, 120, 240 };
    std::vector<size_t> zc_vec = { 72 };
    std::sort(zc_vec.begin(), zc_vec.end());

    for (const size_t& zc : zc_vec) {
        if (zc > avx2enc::ZC_MAX) {
            fprintf(stderr,
                "Zc value %zu not supported by avx2enc. Skipping.\n", zc);
            continue;
        }
        const size_t num_input_bits = ldpc_num_input_bits(kBaseGraph, zc);
        const size_t num_parity_bits = ldpc_num_parity_bits(kBaseGraph, zc);
        const size_t num_encoded_bits = ldpc_num_encoded_bits(kBaseGraph, zc);

        for (size_t i = 0; i < kNumCodeBlocks; i++) {
            input[i] = new int8_t[ldpc_encoding_input_buf_size(kBaseGraph, zc)];
            parity[i]
                = new int8_t[ldpc_encoding_parity_buf_size(kBaseGraph, zc)];
            encoded[i]
                = new int8_t[ldpc_encoding_encoded_buf_size(kBaseGraph, zc)];
            decoded[i]
                = new uint8_t[ldpc_encoding_encoded_buf_size(kBaseGraph, zc)];
        }

        // Randomly generate input
        srand(time(NULL));
        for (size_t n = 0; n < kNumCodeBlocks; n++) {
            for (size_t i = 0; i < bits_to_bytes(num_input_bits); i++)
                input[n][i] = (int8_t)rand();
        }

        const size_t encoding_start_tsc = rdtsc();
        for (size_t n = 0; n < kNumCodeBlocks; n++) {
            ldpc_encode_helper(kBaseGraph, zc, encoded[n], parity[n], input[n]);
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
            memcpy(demod_soft_buffer[symbol_offset] + llr_buffer_offset,
                llrs[n], num_encoded_bits);
        }

        // Decoding
        for (size_t n = 0; n < kNumCodeBlocks; n++) {
            computeDecoding->launch(gen_tag_t::frm_sym_cb(n, 0, 0)._tag);
        }

        rpc->run_event_loop(100);

        for (size_t n = 0; n < kNumCodeBlocks; n++) {
            size_t symbol_offset = cfg->get_total_data_symbol_idx_ul(n, 0);
            size_t output_offset = cfg->get_ldpc_output_offset(0);
            uint8_t* out_buf
                = static_cast<uint8_t*>(decoded_buffer[symbol_offset])
                + output_offset;
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
                        continue; // Don't compare beyond end of input bits
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

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}