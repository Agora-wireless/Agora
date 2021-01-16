#include "docoding.hpp"
#include "concurrent_queue_wrapper.hpp"
#include "encoder.hpp"
#include "phy_ldpc_decoder_5gnr.h"

static constexpr bool kPrintEncodedData = false;
static constexpr bool kPrintLLRData = false;
static constexpr bool kPrintDecodedData = false;

DoEncode::DoEncode(Config* in_config, int in_tid,
    Table<int8_t>& in_raw_data_buffer, Table<int8_t>& in_encoded_buffer,
    Stats* in_stats_manager)
    : Doer(in_config, in_tid)
    , raw_data_buffer_(in_raw_data_buffer)
    , encoded_buffer_(in_encoded_buffer)
{
    duration_stat
        = in_stats_manager->GetDurationStat(DoerType::kEncode, in_tid);
    parity_buffer = static_cast<int8_t*>(
        Agora_memory::padded_aligned_alloc(Agora_memory::Alignment_t::k64Align,
            ldpc_encoding_parity_buf_size(cfg->ldpc_config().base_graph(),
                cfg->ldpc_config().expansion_factor())));
    assert(parity_buffer != nullptr);
    encoded_buffer_temp = static_cast<int8_t*>(
        Agora_memory::padded_aligned_alloc(Agora_memory::Alignment_t::k64Align,
            ldpc_encoding_encoded_buf_size(cfg->ldpc_config().base_graph(),
                cfg->ldpc_config().expansion_factor())));
    assert(encoded_buffer_temp != nullptr);
}

DoEncode::~DoEncode()
{
    std::free(parity_buffer);
    std::free(encoded_buffer_temp);
}

Event_data DoEncode::launch(size_t tag)
{
    const LDPCconfig& LDPC_config = cfg->ldpc_config();
    size_t frame_id = gen_tag_t(tag).frame_id;
    size_t symbol_id = gen_tag_t(tag).symbol_id;
    size_t cb_id = gen_tag_t(tag).cb_id;
    size_t cur_cb_id = cb_id % cfg->ldpc_config().num_blocks_in_symbol();
    size_t ue_id = cb_id / cfg->ldpc_config().num_blocks_in_symbol();
    if (kDebugPrintInTask) {
        std::printf(
            "In doEncode thread %d: frame: %zu, symbol: %zu, code block %zu, "
            "ue_id: %zu\n",
            tid, frame_id, symbol_id, cur_cb_id, ue_id);
    }

    size_t start_tsc = worker_rdtsc();

    size_t symbol_idx_dl = cfg->GetDLSymbolIdx(frame_id, symbol_id);
    int8_t* input_ptr
        = cfg->GetInfoBits(raw_data_buffer_, symbol_idx_dl, ue_id, cur_cb_id);

    ldpc_encode_helper(LDPC_config.base_graph(), LDPC_config.expansion_factor(),
        LDPC_config.num_rows(), encoded_buffer_temp, parity_buffer, input_ptr);
    int8_t* final_output_ptr = cfg->GetEncodedBuf(
        encoded_buffer_, frame_id, symbol_idx_dl, ue_id, cur_cb_id);
    adapt_bits_for_mod(reinterpret_cast<uint8_t*>(encoded_buffer_temp),
        reinterpret_cast<uint8_t*>(final_output_ptr),
        bits_to_bytes(LDPC_config.num_cb_codew_len()), cfg->mod_order_bits());

    // std::printf("Encoded data\n");
    // int num_mod = LDPC_config.num_cb_codew_len() / cfg->mod_order_bits();
    // for(int i = 0; i < num_mod; i++) {
    //     std::printf("%u ", *(final_output_ptr + i));
    // }
    // std::printf("\n");

    size_t duration = worker_rdtsc() - start_tsc;
    duration_stat->task_duration[0] += duration;
    duration_stat->task_count++;
    if (cycles_to_us(duration, cfg->freq_ghz()) > 500) {
        std::printf("Thread %d Encode takes %.2f\n", tid,
            cycles_to_us(duration, cfg->freq_ghz()));
    }

    return Event_data(EventType::kEncode, tag);
}

DoDecode::DoDecode(Config* in_config, int in_tid,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffers,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t>& decoded_buffers,
    PhyStats* in_phy_stats, Stats* in_stats_manager)
    : Doer(in_config, in_tid)
    , demod_buffers_(demod_buffers)
    , decoded_buffers_(decoded_buffers)
    , phy_stats(in_phy_stats)
{
    duration_stat
        = in_stats_manager->GetDurationStat(DoerType::kDecode, in_tid);
    resp_var_nodes = static_cast<int16_t*>(Agora_memory::padded_aligned_alloc(
        Agora_memory::Alignment_t::k64Align, 1024 * 1024 * sizeof(int16_t)));
}

DoDecode::~DoDecode(void) { std::free(resp_var_nodes); }

Event_data DoDecode::launch(size_t tag)
{
    const LDPCconfig& LDPC_config = cfg->ldpc_config();
    const size_t frame_id = gen_tag_t(tag).frame_id;
    const size_t symbol_idx_ul = gen_tag_t(tag).symbol_id;
    const size_t cb_id = gen_tag_t(tag).cb_id;
    const size_t symbol_offset
        = cfg->GetTotalDataSymbolIdxUl(frame_id, symbol_idx_ul);
    const size_t cur_cb_id
        = (cb_id % cfg->ldpc_config().num_blocks_in_symbol());
    const size_t ue_id = (cb_id / cfg->ldpc_config().num_blocks_in_symbol());
    const size_t frame_slot = (frame_id % kFrameWnd);
    if (kDebugPrintInTask == true) {
        std::printf(
            "In doDecode thread %d: frame: %zu, symbol: %zu, code block: "
            "%zu, ue: %zu offset %zu\n",
            tid, frame_id, symbol_idx_ul, cur_cb_id, ue_id, symbol_offset);
    }

    size_t start_tsc = worker_rdtsc();

    struct bblib_ldpc_decoder_5gnr_request ldpc_decoder_5gnr_request {
    };
    struct bblib_ldpc_decoder_5gnr_response ldpc_decoder_5gnr_response {
    };

    // Decoder setup
    int16_t numFillerBits = 0;
    int16_t numChannelLlrs = LDPC_config.num_cb_codew_len();

    ldpc_decoder_5gnr_request.numChannelLlrs = numChannelLlrs;
    ldpc_decoder_5gnr_request.numFillerBits = numFillerBits;
    ldpc_decoder_5gnr_request.maxIterations = LDPC_config.max_decoder_iter();
    ldpc_decoder_5gnr_request.enableEarlyTermination
        = LDPC_config.early_termination();
    ldpc_decoder_5gnr_request.Zc = LDPC_config.expansion_factor();
    ldpc_decoder_5gnr_request.baseGraph = LDPC_config.base_graph();
    ldpc_decoder_5gnr_request.nRows = LDPC_config.num_rows();

    int numMsgBits = LDPC_config.num_cb_len() - numFillerBits;
    ldpc_decoder_5gnr_response.numMsgBits = numMsgBits;
    ldpc_decoder_5gnr_response.varNodes = resp_var_nodes;

    int8_t* llr_buffer_ptr = demod_buffers_[frame_slot][symbol_idx_ul][ue_id]
        + (cfg->mod_order_bits()
              * (LDPC_config.num_cb_codew_len() * cur_cb_id));

    uint8_t* decoded_buffer_ptr
        = decoded_buffers_[frame_slot][symbol_idx_ul][ue_id]
        + (cur_cb_id * roundup<64>(cfg->num_bytes_per_cb()));

    ldpc_decoder_5gnr_request.varNodes = llr_buffer_ptr;
    ldpc_decoder_5gnr_response.compactedMessageBytes = decoded_buffer_ptr;

    size_t start_tsc1 = worker_rdtsc();
    duration_stat->task_duration[1] += start_tsc1 - start_tsc;

    bblib_ldpc_decoder_5gnr(
        &ldpc_decoder_5gnr_request, &ldpc_decoder_5gnr_response);

    size_t start_tsc2 = worker_rdtsc();
    duration_stat->task_duration[2] += start_tsc2 - start_tsc1;

    if (kPrintLLRData) {
        std::printf("LLR data, symbol_offset: %zu\n", symbol_offset);
        for (size_t i = 0; i < LDPC_config.num_cb_codew_len(); i++) {
            std::printf("%d ", *(llr_buffer_ptr + i));
        }
        std::printf("\n");
    }

    if (kPrintDecodedData) {
        std::printf("Decoded data\n");
        for (size_t i = 0; i < (LDPC_config.num_cb_len() >> 3); i++) {
            std::printf("%u ", *(decoded_buffer_ptr + i));
        }
        std::printf("\n");
    }

    if ((kEnableMac == false) && (kPrintPhyStats == true)
        && (symbol_idx_ul >= cfg->frame().client_ul_pilot_symbols())) {
        phy_stats->update_decoded_bits(
            ue_id, symbol_offset, cfg->num_bytes_per_cb() * 8);
        phy_stats->increment_decoded_blocks(ue_id, symbol_offset);
        size_t block_error(0);
        for (size_t i = 0; i < cfg->num_bytes_per_cb(); i++) {
            uint8_t rx_byte = decoded_buffer_ptr[i];
            uint8_t tx_byte = static_cast<uint8_t>(cfg->GetInfoBits(
                cfg->ul_bits(), symbol_idx_ul, ue_id, cur_cb_id)[i]);
            phy_stats->update_bit_errors(
                ue_id, symbol_offset, tx_byte, rx_byte);
            if (rx_byte != tx_byte) {
                block_error++;
            }
        }
        phy_stats->update_block_errors(ue_id, symbol_offset, block_error);
    }

    size_t duration = worker_rdtsc() - start_tsc;
    duration_stat->task_duration[0] += duration;
    duration_stat->task_count++;
    if (cycles_to_us(duration, cfg->freq_ghz()) > 500) {
        std::printf("Thread %d Decode takes %.2f\n", tid,
            cycles_to_us(duration, cfg->freq_ghz()));
    }

    return Event_data(EventType::kDecode, tag);
}
