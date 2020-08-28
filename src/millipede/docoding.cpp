/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "docoding.hpp"
#include "concurrent_queue_wrapper.hpp"
#include "encoder.hpp"
#include "phy_ldpc_decoder_5gnr.h"
#include "udp_server.h"
#include "utils_ldpc.hpp"
#include <malloc.h>

static constexpr bool kPrintEncodedData = false;
static constexpr bool kPrintLLRData = false;
static constexpr bool kPrintDecodedData = false;

/// TODO: FIXME: what size?
static constexpr size_t kRxBufSize = 64 * 1024 * 1024;

DoEncode::DoEncode(Config* in_config, int in_tid, double freq_ghz,
    moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
    moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
    moodycamel::ProducerToken* worker_producer_token,
    Table<int8_t>& in_raw_data_buffer, Table<int8_t>& in_encoded_buffer,
    Stats* in_stats_manager)
    : Doer(in_config, in_tid, freq_ghz, in_task_queue, complete_task_queue,
          worker_producer_token)
    , raw_data_buffer_(in_raw_data_buffer)
    , encoded_buffer_(in_encoded_buffer)
{
    duration_stat
        = in_stats_manager->get_duration_stat(DoerType::kEncode, in_tid);
    parity_buffer = (int8_t*)memalign(64,
        ldpc_encoding_parity_buf_size(
            cfg->LDPC_config.Bg, cfg->LDPC_config.Zc));
    encoded_buffer_temp = (int8_t*)memalign(64,
        ldpc_encoding_encoded_buf_size(
            cfg->LDPC_config.Bg, cfg->LDPC_config.Zc));
}

DoEncode::~DoEncode()
{
    free(parity_buffer);
    free(encoded_buffer_temp);
}

Event_data DoEncode::launch(size_t tag)
{
    LDPCconfig LDPC_config = cfg->LDPC_config;
    size_t frame_id = gen_tag_t(tag).frame_id;
    size_t symbol_id = gen_tag_t(tag).symbol_id;
    size_t cb_id = gen_tag_t(tag).cb_id;
    size_t cur_cb_id = cb_id % cfg->LDPC_config.nblocksInSymbol;
    size_t ue_id = cb_id / cfg->LDPC_config.nblocksInSymbol;
    if (kDebugPrintInTask) {
        printf(
            "In doEncode thread %d: frame: %zu, symbol: %zu, code block %zu\n",
            tid, frame_id, symbol_id, cur_cb_id);
    }

    size_t start_tsc = worker_rdtsc();

    size_t symbol_id_in_buffer = symbol_id - cfg->dl_data_symbol_start;
    int8_t* input_ptr = cfg->get_info_bits(
        raw_data_buffer_, symbol_id_in_buffer, ue_id, cur_cb_id);
    int8_t* output_ptr = encoded_buffer_temp;

    ldpc_encode_helper(LDPC_config.Bg, LDPC_config.Zc, LDPC_config.nRows,
        output_ptr, parity_buffer, input_ptr);
    int8_t* final_output_ptr = cfg->get_encoded_buf(
        encoded_buffer_, frame_id, symbol_id, ue_id, cur_cb_id);
    adapt_bits_for_mod(output_ptr, final_output_ptr,
        (LDPC_config.cbCodewLen + 7) >> 3, cfg->mod_type);

    // printf("Encoded data\n");
    // int mod_type = cfg->mod_type;
    // int num_mod = LDPC_config.cbCodewLen / mod_type;
    // for(int i = 0; i < num_mod; i++) {
    //     printf("%u ", *(final_output_ptr + i));
    // }
    // printf("\n");

    size_t duration = worker_rdtsc() - start_tsc;
    duration_stat->task_duration[0] += duration;
    duration_stat->task_count++;
    if (cycles_to_us(duration, freq_ghz) > 500) {
        printf("Thread %d Encode takes %.2f\n", tid,
            cycles_to_us(duration, freq_ghz));
    }

    return Event_data(EventType::kEncode, tag);
}

DoDecode::DoDecode(Config* in_config, int in_tid, double freq_ghz,
    moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
    moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
    moodycamel::ProducerToken* worker_producer_token,
    Table<int8_t>& in_demod_buffer, Table<uint8_t>& in_decoded_buffer,
    //Table<int>& in_decoded_bits_count, Table<int>& in_error_bits_count,
    PhyStats* in_phy_stats, Stats* in_stats_manager)
    : Doer(in_config, in_tid, freq_ghz, in_task_queue, complete_task_queue,
          worker_producer_token)
    , llr_buffer_(in_demod_buffer)
    , decoded_buffer_(in_decoded_buffer)
    , phy_stats(in_phy_stats)
{
    duration_stat
        = in_stats_manager->get_duration_stat(DoerType::kDecode, in_tid);
    resp_var_nodes = (int16_t*)memalign(64, 1024 * 1024 * sizeof(int16_t));
}

DoDecode::~DoDecode()
{
    free(resp_var_nodes);
    // NOTE: currently, the remote_ldpc_stub_ pointer may be shared with other
    // DoDecode instances. Therefore, we cannot free it here, but rather
    // we can only free it safely from within the worker thread.
}

Event_data DoDecode::launch(size_t tag)
{
    LDPCconfig LDPC_config = cfg->LDPC_config;
    size_t frame_id = gen_tag_t(tag).frame_id;
    size_t symbol_id = gen_tag_t(tag).symbol_id;
    size_t cb_id = gen_tag_t(tag).cb_id;
    size_t symbol_offset
        = cfg->get_total_data_symbol_idx_ul(frame_id, symbol_id);
    size_t cur_cb_id = cb_id % cfg->LDPC_config.nblocksInSymbol;
    size_t ue_id = cb_id / cfg->LDPC_config.nblocksInSymbol;
    if (kDebugPrintInTask) {
        printf("In doDecode thread %d: frame: %zu, symbol: %zu, code block: "
               "%zu ue:  "
               "%zu\n",
            tid, frame_id, symbol_id, cur_cb_id, ue_id);
    }

    size_t start_tsc = worker_rdtsc();

    if (kUseRemote) {
        size_t input_offset = cfg->get_ldpc_input_offset(cb_id);
        size_t llr_buffer_offset = input_offset * cfg->mod_type;
        size_t output_offset = cfg->get_ldpc_output_offset(cb_id);

        auto* send_buf = reinterpret_cast<char*>(
            (int8_t*)llr_buffer_[symbol_offset] + llr_buffer_offset);
        auto* decode_context
            = new DecodeContext(symbol_offset, output_offset, tag, tid, this);

        size_t data_bytes
            = ldpc_max_num_encoded_bits(LDPC_config.Bg, LDPC_config.Zc);

        // TODO: pre-allocate a pool of request buffers to avoid this
        //       temporary allocation on the critical data path.
        size_t msg_len = DecodeMsg::size_without_data() + data_bytes;
        uint8_t* new_buf = new uint8_t[msg_len];
        auto* request = reinterpret_cast<DecodeMsg*>(new_buf);
        request->context = decode_context;
        memcpy(request->data, send_buf, data_bytes);

        // while (rpc_context_->vec_req_msgbuf.size() == 0) {
        //     // printf("Docoding: Running RPC event loop, rpc is %p\n", rpc);
        //     // rt_assert(rpc != nullptr, "RPC is null");
        //     // printf("Ran out of request message buffers!\n");
        //     rpc_context_->rpc->run_event_loop_once();
        // }
        // auto* req_msgbuf = rpc_context_->vec_req_msgbuf.back();
        // rpc_context_->vec_req_msgbuf.pop_back();

        printf("Docoding: Issuing request %zu, context: %p\n",
            remote_ldpc_stub_->num_requests_issued, request->context);
        remote_ldpc_stub_->udp_client.send(cfg->remote_ldpc_addr,
            cfg->remote_ldpc_base_port + tid, new_buf, msg_len);
        remote_ldpc_stub_->num_requests_issued++;
        delete[] new_buf; // TODO: remove this once we use request buf pools
    } else {
        struct bblib_ldpc_decoder_5gnr_request ldpc_decoder_5gnr_request {
        };
        struct bblib_ldpc_decoder_5gnr_response ldpc_decoder_5gnr_response {
        };

        // Decoder setup
        int16_t numFillerBits = 0;
        int16_t numChannelLlrs = LDPC_config.cbCodewLen;

        ldpc_decoder_5gnr_request.numChannelLlrs = numChannelLlrs;
        ldpc_decoder_5gnr_request.numFillerBits = numFillerBits;
        ldpc_decoder_5gnr_request.maxIterations = LDPC_config.decoderIter;
        ldpc_decoder_5gnr_request.enableEarlyTermination
            = LDPC_config.earlyTermination;
        ldpc_decoder_5gnr_request.Zc = LDPC_config.Zc;
        ldpc_decoder_5gnr_request.baseGraph = LDPC_config.Bg;
        ldpc_decoder_5gnr_request.nRows = LDPC_config.nRows;

        int numMsgBits = LDPC_config.cbLen - numFillerBits;
        ldpc_decoder_5gnr_response.numMsgBits = numMsgBits;
        ldpc_decoder_5gnr_response.varNodes = resp_var_nodes;

        auto* llr_buffer_ptr = cfg->get_demod_buf(llr_buffer_, frame_id,
            symbol_id, ue_id, LDPC_config.cbCodewLen * cur_cb_id);
        auto* decoded_buffer_ptr = cfg->get_decode_buf(
            decoded_buffer_, frame_id, symbol_id, ue_id, cur_cb_id);
        ldpc_decoder_5gnr_request.varNodes = llr_buffer_ptr;
        ldpc_decoder_5gnr_response.compactedMessageBytes = decoded_buffer_ptr;

        size_t start_tsc1 = worker_rdtsc();
        duration_stat->task_duration[1] += start_tsc1 - start_tsc;

        bblib_ldpc_decoder_5gnr(
            &ldpc_decoder_5gnr_request, &ldpc_decoder_5gnr_response);

        size_t start_tsc2 = worker_rdtsc();
        duration_stat->task_duration[2] += start_tsc2 - start_tsc1;

        if (kPrintLLRData) {
            printf("LLR data, symbol_offset: %zu\n", symbol_offset);
            for (size_t i = 0; i < LDPC_config.cbCodewLen; i++) {
                printf("%d ", *(llr_buffer_ptr + i));
            }
            printf("\n");
        }

        if (kPrintDecodedData) {
            printf("Decoded data\n");
            for (size_t i = 0; i < (LDPC_config.cbLen >> 3); i++) {
                printf("%u ", *(decoded_buffer_ptr + i));
            }
            printf("\n");
        }

        if (!kEnableMac && kPrintPhyStats && symbol_id == cfg->UL_PILOT_SYMS) {
            phy_stats->update_decoded_bits(
                ue_id, symbol_offset, cfg->num_bytes_per_cb * 8);
            phy_stats->increment_decoded_blocks(ue_id, symbol_offset);
            size_t block_error(0);
            for (size_t i = 0; i < cfg->num_bytes_per_cb; i++) {
                uint8_t rx_byte = decoded_buffer_ptr[i];
                uint8_t tx_byte = (uint8_t)cfg->get_info_bits(
                    cfg->ul_bits, symbol_id, ue_id, cur_cb_id)[i];
                phy_stats->update_bit_errors(
                    ue_id, symbol_offset, tx_byte, rx_byte);
                if (rx_byte != tx_byte)
                    block_error++;
            }
            phy_stats->update_block_errors(ue_id, symbol_offset, block_error);
        }
    }

    double duration = worker_rdtsc() - start_tsc;
    duration_stat->task_duration[0] += duration;
    duration_stat->task_count++;
    if (cycles_to_us(duration, freq_ghz) > 500) {
        printf("Thread %d Decode takes %.2f\n", tid,
            cycles_to_us(duration, freq_ghz));
    }

    // When using remote LDPC, we ship the decoding task asynchronously to a
    // remote server and return immediately
    if (kUseRemote)
        return Event_data(EventType::kPendingToRemote, tag);
    else
        return Event_data(EventType::kDecode, tag);
}


/// This function should be run on its own thread to endlessly receive
/// decode responses from remote LDPC workers and trigger the proper completion
/// events expected by the rest of Millipede.
/// There should only be one thread instance of this function for the entire
/// Millipede process.
void decode_response_loop(Config* cfg)
{
    UDPServer udp_server(cfg->remote_ldpc_completion_port, kRxBufSize);
    size_t decoded_bits = ldpc_encoding_input_buf_size(
        cfg->LDPC_config.Bg, cfg->LDPC_config.Zc);
    // The number of bytes that we receive at one time from the LDPC worker
    // is defined by the `DecodeMsg` struct.
    size_t rx_buf_len = DecodeMsg::size_without_data() + decoded_bits;
    uint8_t* rx_buf = new uint8_t[rx_buf_len];
    DecodeMsg* msg = reinterpret_cast<DecodeMsg*>(rx_buf);

    while (cfg->running) {
        ssize_t bytes_rcvd = udp_server.recv_nonblocking(rx_buf, rx_buf_len);
        if (bytes_rcvd == 0) {
            // no data received
            continue;
        } else if (bytes_rcvd == -1) {
            // There was a socket receive error
            cfg->running = false;
            break;
        }

        rt_assert(bytes_rcvd == (ssize_t)rx_buf_len,
            "Rcvd wrong decode response len");
        DecodeContext* context = msg->context;
        DoDecode* computeDecoding = context->doer;
        rt_assert(
            context->tid == computeDecoding->tid, "DoDecode tid mismatch");

        // Copy the decoded buffer received from the remote LDPC worker
        // into the appropriate location in the decode doer's decoded buffer.
        uint8_t* out_buf
            = static_cast<uint8_t*>(
                  computeDecoding->decoded_buffer_[context->symbol_offset])
            + context->output_offset;
        memcpy(out_buf, msg->data, decoded_bits);

        Event_data resp_event;
        resp_event.num_tags = 1;
        resp_event.tags[0] = context->tag;
        /// TODO: FIXME: is this right? We never send a kDecodeLast event.
        resp_event.event_type = EventType::kDecode;

        try_enqueue_fallback(&computeDecoding->complete_task_queue,
            computeDecoding->worker_producer_token, resp_event);

        // TODO: here, return msg buffers to the pool

        // The message (rx_buf) will be reused, only delete the DecodeContext
        // that was allocated by the DoDecode doer when the request was sent.
        delete context;

        printf("Docoding: Received response %zu\n",
            computeDecoding->remote_ldpc_stub_->num_responses_received);
        computeDecoding->remote_ldpc_stub_->num_responses_received++;
    }

    printf("Exiting decode_response_loop()\n");
    delete rx_buf;
}
