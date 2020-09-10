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

/// The size of the kernel-level socket receive buffer,
/// used for receiving responses from the remote LDPC worker.
static constexpr size_t kRxBufSize = 4 * 1024 * 1024;
/// The factor used to determine when a remote LDPC worker is considered to be
/// falling behind and is in an "unhealthy" state.
static constexpr size_t thresholdPendingRequestsFactor = 10;

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

    ldpc_encode_helper(LDPC_config.Bg, LDPC_config.Zc, LDPC_config.nRows,
        encoded_buffer_temp, parity_buffer, input_ptr);
    int8_t* final_output_ptr = cfg->get_encoded_buf(
        encoded_buffer_, frame_id, symbol_id, ue_id, cur_cb_id);
    adapt_bits_for_mod(reinterpret_cast<uint8_t*>(encoded_buffer_temp),
        reinterpret_cast<uint8_t*>(final_output_ptr),
        bits_to_bytes(LDPC_config.cbCodewLen), cfg->mod_order_bits);

    // printf("Encoded data\n");
    // int num_mod = LDPC_config.cbCodewLen / cfg->mod_order_bits;
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
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffers,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t>& decoded_buffers,
    PhyStats* in_phy_stats, Stats* in_stats_manager)
    : Doer(in_config, in_tid, freq_ghz, in_task_queue, complete_task_queue,
          worker_producer_token)
    , demod_buffers_(demod_buffers)
    , decoded_buffers_(decoded_buffers)
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
    const size_t frame_id = gen_tag_t(tag).frame_id;
    const size_t symbol_idx_ul = gen_tag_t(tag).symbol_id;
    const size_t cb_id = gen_tag_t(tag).cb_id;
    const size_t symbol_offset
        = cfg->get_total_data_symbol_idx_ul(frame_id, symbol_idx_ul);
    const size_t cur_cb_id = cb_id % cfg->LDPC_config.nblocksInSymbol;
    const size_t ue_id = cb_id / cfg->LDPC_config.nblocksInSymbol;
    const size_t frame_slot = frame_id % kFrameWnd;
    if (kDebugPrintInTask) {
        printf("In doDecode thread %d: frame: %zu, symbol: %zu, code block: "
               "%zu, ue: %zu\n",
            tid, frame_id, symbol_idx_ul, cur_cb_id, ue_id);
    }

    // `llr_buffer_ptr` is the input buffer into the decoder.
    int8_t* llr_buffer_ptr = demod_buffers_[frame_slot][symbol_idx_ul][ue_id]
        + (cfg->mod_order_bits * (LDPC_config.cbCodewLen * cur_cb_id));

    if (kPrintLLRData) {
        printf("LLR data, symbol_offset: %zu\n", symbol_offset);
        for (size_t i = 0; i < LDPC_config.cbCodewLen; i++) {
            printf("%d ", *(llr_buffer_ptr + i));
        }
        printf("\n");
    }

    // `decoded_buffer_ptr` is the output buffer to which the decoded data
    // will be written.
    uint8_t* decoded_buffer_ptr
        = decoded_buffers_[frame_slot][symbol_idx_ul][ue_id]
        + (cur_cb_id * roundup<64>(cfg->num_bytes_per_cb));

    size_t start_tsc = worker_rdtsc();

    if (kUseRemote) {
        auto* decode_context
            = new DecodeContext(decoded_buffer_ptr, this, tid, tag);
        size_t data_bytes
            = ldpc_max_num_encoded_bits(LDPC_config.Bg, LDPC_config.Zc);
        size_t msg_len = DecodeMsg::size_without_data() + data_bytes;
        uint16_t dest_port = cfg->remote_ldpc_base_port + tid;

#ifdef USE_DPDK
        uint16_t src_port = 31850 - 1; /// TODO: FIXME: use proper src port
        rte_mbuf* tx_mbuf = DpdkTransport::alloc_udp(
            remote_ldpc_stub_->mbuf_pool, remote_ldpc_stub_->local_mac_addr,
            remote_ldpc_stub_->remote_mac_addr,
            remote_ldpc_stub_->local_ip_addr, remote_ldpc_stub_->remote_ip_addr,
            src_port, dest_port, msg_len);
        // `msg_buf` points to where our payload starts in the DPDK `tx_mbuf`
        uint8_t* msg_buf
            = (rte_pktmbuf_mtod(tx_mbuf, uint8_t*) + kPayloadOffset);
#else
        // TODO: pre-allocate a pool of request buffers to avoid this
        //       temporary allocation on the critical data path.
        uint8_t* msg_buf = new uint8_t[msg_len];
#endif // USE_DPDK

        auto* request = reinterpret_cast<DecodeMsg*>(msg_buf);
        request->context = decode_context;
        request->msg_id = remote_ldpc_stub_->num_requests_issued;
        memcpy(request->data, reinterpret_cast<uint8_t*>(llr_buffer_ptr),
            data_bytes);

        printf("Docoding: Issuing request %zu, context: %p\n",
            remote_ldpc_stub_->num_requests_issued, request->context);
        remote_ldpc_stub_->num_requests_issued++;

#ifdef USE_DPDK
        rt_assert(rte_eth_tx_burst(0, tid, &tx_mbuf, 1) == 1,
            "rte_eth_tx_burst() failed");
#else
        remote_ldpc_stub_->udp_client.send(
            cfg->remote_ldpc_ip_addr, dest_port, msg_buf, msg_len);
        delete[] msg_buf; // TODO: remove this once we use request buf pools
#endif // USE_DPDK

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

        ldpc_decoder_5gnr_request.varNodes = llr_buffer_ptr;
        ldpc_decoder_5gnr_response.compactedMessageBytes = decoded_buffer_ptr;

        size_t start_tsc1 = worker_rdtsc();
        duration_stat->task_duration[1] += start_tsc1 - start_tsc;

        bblib_ldpc_decoder_5gnr(
            &ldpc_decoder_5gnr_request, &ldpc_decoder_5gnr_response);

        size_t start_tsc2 = worker_rdtsc();
        duration_stat->task_duration[2] += start_tsc2 - start_tsc1;

        if (kPrintDecodedData) {
            printf("Decoded data\n");
            for (size_t i = 0; i < (LDPC_config.cbLen >> 3); i++) {
                printf("%u ", *(decoded_buffer_ptr + i));
            }
            printf("\n");
        }

        if (!kEnableMac && kPrintPhyStats
            && symbol_idx_ul == cfg->UL_PILOT_SYMS) {
            phy_stats->update_decoded_bits(
                ue_id, symbol_offset, cfg->num_bytes_per_cb * 8);
            phy_stats->increment_decoded_blocks(ue_id, symbol_offset);
            size_t block_error(0);
            for (size_t i = 0; i < cfg->num_bytes_per_cb; i++) {
                uint8_t rx_byte = decoded_buffer_ptr[i];
                uint8_t tx_byte = (uint8_t)cfg->get_info_bits(
                    cfg->ul_bits, symbol_idx_ul, ue_id, cur_cb_id)[i];
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
/// events expected by the rest of Agora.
/// There should only be one thread instance of this function for the entire
/// Agora process.
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
        memcpy(context->output_ptr, msg->data, decoded_bits);

        Event_data resp_event;
        resp_event.num_tags = 1;
        resp_event.tags[0] = context->tag;
        resp_event.event_type = EventType::kDecode;

        try_enqueue_fallback(&computeDecoding->complete_task_queue,
            computeDecoding->worker_producer_token, resp_event);

        // TODO: here, return msg buffers to the pool

        // `rx_buf` will be reused to receive the next message,
        // so we only delete the DecodeContext that was allocated
        // by the DoDecode doer when the request was sent.
        delete context;

        printf("Docoding: Received response %zu\n", msg->msg_id);
        computeDecoding->remote_ldpc_stub_->num_responses_received++;

        // Optional: here we can check if we have received far fewer responses
        // than requests issued, and then issue a health warning.
        if ((computeDecoding->remote_ldpc_stub_->num_requests_issued
                - computeDecoding->remote_ldpc_stub_->num_responses_received)
            > (thresholdPendingRequestsFactor * cfg->UE_ANT_NUM)) {
            MLPD_WARN("Some remote LDPC requests were lost or got no response. "
                      "Requests: %zu, Responses: %zu, batch size: %zu\n",
                computeDecoding->remote_ldpc_stub_->num_requests_issued,
                computeDecoding->remote_ldpc_stub_->num_responses_received,
                cfg->UE_ANT_NUM);
        }
    }

    printf("Exiting decode_response_loop()\n");
    delete rx_buf;
}
