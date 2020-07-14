/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#ifndef DOCODING
#define DOCODING

#include "Symbols.hpp"
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "doer.hpp"
#include "gettime.h"
#include "memory_manage.h"
#include "modulation.hpp"
#include "phy_stats.hpp"
#include "stats.hpp"
#include <armadillo>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <vector>

#include "encoder.hpp"
#include "iobuffer.hpp"
#include "utils_ldpc.hpp"

#ifdef USE_REMOTE
#include "rpc.h"
#endif

class DoEncode : public Doer {
public:
    DoEncode(Config* in_config, int in_tid, double freq_ghz,
        moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
        moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
        moodycamel::ProducerToken* worker_producer_token,
        Table<int8_t>& in_raw_data_buffer, Table<int8_t>& in_encoded_buffer,
        Stats* in_stats_manager);
    ~DoEncode();

    Event_data launch(size_t tag);

private:
    Table<int8_t>& raw_data_buffer_;
    int8_t* parity_buffer; // Intermediate buffer to hold LDPC encoding parity

    // Intermediate buffer to hold LDPC encoding output
    int8_t* encoded_buffer_temp;
    Table<int8_t>& encoded_buffer_;
    DurationStat* duration_stat;
};

class DoDecode : public Doer {
public:
    DoDecode(Config* in_config, int in_tid, double freq_ghz,
        moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
        moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
        moodycamel::ProducerToken* worker_producer_token,
        Table<int8_t>& in_demod_buffer, Table<uint8_t>& in_decoded_buffer,
        //Table<int>& in_decoded_bits_count, Table<int>& in_error_bits_count,
        PhyStats* in_phy_stats, Stats* in_stats_manager);
    ~DoDecode();

    Event_data launch(size_t tag);

#ifdef USE_REMOTE
    /// Attach an eRPC object to the DoDecode object, and register the session number connected with the remote LDPC object
    void initialize_erpc(erpc::Rpc<erpc::CTransport>* rpc, int session);

    inline size_t get_num_requests() { return num_requests_issued; }
    inline size_t get_num_responses() { return num_responses_received; }

    friend void decode_cont_func(void* _context, void* _tag);
#endif

private:
    int16_t* resp_var_nodes;
    Table<int8_t>& llr_buffer_;
    Table<uint8_t>& decoded_buffer_;
    //Table<int> decoded_bits_count_;
    //Table<int> error_bits_count_;
    PhyStats* phy_stats;
    DurationStat* duration_stat;

#ifdef USE_REMOTE
    /// Number of preallocated msgbufs to hold pending eRPC requests
    static const size_t kRpcMaxMsgBufNum = 64;

    /// eRPC request type for remote LDPC decoding
    static const size_t kRpcReqType = 2;

    /// Maximum size of preallocated msgbufs
    static const size_t kRpcMaxMsgSize = (1 << 20);

    erpc::Rpc<erpc::CTransport>* rpc;
    std::vector<erpc::MsgBuffer*> vec_req_msgbuf;
    std::vector<erpc::MsgBuffer*> vec_resp_msgbuf;
    int session;
    size_t num_requests_issued;
    size_t num_responses_received;
#endif
};

#ifdef USE_REMOTE
/// Local eRPC request tag used for processing responses of decoding tasks
class DecodeTag {
public:
    size_t symbol_offset;
    size_t output_offset;
    size_t tag;
    int tid;
    erpc::MsgBuffer* req_msgbuf;
    erpc::MsgBuffer* resp_msgbuf;
    erpc::Rpc<erpc::CTransport>* rpc;
};
#endif

#endif
