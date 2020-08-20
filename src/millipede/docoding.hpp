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

// #ifdef USE_REMOTE
#include "rpc.h"

/// The state of an ePRC connection to a remote LDPC worker,
/// including request/receive message buffers and counters.
/// A single instance of this is shared between the two `DoDecode` instances
/// in each worker thread.
class RpcContext {
public:
    RpcContext(erpc::Rpc<erpc::CTransport>* rpc_, int session_)
        : rpc(rpc_)
        , session(session_)
        , num_requests_issued(0)
        , num_responses_received(0)
    {
        rpc = rpc_;
        session = session_;
        for (size_t i = 0; i < kRpcMaxMsgBufNum; i++) {
            auto* req_msgbuf = new erpc::MsgBuffer;
            *req_msgbuf = rpc->alloc_msg_buffer_or_die(kRpcMaxMsgSize);
            vec_req_msgbuf.push_back(req_msgbuf);
        }
        for (size_t i = 0; i < kRpcMaxMsgBufNum; i++) {
            auto* resp_msgbuf = new erpc::MsgBuffer;
            *resp_msgbuf = rpc->alloc_msg_buffer_or_die(kRpcMaxMsgSize);
            vec_resp_msgbuf.push_back(resp_msgbuf);
        }
    }

public:
    /// Number of preallocated msgbufs to hold pending eRPC requests
    static const size_t kRpcMaxMsgBufNum = 6400;
    /// Maximum size of preallocated msgbufs
    static const size_t kRpcMaxMsgSize = (1 << 15);

    erpc::Rpc<erpc::CTransport>* rpc;
    std::vector<erpc::MsgBuffer*> vec_req_msgbuf;
    std::vector<erpc::MsgBuffer*> vec_resp_msgbuf;
    int session;
    size_t num_requests_issued;
    size_t num_responses_received;
};
// #endif

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

    // #ifdef USE_REMOTE

    /// Creates and returns a new `RpcContext` that this doer will use to
    /// communicate with the remote LDPC worker.
    /// The returned `RpcContext` can be shared with other doers;
    /// see the `set_initialized_rpc_context()` method.
    RpcContext* initialize_erpc(erpc::Rpc<erpc::CTransport>* rpc, int session);

    inline size_t get_num_requests()
    {
        return rpc_context_->num_requests_issued;
    }
    inline size_t get_num_responses()
    {
        return rpc_context_->num_responses_received;
    }

    friend void decode_cont_func(void* _context, void* _tag);

    /// Sets this doer's `RpcContext` to an already-initialized instance.
    /// This is useful because a single worker thread creates **two** instances
    /// of `DoDecode`, and they must share a single Rpc context.
    void set_initialized_rpc_context(RpcContext* rpc_context)
    {
        rpc_context_ = rpc_context;
    }
    // #endif

public:
    /// eRPC request type for remote LDPC decoding
    static const size_t kRpcReqType = 2;
    
private:
    int16_t* resp_var_nodes;
    Table<int8_t>& llr_buffer_;
    Table<uint8_t>& decoded_buffer_;
    //Table<int> decoded_bits_count_;
    //Table<int> error_bits_count_;
    PhyStats* phy_stats;
    DurationStat* duration_stat;

    // #ifdef USE_REMOTE

    RpcContext* rpc_context_;
    // #endif
};

// #ifdef USE_REMOTE
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
// #endif

#endif
