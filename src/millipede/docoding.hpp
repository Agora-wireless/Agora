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
#include "udp_client.h"
#include <armadillo>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <vector>

#include "encoder.hpp"
#include "iobuffer.hpp"
#include "utils_ldpc.hpp"

#ifdef USE_REMOTE
/// A forward declaration of the function used to handle decode responses
/// from remote LDPC workers.
void decode_response_loop(Config* cfg);

/// A connection to a remote LDPC worker, including request/receive buffers
/// and counters.
/// A single instance of this is shared between the two `DoDecode` instances
/// in each worker thread.
class RemoteLdpcStub {
public:
    RemoteLdpcStub()
        : num_requests_issued(0)
        , num_responses_received(0)
    {
        // TODO: previously, eRPC req/resp buffers were pre-allocated here.
        // for (size_t i = 0; i < kRpcMaxMsgBufNum; i++) {
        //     auto* req_msgbuf = new erpc::MsgBuffer;
        //     *req_msgbuf = rpc->alloc_msg_buffer_or_die(kRpcMaxMsgSize);
        //     vec_req_msgbuf.push_back(req_msgbuf);
        // }
    }

public:
    /// Number of preallocated msgbufs to hold pending eRPC requests
    static const size_t kRpcMaxMsgBufNum = 6400;
    /// Maximum size of preallocated msgbufs
    static const size_t kRpcMaxMsgSize = (1 << 15);

    /// The UDP socket used to send decode requests to the remote LDPC worker.
    UDPClient udp_client;
    size_t num_requests_issued;
    size_t num_responses_received;
};
#endif // USE_REMOTE

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
public:
    /// Returns the `RemoteLdpcStub` that this doer uses to communicate
    /// with a remote LDPC worker.
    /// The returned `RemoteLdpcStub` can be shared with other doers;
    /// see the `set_initialized_remote_ldpc_stub()` method.
    RemoteLdpcStub* initialize_remote_ldpc_stub()
    {
        remote_ldpc_stub_ = new RemoteLdpcStub();
        return remote_ldpc_stub_;
    }

    inline size_t get_num_requests()
    {
        return remote_ldpc_stub_->num_requests_issued;
    }
    inline size_t get_num_responses()
    {
        return remote_ldpc_stub_->num_responses_received;
    }

    friend void decode_response_loop(Config* cfg);

    /// Sets this doer's `RemoteLdpcStub` to an already-initialized instance.
    /// This is useful because a single worker thread creates **two** instances
    /// of `DoDecode`, and they must share a single `RemoteLdpcStub` context.
    void set_initialized_remote_ldpc_stub(RemoteLdpcStub* stub)
    {
        remote_ldpc_stub_ = stub;
    }

private:
    RemoteLdpcStub* remote_ldpc_stub_;
#endif // USE_REMOTE

private:
    int16_t* resp_var_nodes;
    Table<int8_t>& llr_buffer_;
    Table<uint8_t>& decoded_buffer_;
    //Table<int> decoded_bits_count_;
    //Table<int> error_bits_count_;
    PhyStats* phy_stats;
    DurationStat* duration_stat;
};

#ifdef USE_REMOTE
struct DecodeMsg; // forward declaration

/// A context object used when receiving responses from the remote LDPC worker.
class DecodeContext {
public:
    size_t symbol_offset;
    size_t output_offset;
    size_t tag;
    int tid;
    /// The DoDecode instance that sent the request.
    DoDecode* doer;
    std::vector<DecodeMsg*> request_buffer_pool;

    DecodeContext(size_t symbol_offset, size_t output_offset, size_t tag,
        int tid, DoDecode* doer)
        : symbol_offset(symbol_offset)
        , output_offset(output_offset)
        , tag(tag)
        , tid(tid)
        , doer(doer)
    {
    }
};

/// The "packet" format of a decode request or response sent between
/// Millipede and a remote LDPC worker.
struct DecodeMsg {
    /// The context of this decode request, which is only validly accessible
    /// from the Millipede side where the `DoDecode` instance was created.
    /// NOTE: THIS IS NOT VALID from within the remote LDPC worker side.
    DecodeContext* context;
    /// The dynamically-sized data in the message.
    uint8_t data[];

    /// Returns the size of the "header" of this message, i.e.,
    /// everything before the dynamically-sized data.
    static size_t size_without_data()
    {
        return offsetof(DecodeMsg, data);
    }
};
#endif // USE_REMOTE

#endif // DOCODING
