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

#ifdef USE_DPDK
#include "dpdk_transport.hpp"
#include <arpa/inet.h>
#include <netinet/ether.h>
#endif

/// A loop that handles decode responses from all remote LDPC workers.
void decode_response_loop(Config* cfg);

/// A connection to a remote LDPC worker, including request/receive buffers
/// and counters.
/// A single instance of this is shared between the two `DoDecode` instances
/// in each worker thread.
class RemoteLdpcStub {
public:
    RemoteLdpcStub(Config* cfg)
        : num_requests_issued(0)
        , num_responses_received(0)
    {
        // TODO: previously, eRPC req/resp buffers were pre-allocated here.
        // for (size_t i = 0; i < kRpcMaxMsgBufNum; i++) {
        //     auto* req_msgbuf = new erpc::MsgBuffer;
        //     *req_msgbuf = rpc->alloc_msg_buffer_or_die(kRpcMaxMsgSize);
        //     vec_req_msgbuf.push_back(req_msgbuf);
        // }

#ifdef USE_DPDK
        /// TODO: what's the proper core offset and worker thread num?
        DpdkTransport::dpdk_init(cfg->core_offset, cfg->worker_thread_num);
        mbuf_pool = DpdkTransport::create_mempool();

        uint16_t portid = 0;
        if (!DpdkTransport::nic_init(portid, mbuf_pool, cfg->worker_thread_num))
        {
            rte_exit(EXIT_FAILURE, "Cannot init NIC with port %u\n", portid);
        }

        // `server_addr` is this machine's local IP address, 
        // because this code runs on the Millipede "server".
        int ret = inet_pton(AF_INET, cfg->server_addr.c_str(), &local_ip_addr);
        rt_assert(ret == 1, "Invalid local IP address");
        ret = inet_pton(
            AF_INET, cfg->remote_ldpc_ip_addr.c_str(), &remote_ip_addr);
        rt_assert(ret == 1, "Invalid remote IP address");

        ether_addr* parsed_mac = ether_aton(cfg->remote_ldpc_mac_addr.c_str());
        rt_assert(parsed_mac != NULL, "Invalid remote LDPC MAC address");
        memcpy(&remote_mac_addr, parsed_mac, sizeof(ether_addr));

        ret = rte_eth_macaddr_get(portid, &local_mac_addr);
        rt_assert(ret == 0, "Cannot get MAC address of the port");
        printf("Number of DPDK cores: %d\n", rte_lcore_count());
#endif // USE_DPDK
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

#ifdef USE_DPDK
    struct rte_mempool* mbuf_pool;
    /// The IP address of this local machine.
    uint32_t local_ip_addr;
    /// The IP address of the remote LDPC machine.
    uint32_t remote_ip_addr;
    /// The MAC address of the NIC we're using on this local machine.
    rte_ether_addr local_mac_addr;
    /// The MAC address of the remote LDPC worker to which we send requests.
    rte_ether_addr remote_mac_addr;

#endif // USE_DPDK
};

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

    /// Returns the `RemoteLdpcStub` that this doer uses to communicate
    /// with a remote LDPC worker.
    /// The returned `RemoteLdpcStub` can be shared with other doers;
    /// see the `set_initialized_remote_ldpc_stub()` method.
    RemoteLdpcStub* initialize_remote_ldpc_stub()
    {
        remote_ldpc_stub_ = new RemoteLdpcStub();
        return remote_ldpc_stub_;
    }

    /// Sets this doer's `RemoteLdpcStub` to an already-initialized instance.
    /// This is useful because a single worker thread creates **two** instances
    /// of `DoDecode`, and they must share a single `RemoteLdpcStub` context.
    void set_initialized_remote_ldpc_stub(RemoteLdpcStub* stub)
    {
        remote_ldpc_stub_ = stub;
    }

    /// The loop that runs to receive decode completion responses from
    /// all remote LDPC workers.
    /// This is a friend method because it must access private member fields
    /// within `DoDecode`.
    friend void decode_response_loop(Config* cfg);

private:
    int16_t* resp_var_nodes;
    Table<int8_t>& llr_buffer_;
    Table<uint8_t>& decoded_buffer_;
    PhyStats* phy_stats;
    DurationStat* duration_stat;
    /// Represents the connection to a remote LDPC worker, which is only used
    /// when `kUseRemote` is true.
    RemoteLdpcStub* remote_ldpc_stub_;
};

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
    /// TODO: create and use this request buffer pool.
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
    /// NOTE: THIS MUST BE the last field in this struct.
    uint8_t data[];

    /// Returns the size of the "header" of this message, i.e.,
    /// everything before the dynamically-sized data.
    static size_t size_without_data() { return offsetof(DecodeMsg, data); }
};

#endif // DOCODING
