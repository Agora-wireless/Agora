/**
 * @file docoding.h
 * @brief Declaration file for the Docoding class.  Includes the DoEncode and
 * DoDecode classes.
 */

#ifndef DOCODING_H_
#define DOCODING_H_

#include <armadillo>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <vector>

#include "buffer.h"
#include "concurrentqueue.h"
#include "config.h"
#include "doer.h"
#include "encoder.h"
#include "gettime.h"
#include "iobuffer.h"
#include "memory_manage.h"
#include "modulation.h"
#include "phy_stats.h"
#include "scrambler.h"
#include "stats.h"
#include "symbols.h"
#include "udp_client.h"
#include "utils_ldpc.h"

#if defined(USE_DPDK)
#include <arpa/inet.h>
#include <netinet/ether.h>

#include "dpdk_transport.h"
#endif
/// A loop that handles decode responses from all remote LDPC workers.
void DecodeResponseLoop(Config* cfg);

/// A connection to a remote LDPC worker, including request/receive buffers
/// and counters.
/// A single instance of this is shared between the two `DoDecode` instances
/// in each worker thread.
class RemoteLdpcStub {
 public:
  RemoteLdpcStub(Config* cfg, int core_for_dpdk)
      : num_requests_issued_(0), num_responses_received_(0) {
    unused(cfg);            // used only for DPDK
    unused(core_for_dpdk);  // used only for DPDK

    // TODO: previously, eRPC req/resp buffers were pre-allocated here.
    // for (size_t i = 0; i < kRpcMaxMsgBufNum; i++) {
    //     auto* req_msgbuf = new erpc::MsgBuffer;
    //     *req_msgbuf = rpc->alloc_msg_buffer_or_die(kRpcMaxMsgSize);
    //     vec_req_msgbuf.push_back(req_msgbuf);
    // }

#ifdef USE_DPDK
    // Init DPDK on only this core
    DpdkTransport::dpdk_init(core_for_dpdk, 1);
    mbuf_pool = DpdkTransport::create_mempool();

    uint16_t portid = 0;
    if (!DpdkTransport::nic_init(portid, mbuf_pool, 1)) {
      rte_exit(EXIT_FAILURE, "Cannot init NIC with port %u\n", portid);
    }

    // `bs_server_addr` is this machine's local IP address,
    // because this code runs on the Agora "server".
    int ret = inet_pton(AF_INET, cfg->bs_server_addr.c_str(), &local_ip_addr);
    rt_assert(ret == 1, "Invalid local IP address");
    ret = inet_pton(AF_INET, cfg->remote_ldpc_ip_addr.c_str(), &remote_ip_addr);
    rt_assert(ret == 1, "Invalid remote IP address");

    ether_addr* parsed_mac = ether_aton(cfg->remote_ldpc_mac_addr.c_str());
    rt_assert(parsed_mac != NULL, "Invalid remote LDPC MAC address");
    memcpy(&remote_mac_addr, parsed_mac, sizeof(ether_addr));

    ret = rte_eth_macaddr_get(portid, &local_mac_addr);
    rt_assert(ret == 0, "Cannot get MAC address of the port");
    printf("Number of DPDK cores: %d\n", rte_lcore_count());
#endif  // USE_DPDK
  }

 public:
  /// Number of preallocated msgbufs to hold pending eRPC requests
  static const size_t kRpcMaxMsgBufNum = 6400;
  /// Maximum size of preallocated msgbufs
  static const size_t kRpcMaxMsgSize = (1 << 15);

  /// The UDP socket used to send decode requests to the remote LDPC worker.
  UDPClient udp_client_;
  size_t num_requests_issued_;
  size_t num_responses_received_;

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

#endif  // USE_DPDK
};

class DoEncode : public Doer {
 public:
  DoEncode(Config* in_config, int in_tid, Table<int8_t>& in_raw_data_buffer,
           Table<int8_t>& in_encoded_buffer, Stats* in_stats_manager);
  ~DoEncode() override;

  EventData Launch(size_t tag) override;

 private:
  Table<int8_t>& raw_data_buffer_;
  int8_t* parity_buffer_;  // Intermediate buffer to hold LDPC encoding parity

  // Intermediate buffer to hold LDPC encoding output
  int8_t* encoded_buffer_temp_;
  Table<int8_t>& encoded_buffer_;
  DurationStat* duration_stat_;
  int8_t* scrambler_buffer_;

  std::unique_ptr<AgoraScrambler::Scrambler> scrambler_;
};

class DoDecode : public Doer {
 public:
  DoDecode(Config* in_config, int in_tid,
           PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffers,
           PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t>& decoded_buffers,
           PhyStats* in_phy_stats, Stats* in_stats_manager);
  ~DoDecode() override;

  EventData Launch(size_t tag) override;

  /// Returns the `RemoteLdpcStub` that this doer uses to communicate
  /// with a remote LDPC worker.
  /// The returned `RemoteLdpcStub` can be shared with other doers;
  /// see the `set_initialized_remote_ldpc_stub()` method.
  RemoteLdpcStub* InitializeRemoteLdpcStub(int core_for_dpdk) {
    remote_ldpc_stub_ = new RemoteLdpcStub(cfg_, core_for_dpdk);
    return remote_ldpc_stub_;
  }

  /// Sets this doer's `RemoteLdpcStub` to an already-initialized instance.
  /// This is useful because a single worker thread creates **two** instances
  /// of `DoDecode`, and they must share a single `RemoteLdpcStub` context.
  void SetInitializedRemoteLdpcStub(RemoteLdpcStub* stub) {
    remote_ldpc_stub_ = stub;
  }

  /// The loop that runs to receive decode completion responses from
  /// all remote LDPC workers.
  /// This is a friend method because it must access private member fields
  /// within `DoDecode`.
  friend void DecodeResponseLoop(Config* cfg);

 private:
  int16_t* resp_var_nodes_;
  PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffers_;
  PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t>& decoded_buffers_;
  PhyStats* phy_stats_;
  DurationStat* duration_stat_;
  std::unique_ptr<AgoraScrambler::Scrambler> scrambler_;
  /// when `kUseRemote` is true.
  RemoteLdpcStub* remote_ldpc_stub_;
};

// Forward declaration
class DecodeMsg;

/// A context object used when receiving responses from the remote LDPC
/// worker.
class DecodeContext {
 public:
  /// The pointer to which the decoded data should be copied.
  uint8_t* output_ptr_;
  /// The DoDecode instance that sent the request.
  DoDecode* doer_;
  /// The worker thread ID that owns (created) the above `doer`.
  size_t tid_;
  size_t tag_;

  /// TODO: create and use this request buffer pool.
  std::vector<DecodeMsg*> request_buffer_pool_;

  DecodeContext(uint8_t* output_ptr, DoDecode* doer, size_t tid, size_t tag)
      : output_ptr_(output_ptr), doer_(doer), tid_(tid), tag_(tag) {}
};

/// The "packet" format of a decode request or response sent between
/// Agora and a remote LDPC worker.
struct DecodeMsg {
  /// The context of this decode request, which is only validly accessible
  /// from the Agora side where the `DoDecode` instance was created.
  /// NOTE: THIS IS NOT VALID from within the remote LDPC worker side.
  DecodeContext* context;
  /// An optional value used for debugging request/response packet drops.
  size_t msg_id;

  /// The dynamically-sized data in the message.
  /// NOTE: THIS MUST BE the last field in this struct.
  uint8_t data[];

  /// Returns the size of the "header" of this message, i.e.,
  /// everything before the dynamically-sized data.
  static size_t SizeWithoutData() { return offsetof(DecodeMsg, data); }
};

#endif  // DOCODING_H_
