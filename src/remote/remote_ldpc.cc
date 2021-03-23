/**
 * @file remote_ldpc.cc
 * @brief This file implments the RemoteLDPC class and the ldpc request handler
 * function
 */
#include "remote_ldpc.h"

#include <arpa/inet.h>
#include <malloc.h>

#include <string>
#include <thread>
#include <vector>

#include "udp_server.h"
#include "utils.h"
#include "utils_ldpc.h"

static constexpr size_t kRxBufSize = 4 * 1024 * 1024;
static bool is_running = true;

RemoteLDPC::RemoteLDPC(Config* cfg, size_t tid, size_t rx_buf_size)
    : cfg_(cfg),
      ldpc_config_(cfg->LdpcConfig()),
      tid_(tid),
      decoded_bits_(LdpcEncodingInputBufSize(ldpc_config_.BaseGraph(),
                                             ldpc_config_.ExpansionFactor())),
      udp_server_(cfg->RemoteLdpcBasePort() + tid, rx_buf_size) {
  resp_var_nodes_ = (int16_t*)memalign(64, 1024 * 1024 * sizeof(int16_t));
  worker_thread_ = std::thread([=] { WorkerLoop(tid); });
}

RemoteLDPC::~RemoteLDPC() { std::free(resp_var_nodes_); }

void RemoteLDPC::Decode(int8_t* in_buffer, uint8_t* out_buffer) {
  struct bblib_ldpc_decoder_5gnr_request ldpc_request {};
  struct bblib_ldpc_decoder_5gnr_response ldpc_response {};

  // Decoder setup
  int16_t numFillerBits = 0;
  int16_t numChannelLlrs = ldpc_config_.NumCbCodewLen();

  ldpc_request.numChannelLlrs = numChannelLlrs;
  ldpc_request.numFillerBits = numFillerBits;
  ldpc_request.maxIterations = ldpc_config_.MaxDecoderIter();
  ldpc_request.enableEarlyTermination = ldpc_config_.EarlyTermination();
  ldpc_request.Zc = ldpc_config_.ExpansionFactor();
  ldpc_request.baseGraph = ldpc_config_.BaseGraph();
  ldpc_request.nRows = ldpc_config_.NumRows();

  int numMsgBits = ldpc_config_.NumCbLen() - numFillerBits;
  ldpc_response.numMsgBits = numMsgBits;
  ldpc_response.varNodes = resp_var_nodes_;

  ldpc_request.varNodes = in_buffer;
  ldpc_response.compactedMessageBytes = out_buffer;

  bblib_ldpc_decoder_5gnr(&ldpc_request, &ldpc_response);
}

/// The main loop that continuously polls for incoming requests,
/// processes them, and returns a response (if enabled).
void RemoteLDPC::WorkerLoop(size_t tid) {
  PinToCoreWithOffset(ThreadType::kWorker, cfg_->RemoteLdpcCoreOffset(), tid);

  // Create buffer for receiving a request
  size_t data_bytes = LdpcMaxNumEncodedBits(ldpc_config_.BaseGraph(),
                                            ldpc_config_.ExpansionFactor());
  size_t msg_len = DecodeMsg::SizeWithoutData() + data_bytes;
  uint8_t* rx_buf = new uint8_t[msg_len];
  DecodeMsg* request = reinterpret_cast<DecodeMsg*>(rx_buf);
  int8_t* in_buf = reinterpret_cast<int8_t*>(request->data);

  // Create buffer for sending a response
  size_t decoded_bits = LdpcEncodingInputBufSize(
      ldpc_config_.BaseGraph(), ldpc_config_.ExpansionFactor());
  size_t tx_buf_len = DecodeMsg::SizeWithoutData() + decoded_bits;
  uint8_t* tx_buf = new uint8_t[tx_buf_len];
  DecodeMsg* response = reinterpret_cast<DecodeMsg*>(tx_buf);
  uint8_t* out_buf = reinterpret_cast<uint8_t*>(response->data);

  // Local variables for getting the IP addr of the sender
  sockaddr from;
  socklen_t addr_len = sizeof(sockaddr);
  sockaddr_in* from_ipv4 = reinterpret_cast<sockaddr_in*>(&from);
  char ipv4_addr[INET_ADDRSTRLEN];

  while (is_running) {
    ssize_t bytes_rcvd =
        udp_server_.RecvFromNonblocking(rx_buf, msg_len, &from, &addr_len);
    if (bytes_rcvd == 0) {
      // no data received
      continue;
    } else if (bytes_rcvd == -1) {
      // There was a socket receive error
      is_running = false;
      break;
    }
    RtAssert(bytes_rcvd == (ssize_t)msg_len,
             "Received wrong size LDPC request");
    RtAssert(from.sa_family == AF_INET, "Received non-IPv4 UDP packet");
    inet_ntop(AF_INET, &from_ipv4->sin_addr, ipv4_addr, sizeof(ipv4_addr));
    std::string response_dest_uri(ipv4_addr);

    printf("RemoteLDPC: Received request %zu from %s, context %p\n",
           request->msg_id, response_dest_uri.c_str(), request->context);

    Decode(in_buf, out_buf);
    // The `context` ptr is meaningless here in the remote LDPC process,
    // so we just copy it over such that the asynchronous receiver that
    // handles this response from us can handle it with the proper context.
    response->context = request->context;
    response->msg_id = request->msg_id;
    udp_client_.Send(response_dest_uri, cfg_->RemoteLdpcCompletionPort(),
                     tx_buf, tx_buf_len);

    num_reqs_recvd_++;
  }

  printf("RemoteLDPC worker %zu stopping now.\n", tid);
  delete[] rx_buf;
  delete[] tx_buf;
}

int main(int argc, char** argv) {
  std::string conf_file;
  if (argc == 2) {
    conf_file = std::string(argv[1]);
  } else {
    conf_file = TOSTRING(PROJECT_DIRECTORY) +
                std::string("/data/tddconfig-sim-ul.json");
  }
  auto cfg = std::make_unique<Config>(conf_file.c_str());
  cfg->GenData();

  std::vector<RemoteLDPC*> remote_ldpc_workers;
  remote_ldpc_workers.reserve(cfg->RemoteLdpcNumThreads());
  for (size_t tid = 0; tid < cfg->RemoteLdpcNumThreads(); tid++) {
    remote_ldpc_workers.push_back(new RemoteLDPC(cfg.get(), tid, kRxBufSize));
  }

  // Wait for all the remote LDPC workers to complete, then free them.
  for (auto& r : remote_ldpc_workers) {
    r->Join();
    delete r;
  }

  // TODO: set signal handler to stop threads upon interruption/kill
  return 0;
}