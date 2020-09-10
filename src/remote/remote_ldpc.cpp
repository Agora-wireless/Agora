#include "remote_ldpc.hpp"
#include "udp_server.h"
#include "utils.h"
#include "utils_ldpc.hpp"
#include <arpa/inet.h>
#include <malloc.h>
#include <string>
#include <thread>
#include <vector>

static constexpr size_t kRxBufSize = 4 * 1024 * 1024;
static bool is_running = true;

RemoteLDPC::RemoteLDPC(Config* cfg, int tid, size_t rx_buf_size)
    : cfg(cfg)
    , LDPC_config(cfg->LDPC_config)
    , tid(tid)
    , decoded_bits(ldpc_encoding_input_buf_size(LDPC_config.Bg, LDPC_config.Zc))
    , udp_server(cfg->remote_ldpc_base_port + tid, rx_buf_size)
{
    resp_var_nodes = (int16_t*)memalign(64, 1024 * 1024 * sizeof(int16_t));
    worker_thread = std::thread([=] { worker_loop(tid); });
}

RemoteLDPC::~RemoteLDPC() { free(resp_var_nodes); }

void RemoteLDPC::decode(int8_t* in_buffer, uint8_t* out_buffer)
{
    struct bblib_ldpc_decoder_5gnr_request ldpc_request {
    };
    struct bblib_ldpc_decoder_5gnr_response ldpc_response {
    };

    // Decoder setup
    int16_t numFillerBits = 0;
    int16_t numChannelLlrs = LDPC_config.cbCodewLen;

    ldpc_request.numChannelLlrs = numChannelLlrs;
    ldpc_request.numFillerBits = numFillerBits;
    ldpc_request.maxIterations = LDPC_config.decoderIter;
    ldpc_request.enableEarlyTermination = LDPC_config.earlyTermination;
    ldpc_request.Zc = LDPC_config.Zc;
    ldpc_request.baseGraph = LDPC_config.Bg;
    ldpc_request.nRows = LDPC_config.nRows;

    int numMsgBits = LDPC_config.cbLen - numFillerBits;
    ldpc_response.numMsgBits = numMsgBits;
    ldpc_response.varNodes = resp_var_nodes;

    ldpc_request.varNodes = in_buffer;
    ldpc_response.compactedMessageBytes = out_buffer;

    bblib_ldpc_decoder_5gnr(&ldpc_request, &ldpc_response);
}

/// The main loop that continuously polls for incoming requests,
/// processes them, and returns a response (if enabled).
void RemoteLDPC::worker_loop(size_t tid)
{
    pin_to_core_with_offset(
        ThreadType::kWorker, cfg->remote_ldpc_core_offset, tid);

    // Create buffer for receiving a request
    size_t data_bytes
        = ldpc_max_num_encoded_bits(LDPC_config.Bg, LDPC_config.Zc);
    size_t msg_len = DecodeMsg::size_without_data() + data_bytes;
    uint8_t* rx_buf = new uint8_t[msg_len];
    DecodeMsg* request = reinterpret_cast<DecodeMsg*>(rx_buf);
    int8_t* in_buf = reinterpret_cast<int8_t*>(request->data);

    // Create buffer for sending a response
    size_t decoded_bits = ldpc_encoding_input_buf_size(
        cfg->LDPC_config.Bg, cfg->LDPC_config.Zc);
    size_t tx_buf_len = DecodeMsg::size_without_data() + decoded_bits;
    uint8_t* tx_buf = new uint8_t[tx_buf_len];
    DecodeMsg* response = reinterpret_cast<DecodeMsg*>(tx_buf);
    uint8_t* out_buf = reinterpret_cast<uint8_t*>(response->data);

    // Local variables for getting the IP addr of the sender
    sockaddr from;
    socklen_t addr_len = sizeof(sockaddr);
    sockaddr_in* from_ipv4 = reinterpret_cast<sockaddr_in*>(&from);
    char ipv4_addr[INET_ADDRSTRLEN];

    while (is_running) {
        ssize_t bytes_rcvd = udp_server.recvfrom_nonblocking(
            rx_buf, msg_len, &from, &addr_len);
        if (bytes_rcvd == 0) {
            // no data received
            continue;
        } else if (bytes_rcvd == -1) {
            // There was a socket receive error
            is_running = false;
            break;
        }
        rt_assert(
            bytes_rcvd == (ssize_t)msg_len, "Received wrong size LDPC request");
        rt_assert(from.sa_family == AF_INET, "Received non-IPv4 UDP packet");
        inet_ntop(AF_INET, &from_ipv4->sin_addr, ipv4_addr, sizeof(ipv4_addr));
        std::string response_dest_uri(ipv4_addr);

        printf("RemoteLDPC: Received request %zu from %s, context %p\n",
            request->msg_id, response_dest_uri.c_str(), request->context);

        decode(in_buf, out_buf);
        // The `context` ptr is meaningless here in the remote LDPC process,
        // so we just copy it over such that the asynchronous receiver that 
        // handles this response from us can handle it with the proper context. 
        response->context = request->context;
        response->msg_id = request->msg_id;
        udp_client.send(response_dest_uri, cfg->remote_ldpc_completion_port,
            tx_buf, tx_buf_len);

        num_reqs_recvd++;
    }

    printf("RemoteLDPC worker %zu stopping now.\n", tid);
    delete[] rx_buf;
    delete[] tx_buf;
}

int main(int argc, char** argv)
{
    std::string confFile;
    if (argc == 2) {
        confFile = std::string(argv[1]);
    } else {
        confFile = TOSTRING(PROJECT_DIRECTORY)
            + std::string("/data/tddconfig-sim-ul.json");
    }
    auto* cfg = new Config(confFile.c_str());
    cfg->genData();

    std::vector<RemoteLDPC*> remote_ldpc_workers;
    remote_ldpc_workers.reserve(cfg->remote_ldpc_num_threads);
    for (size_t tid = 0; tid < cfg->remote_ldpc_num_threads; tid++) {
        remote_ldpc_workers.push_back(new RemoteLDPC(cfg, tid, kRxBufSize));
    }

    // Wait for all the remote LDPC workers to complete, then free them.
    for (auto& r : remote_ldpc_workers) {
        r->join();
        delete r;
    }

    // TODO: set signal handler to stop threads upon interruption/kill

    return 0;
}