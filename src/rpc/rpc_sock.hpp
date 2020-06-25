#ifndef RPC_SOCK
#define RPC_SOCK

#include "rpc.h"
#include <string>

static const std::string kServerHostname = "192.168.12.146";
static const std::string kClientHostname = "192.168.12.145";

static constexpr uint16_t kUDPPort = 31850;
static constexpr uint8_t kReqType = 2;
static constexpr size_t kMsgSize = 1024;

void run_erpc_server();
void run_erpc_client();

class RPCContext {
public:
    RPCContext(std::string local_uri, size_t obj_id);
    ~RPCContext();

    int Serve();
    int Connect(std::string uri, size_t obj_id);
    int Send(int session_num, char* buf, size_t msg_len);
    int Send(char *buf, size_t msg_len);

    erpc::Nexus *nexus;
    erpc::Rpc<erpc::CTransport> *rpc;
    std::vector<int> session_vec;
    erpc::MsgBuffer req_msgbuf;
    erpc::MsgBuffer resp_msgbuf;
    int dedicated_session;
};

#endif // RPC_SOCK