#ifndef RPC_SOCK
#define RPC_SOCK

#include "rpc.h"
#include <string>

static const std::string kServerHostname = "192.168.12.145";
static const std::string kClientHostname = "192.168.12.146";

static constexpr uint16_t kUDPPort = 31850;
static constexpr uint8_t kReqType = 2;
static constexpr size_t kMsgSize = 16;

void run_erpc_server();
void run_erpc_client();

#endif // RPC_SOCK