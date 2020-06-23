#include "rpc_sock.hpp"

erpc::Rpc<erpc::CTransport> *server_rpc;
erpc::Rpc<erpc::CTransport> *client_rpc;
erpc::MsgBuffer req;
erpc::MsgBuffer resp;

void server_req_handler(erpc::ReqHandle *req_handle, void *) {
    auto &resp = req_handle->pre_resp_msgbuf;
    server_rpc->resize_msg_buffer(&resp, kMsgSize);
    sprintf(reinterpret_cast<char *>(resp.buf), "hello");

    server_rpc->enqueue_response(req_handle, &resp);
}

void run_erpc_server() {
    std::string server_uri = kServerHostname + ":" + std::to_string(kUDPPort);
    erpc::Nexus nexus(server_uri, 0, 0);
    nexus.register_req_func(kReqType, server_req_handler);

    server_rpc = new erpc::Rpc<erpc::CTransport>(&nexus, nullptr, 0, nullptr);
    server_rpc->run_event_loop(100000);
}

void sm_handler(int, erpc::SmEventType, erpc::SmErrType, void *) {}

void cont_func(void *, void *) { printf("%s\n", resp.buf); }

void run_erpc_client() {
    std::string client_uri = kClientHostname + ":" + std::to_string(kUDPPort);
    erpc::Nexus nexus(client_uri, 0, 0);

    client_rpc = new erpc::Rpc<erpc::CTransport>(&nexus, nullptr, 0, sm_handler);

    std::string server_uri = kServerHostname + ":" + std::to_string(kUDPPort);
    int session_num = client_rpc->create_session(server_uri, 0);

    while (!client_rpc->is_connected(session_num)) client_rpc->run_event_loop_once();

    req = client_rpc->alloc_msg_buffer_or_die(kMsgSize);
    resp = client_rpc->alloc_msg_buffer_or_die(kMsgSize);

    client_rpc->enqueue_request(session_num, kReqType, &req, &resp, cont_func, nullptr);
    client_rpc->run_event_loop(100);

    delete client_rpc;
}