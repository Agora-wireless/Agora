#include "rpc_sock.hpp"

// erpc::Rpc<erpc::CTransport> *server_rpc;
// erpc::Rpc<erpc::CTransport> *client_rpc;
// erpc::MsgBuffer req;
// erpc::MsgBuffer resp;

RPCContext **ctx_list;

// void server_req_handler(erpc::ReqHandle *req_handle, void *) {
//     auto &resp = req_handle->pre_resp_msgbuf;
//     server_rpc->resize_msg_buffer(&resp, kMsgSize);
//     sprintf(reinterpret_cast<char *>(resp.buf), "hello");

//     server_rpc->enqueue_response(req_handle, &resp);
// }

// void run_erpc_server() {
//     std::string server_uri = kServerHostname + ":" + std::to_string(kUDPPort);
//     erpc::Nexus nexus(server_uri, 0, 0);
//     nexus.register_req_func(kReqType, server_req_handler);

//     server_rpc = new erpc::Rpc<erpc::CTransport>(&nexus, nullptr, 0, nullptr);
//     server_rpc->run_event_loop(100000);
// }

// void run_erpc_client() {
//     std::string client_uri = kClientHostname + ":" + std::to_string(kUDPPort);
//     erpc::Nexus nexus(client_uri, 0, 0);

//     client_rpc = new erpc::Rpc<erpc::CTransport>(&nexus, nullptr, 0, sm_handler);

//     std::string server_uri = kServerHostname + ":" + std::to_string(kUDPPort);
//     int session_num = client_rpc->create_session(server_uri, 0);

//     while (!client_rpc->is_connected(session_num)) client_rpc->run_event_loop_once();

//     req = client_rpc->alloc_msg_buffer_or_die(kMsgSize);
//     resp = client_rpc->alloc_msg_buffer_or_die(kMsgSize);

//     client_rpc->enqueue_request(session_num, kReqType, &req, &resp, cont_func, nullptr);
//     client_rpc->run_event_loop(100);

//     delete client_rpc;
// }

void sm_handler(int session_num, erpc::SmEventType sm_event_type, erpc::SmErrType sm_err_type, void *_context) {
    auto *context = static_cast<RPCContext *>(_context);

    
}

void req_handler(erpc::ReqHandle *req_handle, void * _context) {
    auto *context = static_cast<RPCContext *>(_context);

    erpc::Rpc<erpc::CTransport>::resize_msg_buffer(&req_handle->pre_resp_msgbuf, kMsgSize);
    sprintf(reinterpret_cast<char *>(req_handle->pre_resp_msgbuf.buf), "hello");

    context->rpc->enqueue_response(req_handle, &req_handle->pre_resp_msgbuf);
}

void app_cont_func(void *_context, void *_tag) {
    auto *context = static_cast<RPCContext *>(_context);

    printf("RPC obj %d: %s\n", context->rpc->get_rpc_id(), context->resp_msgbuf.buf);
}

RPCContext::RPCContext(std::string local_uri, size_t obj_id) {
    nexus = new erpc::Nexus(local_uri, 0, 0);
    nexus->register_req_func(kReqType, req_handler);
    rpc = new erpc::Rpc<erpc::CTransport>(nexus, static_cast<void *>(this), obj_id, sm_handler, 0);
    rpc->retry_connect_on_invalid_rpc_id = true;
    req_msgbuf = rpc->alloc_msg_buffer_or_die(kMsgSize);
    resp_msgbuf = rpc->alloc_msg_buffer_or_die(kMsgSize);
}

void RPCContext::poll_event() {
    // while (true) {
        rpc->run_event_loop_once();
    // }
}

int RPCContext::connect(std::string uri, size_t obj_id) {
    int session_num = rpc->create_session(uri, obj_id);
    if (session_num < 0) {
        return session_num;
    }
    session_vec.push_back(session_num);

    return session_num;
}

int RPCContext::send(int session_num, char *buf, size_t msg_len) {
    bool found = false;
    for (const auto& num : session_vec) {
        if (num == session_num) {
            found = true;
            break;
        }
    }
    if (!found) {
        return -1;
    }
    if (buf) {
        memcpy(req_msgbuf.buf, buf, msg_len);
    }
    rpc->enqueue_request(session_num, kReqType, &req_msgbuf, &resp_msgbuf, app_cont_func, nullptr);
    return 0;
}

int RPCContext::send(char *buf, size_t msg_len) {
    bool found = false;
    for (const auto& num : session_vec) {
        if (num == dedicated_session) {
            found = true;
            break;
        }
    }
    if (!found) {
        return -1;
    }
    if (buf) {
        memcpy(req_msgbuf.buf, buf, msg_len);
    }
    rpc->enqueue_request(dedicated_session, kReqType, &req_msgbuf, &resp_msgbuf, app_cont_func, nullptr);
    return 0;
}

void RPCContext::set_dedicate_session(int session_num) {
    dedicated_session = session_num;
}

bool RPCContext::check_connection(int session_num) {
    return rpc->is_connected(session_num);
}

bool RPCContext::check_connection() {
    return rpc->is_connected(dedicated_session);
}