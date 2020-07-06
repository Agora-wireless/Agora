#include "rpc_context.hpp"

RPCContext **ctx_list;

void basic_sm_handler(int session_num, erpc::SmEventType sm_event_type, erpc::SmErrType sm_err_type, void *_context) {
    auto *context = static_cast<RPCContext *>(_context);

    context->insert_session(session_num);

    printf("Connected session: %d\n", session_num);
}

void basic_req_handler(erpc::ReqHandle *req_handle, void * _context) {
    auto *context = static_cast<RPCContext *>(_context);

    // erpc::Rpc<erpc::CTransport>::resize_msg_buffer(&req_handle->pre_resp_msgbuf, kMsgSize);
    // sprintf(reinterpret_cast<char *>(req_handle->pre_resp_msgbuf.buf), "hello");

    // context->rpc->enqueue_response(req_handle, &req_handle->pre_resp_msgbuf);
    context->respond(req_handle, nullptr, 0);
}

void basic_cont_func(void *_context, void *_tag) {
    auto *context = static_cast<RPCContext *>(_context);

    // printf("RPC obj %d: %s\n", context->rpc->get_rpc_id(), context->resp_msgbuf.buf);
}

RPCContext::RPCContext(erpc::Nexus *nexus, size_t obj_id, void *info_, erpc::sm_handler_t sm_handler) {
    if (sm_handler == nullptr) {
        rpc = new erpc::Rpc<erpc::CTransport>(nexus, static_cast<void *>(this), obj_id, basic_sm_handler, 0);
    } else {
        rpc = new erpc::Rpc<erpc::CTransport>(nexus, static_cast<void *>(this), obj_id, sm_handler, 0);
    }
    rpc->retry_connect_on_invalid_rpc_id = true;
    req_msgbuf = rpc->alloc_msg_buffer_or_die(kMsgSize);
    resp_msgbuf = rpc->alloc_msg_buffer_or_die(kMsgSize);

    info = info_;
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

    return session_num;
}

int RPCContext::send(int session_num, char *buf, size_t msg_len, erpc::erpc_cont_func_t cont_func, void *tag) {
    bool found = false;
    for (const auto& num : session_vec) {
        if (num == session_num) {
            found = true;
            break;
        }
    }
    if (!found) {
        printf("Session %d not found\n", session_num);
        return -1;
    }
    rpc->resize_msg_buffer(&req_msgbuf, msg_len);
    if (buf) {
        memcpy(req_msgbuf.buf, buf, msg_len);
    }

    if (cont_func == nullptr) {
        rpc->enqueue_request(session_num, kReqType, &req_msgbuf, &resp_msgbuf, basic_cont_func, tag);
    } else {
        rpc->enqueue_request(session_num, kReqType, &req_msgbuf, &resp_msgbuf, cont_func, tag);
    }
    return 0;
}

int RPCContext::send(char *buf, size_t msg_len, erpc::erpc_cont_func_t cont_func, void *tag) {
    bool found = false;
    for (const auto& num : session_vec) {
        if (num == dedicated_session) {
            found = true;
            break;
        }
    }
    if (!found) {
        printf("Session %d not found\n", dedicated_session);
        return -1;
    }
    rpc->resize_msg_buffer(&req_msgbuf, msg_len);
    if (buf) {
        memcpy(req_msgbuf.buf, buf, msg_len);
    }
    
    if (cont_func == nullptr) {
        rpc->enqueue_request(dedicated_session, kReqType, &req_msgbuf, &resp_msgbuf, basic_cont_func, tag);
    } else {
        rpc->enqueue_request(dedicated_session, kReqType, &req_msgbuf, &resp_msgbuf, cont_func, tag);
    }
    return 0;
}

int RPCContext::respond(erpc::ReqHandle *req_handle, char *buf, size_t msg_len) {
    req_handle->dyn_resp_msgbuf = rpc->alloc_msg_buffer(msg_len);
    if (buf) {
        memcpy(req_handle->dyn_resp_msgbuf.buf, buf, msg_len);
    }

    rpc->enqueue_response(req_handle, &req_handle->dyn_resp_msgbuf);
    return 0;
}

int RPCContext::respond_without_copy(erpc::ReqHandle *req_handle, erpc::MsgBuffer *msg_buf) {
    rpc->enqueue_response(req_handle, msg_buf);
    return 0;
}

erpc::MsgBuffer RPCContext::alloc_msg_buffer(size_t max_data_size) {
    return rpc->alloc_msg_buffer_or_die(max_data_size);
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

uint8_t* RPCContext::get_resp_buf() {
    return resp_msgbuf.buf;
}
    
size_t RPCContext::get_resp_buf_size() {
    return resp_msgbuf.get_data_size();
}

void* RPCContext::get_info() {
    return info;
}

void RPCContext::insert_session(int session_num) {
    session_vec.push_back(session_num);
}

bool RPCContext::check_session(int session_num) {
    for (const auto session : session_vec) {
        if (session == session_num) {
            return true;
        }
    }
    return false;
}