#ifndef RPC_CONTEXT
#define RPC_CONTEXT

#include "rpc.h"
#include <string>

static const std::string kServerHostname = "192.168.12.146";
static const std::string kClientHostname = "192.168.12.145";

static constexpr uint16_t kUDPPort = 31850;
static constexpr uint8_t kReqType = 2;
static constexpr size_t kMsgSize = 1024;

/**
 * @brief This class is used to ship Millipede tasks such as LDPC decoding
 * to remote servers
 */
class RPCContext {
public:

    /**
     * @brief Create an RPCContext class and initialize the eRPC object
     * 
     * @param local_uri The uri of the eRPC object to be created
     * @param obj_id The object ID of the eRPC object to be created
     */ 
    RPCContext(erpc::Nexus *nexus, size_t obj_id, void *info=nullptr, erpc::sm_handler_t sm_handler=nullptr);
    ~RPCContext();

    /**
     * @brief This function is used to poll events once in eRPC object, 
     * including requests, session management events, continuation events 
     */
    void poll_event();

    /**
     * @brief This function is used to create a session with a remote eRPC
     * object, which can be used to transmit data
     *
     * @param uri The uri of the remote eRPC object
     * @param obj_id The object ID of the remote eRPC object
     *
     * @return The function returns the session number (non-negative) for
     * sucess, and returns negative numbers when fails 
     */
    int connect(std::string uri, size_t obj_id);

    /**
     * @brief These two functions are used to send data in a session
     * 
     * @param session_num The session number in which we send data,
     * by default it is the dedicated session number
     * @param buf The pointer of the data
     * @param msg_len The length of the data to be sent
     * 
     * @return Returns 0 on success, and returns -1 on failure
     */
    int send(int session_num, char* buf, size_t msg_len, erpc::erpc_cont_func_t cont_func=nullptr, void *tag=nullptr);
    int send(char* buf, size_t msg_len, erpc::erpc_cont_func_t cont_func=nullptr, void *tag=nullptr);

    int respond(erpc::ReqHandle *req_handle, char *buf, size_t msg_len);
    int respond_without_copy(erpc::ReqHandle *req_handle, erpc::MsgBuffer *msg_buf);

    erpc::MsgBuffer alloc_msg_buffer(size_t max_data_size);

    uint8_t* get_resp_buf();
    size_t get_resp_buf_size();

    /**
     * @brief These two functions are used to check whether the session
     * is successfully created
     * 
     * @param session_num The session number we want to check, by default
     * it is the dedicated session number
     * 
     * @return Returns true if the session is successfully created
     */ 
    bool check_connection(int session_num);
    bool check_connection();

    /**
     * @brief Set the dedicated session number 
     */
    void set_dedicate_session(int session_num);

    void* get_info();

private:
    erpc::Rpc<erpc::CTransport>* rpc;
    std::vector<int> session_vec;
    erpc::MsgBuffer req_msgbuf;
    erpc::MsgBuffer resp_msgbuf;
    int dedicated_session;
    void *info;
};

#endif // RPC_CONTEXT