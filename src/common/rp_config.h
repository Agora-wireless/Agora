#ifndef RP_CONFIG_H_
#define RP_CONFIG_H_

/**
 * @brief The packet that contains the control information that tells
 * Agora how much resource to allocate.
 * TODO: this is a work in progress.
 */
class RPControlMsg {
 public:
  size_t add_core_;
  size_t remove_core_;
};

/**
 * @brief The packet that contains the status information that tells
 * remotely running Resource Provisioner (RP) how busy Agora is.
 * TODO: this is a work in progress.
 */
class RPStatusMsg {
 public:
  size_t latency_;
  size_t core_num_;
};

#endif  // RP_CONFIG_H_
