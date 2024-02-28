#ifndef RP_CONFIG_H_
#define RP_CONFIG_H_

/**
 * @brief The packet that contains the control information from RP and
 * requests/instructs Agora on cores usage/allocation.
 * TODO: this is a work in progress.
 */
class RPControlMsg {
 public:
  size_t msg_type_;
  size_t msg_arg_1_;
  size_t msg_arg_2_;
};

/**
 * @brief The packet that contains the status information from Agora and
 * informs RP.
 * TODO: this is a work in progress.
 */
class RPStatusMsg {
 public:
  size_t status_msg_0_;
  size_t status_msg_1_;
  size_t status_msg_2_;
};

#endif  // RP_CONFIG_H_
