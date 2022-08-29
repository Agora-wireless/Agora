/**
 * @file control_channel.h
 * @brief Declaration file for ControlChannel class
 */
#ifndef CONTROL_CHANNEL_H_
#define CONTROL_CHANNEL_H_

#include <cstddef>
#include <thread>

#include "control_message.h"
#include "udp_comm.h"

class ControlChannel {
 public:
  ControlChannel(std::string bs_ip_address, uint16_t bs_rx_port,
                 size_t rx_buffer_size, size_t tx_buffer_size);
  ~ControlChannel();

  void Start();
  void Stop();

 private:
  void BsControlServer();
  void HandleControlMessage(ControlMessage* control_message);
  //Member Variables
  UDPComm comm_;
  bool running_;
  bool term_;

  std::thread thread_;
};

#endif /* CONTROL_CHANNEL_H_ */