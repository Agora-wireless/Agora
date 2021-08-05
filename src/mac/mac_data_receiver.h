/**
 * @file mac_data_receiver.h
 * @brief Declaration file for the MacDataReceiver interface class
 */
#ifndef MAC_DATA_RECEIVER_H_
#define MAC_DATA_RECEIVER_H_

#include <cstdint>

/**
 * @brief The MacDataReceiver interface class
 */
class MacDataReceiver {
 public:
  virtual void Load(char *destination, size_t num_load_bytes) = 0;

 protected:
  inline MacDataReceiver(){};
};

#endif  // MAC_DATA_RECEIVER_H_
