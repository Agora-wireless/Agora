/**
 * @file mac_data_receiver.h
 * @brief Declaration file for the MacDataReceiver interface class
 */
#ifndef MAC_DATA_RECEIVER_H_
#define MAC_DATA_RECEIVER_H_

#include <cstddef>

/**
 * @brief The MacDataReceiver interface class
 */
class MacDataReceiver {
 public:
  virtual size_t Load(unsigned char *destination, size_t requested_bytes) = 0;

 protected:
  inline MacDataReceiver() = default;
  virtual ~MacDataReceiver() = default;
};

#endif  // MAC_DATA_RECEIVER_H_