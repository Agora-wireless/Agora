/**
 * @file file_receiver.h
 * @brief Declaration file for the FileReceiver class
 */
#ifndef FILE_RECEIVER_H_
#define FILE_RECEIVER_H_

#include <array>
#include <fstream>
#include <ios>
#include <iostream>
#include <string>

#include "mac_data_receiver.h"

/**
 * @brief The File Receiver class creates a binary file source for Agora
 */
class FileReceiver : public MacDataReceiver {
 public:
  static constexpr size_t kFileStreamRxSize = (2048u);
  static constexpr size_t kFileStreamLocalRxBufSize = (kFileStreamRxSize * 10u);

  explicit FileReceiver(std::string &file_name);
  ~FileReceiver() override;

  size_t Load(unsigned char *destination, size_t requested_bytes) final;

 private:
  std::string file_name_;
  std::ifstream data_stream_;
  std::array<uint8_t, FileReceiver::kFileStreamLocalRxBufSize> local_rx_buffer_;

  size_t data_available_;
  size_t data_start_offset_;
};

#endif  // FILE_RECEIVER_H_
