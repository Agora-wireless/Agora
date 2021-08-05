/**
 * @file file_receiver.cc
 * @brief Implementation file for the FileReceiver class
 */

#include "file_receiver.h"

#include <cassert>
#include <cstring>

static constexpr size_t kMaxReadAttempts = 2;

FileReceiver::FileReceiver(std::string &file_name)
    : file_name_(file_name), data_available_(0), data_start_offset_(0) {
  ///\todo Make sure that the file size is > FileReceiver::kFileStreamRxSize
  data_stream_.open(file_name_, (std::ifstream::in | std::ifstream::binary));
  assert(data_stream_.is_open() == true);
}

FileReceiver::~FileReceiver() {
  if (data_stream_.is_open() == true) {
    data_stream_.close();
  }
}

void FileReceiver::Load(char *destination, size_t num_load_bytes) {
  const size_t file_read_size =
      std::max(FileReceiver::kFileStreamRxSize, num_load_bytes);
  //Make sure the local buffer is large enough to perform the correct read
  assert(file_read_size <= local_rx_buffer_.size());
  assert(data_stream_.is_open() == true);
  if (num_load_bytes > data_available_) {
    //Check for potential local buffer wrap-around
    if ((data_available_ + data_start_offset_ + file_read_size) >
        local_rx_buffer_.size()) {
      memcpy(&local_rx_buffer_.at(0), &local_rx_buffer_.at(data_start_offset_),
             data_available_);
      data_start_offset_ = 0;
    }

    size_t data_read = 0;
    size_t read_attempts = 0;
    //Read until we have enough bytes, limit to kMaxReadAttempts
    while ((data_read < file_read_size) && (read_attempts < kMaxReadAttempts)) {
      data_stream_.read(reinterpret_cast<char *>(&local_rx_buffer_.at(
                            data_start_offset_ + data_available_ + data_read)),
                        file_read_size - data_read);

      data_read += data_stream_.gcount();
      std::printf("[FileReceiver] data received: %zu:%zu \n", data_read,
                  file_read_size);

      // Check for eof after read
      if (data_stream_.eof()) {
        std::printf(
            "[FileReceiver]: ***EndofFileStream - requested %zu read count "
            "%zu\n",
            file_read_size, data_read);
        data_stream_.close();
        data_stream_.open(file_name_,
                          std::ifstream::in | std::ifstream::binary);
      }

      //Check for errors on the stream
      if (!data_stream_) {
        throw std::runtime_error("[FileReceiver] data stream errors\n");
      }
      read_attempts++;
    }
    data_available_ += data_read;
  }

  if (num_load_bytes > data_available_) {
    throw std::runtime_error("[FileReceiver] failed to load enough data\n");
  } else {
    //Copy data from local buffer to requested memory location
    memcpy(destination, &local_rx_buffer_.at(data_start_offset_),
           num_load_bytes);
    std::printf("[Receive Server] Data loaded: %zu\n", num_load_bytes);
    data_start_offset_ += num_load_bytes;
    data_available_ -= num_load_bytes;
  }
}