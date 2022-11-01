/**
 * @file file_receiver.cc
 * @brief Implementation file for the FileReceiver class
 */

#include "file_receiver.h"

#include <cassert>
#include <cstring>

#include "logger.h"

static constexpr size_t kMaxReadAttempts = 2u;

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

size_t FileReceiver::Load(unsigned char *destination, size_t requested_bytes) {
  size_t loaded_bytes = 0;
  const size_t file_read_size =
      std::max(FileReceiver::kFileStreamRxSize, requested_bytes);
  // Make sure the local buffer is large enough to perform the correct read
  assert(file_read_size <= local_rx_buffer_.size());
  assert(data_stream_.is_open() == true);
  if (requested_bytes > data_available_) {
    // Check for potential local buffer wrap-around
    if ((data_available_ + data_start_offset_ + file_read_size) >
        local_rx_buffer_.size()) {
      std::memcpy(&local_rx_buffer_.at(0),
                  &local_rx_buffer_.at(data_start_offset_), data_available_);
      data_start_offset_ = 0;
    }

    size_t data_read = 0;
    size_t read_attempts = 0;
    // Read until we have enough bytes, limit to kMaxReadAttempts
    while ((data_read < file_read_size) && (read_attempts < kMaxReadAttempts)) {
      data_stream_.read(reinterpret_cast<char *>(&local_rx_buffer_.at(
                            data_start_offset_ + data_available_ + data_read)),
                        file_read_size - data_read);

      data_read += data_stream_.gcount();
      AGORA_LOG_FRAME("[FileReceiver] data received: %zu:%zu \n", data_read,
                      file_read_size);

      // Check for eof after read
      if (data_stream_.eof()) {
        AGORA_LOG_INFO(
            "[FileReceiver]: ***EndofFileStream - requested %zu read count "
            "%zu\n",
            file_read_size, data_read);
        data_stream_.close();
        data_stream_.open(file_name_,
                          std::ifstream::in | std::ifstream::binary);
      }

      // Check for errors on the stream
      if (!data_stream_) {
        throw std::runtime_error(
            "[FileReceiver] data stream errors - does the file exist?\n");
      }
      read_attempts++;
    }
    data_available_ += data_read;
  }

  if (requested_bytes > data_available_) {
    throw std::runtime_error("[FileReceiver] failed to load enough data\n");
  } else {
    // Copy data from local buffer to requested memory location
    std::memcpy(destination, &local_rx_buffer_.at(data_start_offset_),
                requested_bytes);
    AGORA_LOG_FRAME("[FileReceiver] data loaded: %zu\n", requested_bytes);
    data_start_offset_ += requested_bytes;
    data_available_ -= requested_bytes;
    loaded_bytes = requested_bytes;
  }
  return loaded_bytes;
}