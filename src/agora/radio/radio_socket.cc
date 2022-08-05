/**
 * @file radio_socket.cc
 * @brief Implementation file for the RadioSocket class.
 */
#include "radio_socket.h"

#include <cassert>
#include <chrono>

#include "logger.h"
#include "utils.h"

static constexpr size_t kMaxMTU = 9000;
//Could be dependant on the number of samples per frame * bytes per samples.
static constexpr size_t kRxBufferSize = 524288;
//This can be kRxBufferSize / 6 safely (1 sample per 6 bytes)
static constexpr size_t kRxSampleRemBufSize = 4096;
static constexpr bool kDebugIrisRx = false;

//#define SOCKET_DEBUG_OUTPUT
#if defined(SOCKET_DEBUG_OUTPUT)
#define DEBUG_OUTPUT(...)   \
  std::printf(__VA_ARGS__); \
  std::fflush(stdout)
#else
#define DEBUG_OUTPUT(...) ((void)0)
#endif

//#define MAX_TX_STATUS_DEPTH (64)

// #define RX_SOCKET_BUFFER_BYTES
//   (50 * 1024 * 1024)  // arbitrary and large PC buffer size

// #define ETHERNET_MTU (1500)  // L2 MTU without the 14-byte eth header
// #define ROUTE_HDR_SIZE (16)  // 128-bit transfer for routing header
// #define PADDED_ETH_HDR_SIZE
//   (16)  // 14 bytes + 2 bytes padding (holds size in bytes)
// #define IPv6_UDP_SIZE (40 + 8)  // 40 bytes of IPv6 + 8 bytes of UDP header
// #define TWBW_HDR_SIZE (sizeof(uint64_t) * 4)  // 4 transfers at 64-bits width

/// Iris tx status header flags
#define SEQUENCE_MASK (0xffff)
#define HAS_SEQUENCE_bp (16)
#define HAS_SEQUENCE_bf (uint64_t(1) << HAS_SEQUENCE_bp)
#define SEQUENCE_ERROR_bp (17)
#define SEQUENCE_ERROR_bf (uint64_t(1) << SEQUENCE_ERROR_bp)
#define HAS_STATUS_bp (18)
#define HAS_STATUS_bf (uint64_t(1) << HAS_STATUS_bp)
#define HAS_TIME_bp (19)
#define HAS_TIME_bf (uint64_t(1) << HAS_TIME_bp)
#define TIME_ERROR_bp (20)
#define TIME_ERROR_bf (uint64_t(1) << TIME_ERROR_bp)
#define UNDERFLOW_ERROR_bp (21)
#define UNDERFLOW_ERROR_bf (uint64_t(1) << UNDERFLOW_ERROR_bp)
#define BURST_END_bp (22)
#define BURST_END_bf (uint64_t(1) << BURST_END_bp)
#define OVERFLOW_ERROR_bp (23)
#define OVERFLOW_ERROR_bf (uint64_t(1) << OVERFLOW_ERROR_bp)

/// Iris rx header flags
#define SEQ_REQUEST_bp (25)
#define SEQ_REQUEST_bf (uint64_t(1) << SEQ_REQUEST_bp)
#define IS_TRIGGER_bp (26)
#define IS_TRIGGER_bf (uint64_t(1) << IS_TRIGGER_bp)
#define IS_BURST_bp (28)
#define IS_BURST_bf (uint64_t(1) << IS_BURST_bp)
#define RX_OVERFLOW_bp (29)
#define RX_OVERFLOW_bf (uint64_t(1) << RX_OVERFLOW_bp)
#define RX_TIME_ERROR_bp (30)
#define RX_TIME_ERROR_bf (uint64_t(1) << RX_TIME_ERROR_bp)
#define HAS_TIME_RX_bp (31)
#define HAS_TIME_RX_bf (uint64_t(1) << HAS_TIME_RX_bp)

// Packing this would probably be advisable
struct IrisCommData {
  uint64_t header_[2u];
  // Raw sample data, byte accessable
  uint8_t data_[];
} __attribute__((packed));
static_assert(sizeof(IrisCommData::header_) == 16);

///If Samples to Load > 0 we should stop trying to receive
///Otherwise, keep trying to get more data
static inline size_t ValidateSamples(long long& stream_rx_time,
                                     const long long& pkt_rx_time,
                                     size_t previous_samples,
                                     size_t new_samples,
                                     size_t requested_samples,
                                     size_t burst_samples, size_t output_dim) {
  const size_t completed_samples = previous_samples + new_samples;
  size_t samples_to_load = 0;
  //Check for sample discontinuity and short packets (samples < burst)
  if ((completed_samples < requested_samples) &&
      (new_samples != (burst_samples * output_dim))) {
    AGORA_LOG_WARN(
        "Rx packet does not have burst count number of symbols %zu:%zu "
        "with %zu available out of %zu requested\n",
        new_samples, (burst_samples * output_dim), completed_samples,
        requested_samples);
    //Load all of the samples including this rx
    samples_to_load = previous_samples + new_samples;
  } else if (stream_rx_time == 0) {
    //If stream rx time has not been set, then set for subsequent rx calls
    stream_rx_time = pkt_rx_time;
  } else if ((stream_rx_time + (long long)(previous_samples / output_dim)) !=
             pkt_rx_time) {
    AGORA_LOG_WARN(
        "Rx packet has discontinuous samples %lld pending samples %zu, "
        "new packet start time %lld, new samples %zu\n",
        stream_rx_time, previous_samples, pkt_rx_time, new_samples);
    //Load all samples availble previous to this rx
    samples_to_load = previous_samples;
  } else if (requested_samples <= completed_samples) {
    samples_to_load = requested_samples;
  } else {
    samples_to_load = 0;
  }
  return samples_to_load;
}

//bytes_per_element is based on "WIRE" format
//info.options = {SOAPY_SDR_CS16, SOAPY_SDR_CS12, SOAPY_SDR_CS8};
//info.optionNames = {"Complex int16", "Complex int12", "Complex int8"};
// this is also a bit more complex and needs to be reviewed for multiple cases
// 3 bytes == 1 Sample (8 bit + 4 bit) Real (8 bit + 4 bit) Img = 24 bits = 3 Bytes
RadioSocket::RadioSocket()
    : rx_buffer_(kRxBufferSize, std::byte(0)),
      rx_pkt_byte_count_(),
      sample_buffer_(kRxSampleRemBufSize, std::complex<int16_t>(0, 0)) {
  rx_buffer_.reserve(kRxBufferSize);
  sample_buffer_.reserve(kRxSampleRemBufSize);
  sample_buffer_.clear();
}

RadioSocket::~RadioSocket() { socket_.reset(); }

void RadioSocket::Create(size_t samples_per_symbol,
                         const std::string& local_addr,
                         const std::string& remote_addr,
                         const std::string& local_port,
                         const std::string& remote_port) {
  samples_per_symbol_ = samples_per_symbol;
  static constexpr size_t kSockBufSize = (1024 * 1024 * 64 * 8) - 1;
  socket_ = std::make_unique<UDPServer>(local_addr, local_port, kSockBufSize);

  //Creates a 1:1 connection for DATAGRAM sockets
  auto ret = socket_->Connect(remote_addr, remote_port);
  if (ret != 0) {
    throw std::runtime_error("RadioSocket::setupStream: Failed to connect to " +
                             remote_addr + " : " + remote_port);
  }
  AGORA_LOG_TRACE(
      "RadioSocket::Create: ip6_dst %s\n udp_dst %s with connect status %ld\n",
      socket_->Address().c_str(), socket_->Port().c_str(), ret);
}

//Function will return the number of requested samples (rx_samples) placed in out_data,  none,
// or less samples if the next samples become discontinuous.
int RadioSocket::RxSamples(std::vector<void*>& out_data, long long& rx_time_ns,
                           size_t req_samples_per_channel) {
  size_t loaded_samples_per_channel = 0;
  bool try_rx = true;
  //Total number of samples (per channel * num channels)
  size_t samples_to_load = 0;
  rx_time_ns = 0;

  const size_t num_channels = out_data.size();
  const size_t unpacked_samples = sample_buffer_.size();
  const size_t req_total_samples = req_samples_per_channel * num_channels;
  //Unpacked samples are the "oldest", samples per output dimension (not total samples)
  size_t samples_available = (unpacked_samples + rx_samples_);
  RtAssert((samples_available % num_channels) == 0,
           "Pending samples do not align with output dimensions");

  long long stream_rx_time;
  //Remove the 16...
  if (sample_buffer_.empty() == false) {
    stream_rx_time = rx_time_unpacked_;
  } else if (rx_bytes_ >= 16) {
    const auto* rx_data =
        reinterpret_cast<const IrisCommData*>(rx_buffer_.data());
    stream_rx_time = static_cast<long long>(rx_data->header_[1u]);
  } else {
    stream_rx_time = 0;
  }

  //Loop until we have enough samples or there is no pending data at the socket
  while (try_rx) {
    const size_t udp_rx_size = (rx_buffer_.size() - rx_bytes_);
    //If the requested rec is < the pending udp packet then the remainer will be thrown out without knowledge
    RtAssert(udp_rx_size >= kMaxMTU,
             "Requesting less samples than an MTU, could cause a truncated "
             "reception");
    //This function will return 1 UDP packet of up to the requested size.
    const int rx_return = socket_->Recv(&rx_buffer_.at(rx_bytes_), udp_rx_size);
    try_rx = false;

    if (rx_return > 0) {
      const size_t new_bytes = static_cast<size_t>(rx_return);
      rx_pkt_byte_count_.push(new_bytes);
      DEBUG_OUTPUT(
          "RadioSocket::RxSamples: Received %zu new bytes. Pending Total "
          "Samples (Packed %zu, Unpacked %zu)\n",
          new_bytes, rx_samples_, sample_buffer_.size());
      long long pkt_rx_time;
      size_t burst_count;
      const size_t new_rx_samples = InspectRx(
          &rx_buffer_.at(rx_bytes_), new_bytes, pkt_rx_time, burst_count);

      rx_bytes_ += new_bytes;
      rx_samples_ += new_rx_samples;
      RtAssert((new_rx_samples % num_channels) == 0,
               "Newly received samples do not align with output dimensions");

      //Modifies stream_rx_time for next call
      samples_to_load = ValidateSamples(
          stream_rx_time, pkt_rx_time, samples_available, new_rx_samples,
          req_total_samples, burst_count, num_channels);

      samples_available += new_rx_samples;
      if (samples_to_load == 0) {
        try_rx = true;
      }
    } else if (rx_return < 0) {
      if ((errno != EAGAIN) && (errno != EWOULDBLOCK)) {
        throw std::runtime_error("Error in socket receive call!");
      }
    }
  }  // end while (try_rx)

  if (samples_to_load > 0) {
    DEBUG_OUTPUT(
        "Samples Available %zu out of %zu requested. Transferring %zu with "
        "Pending Total Samples (Packed %zu, Unpacked %zu)\n",
        samples_available, req_total_samples, samples_to_load, rx_samples_,
        unpacked_samples);

    loaded_samples_per_channel =
        GetUnpackedSamples(out_data, rx_time_ns, samples_to_load);

    DEBUG_OUTPUT(
        "Loaded %zu Unpacked - Pending Samples (Packed %zu, Unpacked %zu)\n",
        loaded_samples_per_channel, rx_samples_, sample_buffer_.size());

    const size_t remaining_samples =
        (samples_to_load / num_channels) - loaded_samples_per_channel;
    loaded_samples_per_channel += GetPackedSamples(
        out_data, rx_time_ns, loaded_samples_per_channel, remaining_samples);

    DEBUG_OUTPUT(
        "Loaded %zu Packed - Pending Samples (Packed %zu, Unpacked %zu)\n",
        loaded_samples_per_channel, rx_samples_, sample_buffer_.size());
  }
  RtAssert(
      (loaded_samples_per_channel == 0) ||
          ((samples_to_load / num_channels) == loaded_samples_per_channel) ||
          (loaded_samples_per_channel == req_samples_per_channel),
      "The number of loaded samples is incorrect");
  return loaded_samples_per_channel;
}

///returns the number of processed_samples (unpacked / per channel) samples from the rx buffer
///this function unpackes the data from the byte buffer
size_t RadioSocket::UnpackSamples(std::vector<void*>& out_samples,
                                  size_t req_samples, long long& rx_time) {
  const size_t num_out_dims = out_samples.size();
  RtAssert((req_samples * num_out_dims) <= rx_samples_,
           "Unexpected and Invalid amount of rx_samples_");

  AGORA_LOG_TRACE(
      "UnpackSamples - Available (Bytes %zu Samples %zu). Samples Requested "
      "%zu Pending (Packed %zu, Unpacked %zu)\n",
      rx_bytes_, rx_samples_, req_samples * num_out_dims, rx_samples_,
      sample_buffer_.size());

  size_t processed_bytes = 0;
  size_t processed_samples = 0;

  //If req_samples > rx_samples_ ... exit with 0
  while ((processed_samples < req_samples) && (processed_bytes < rx_bytes_)) {
    // unpacker logic for twbw_rx_framer64
    const auto* rx_data =
        reinterpret_cast<const IrisCommData*>(&rx_buffer_.at(processed_bytes));
    const auto* payload = rx_data->data_;

    //return the first rx_time
    if (processed_samples == 0) {
      rx_time = static_cast<long long>(rx_data->header_[1u]);
    }

    if (rx_pkt_byte_count_.empty()) {
      AGORA_LOG_ERROR(
          "rx_pkt_byte_count_ queue empty when processing packets\n");
      throw std::runtime_error(
          "rx_pkt_byte_count_ queue empty when processing packets\n");
    }

    const auto pkt_bytes = rx_pkt_byte_count_.front();
    rx_pkt_byte_count_.pop();
    const size_t header_size = sizeof(IrisCommData::header_);
    RtAssert(pkt_bytes >= header_size,
             "Invalid RX size, less than a packet header");
    const size_t payload_bytes = pkt_bytes - header_size;
    const size_t pkt_samples =
        payload_bytes / (bytes_per_element_ * num_out_dims);
    RtAssert(((payload_bytes % (bytes_per_element_ * num_out_dims)) == 0),
             "Invalid payload size, contains a partial sample!");

    processed_bytes += sizeof(rx_data->header_);
    //Number of total samples through the end of this data parsing const size_t
    const size_t current_total_samples = processed_samples + pkt_samples;
    AGORA_LOG_TRACE(
        "UnpackSamples - Samples %zu:%zu header size %zu rx bytes %zu:%zu, "
        "Samples at end of pkt %zu\n",
        processed_samples, req_samples, sizeof(rx_data->header_),
        processed_bytes, rx_bytes_, current_total_samples);

    size_t ch = 0;
    //Maybe able to combine processed_bytes + pkt_byte_offset;
    size_t pkt_byte_offset = 0;
    while (processed_samples < current_total_samples) {
      // Optimization point (multi channel mode may not be trivial)
      // Unpack + Split
      const uint16_t i_lsb = uint16_t(payload[pkt_byte_offset]);
      const uint16_t split = uint16_t(payload[pkt_byte_offset + 1u]);
      const uint16_t q_msb = uint16_t(payload[pkt_byte_offset + 2u]);

      const std::complex<int16_t> new_sample =
          std::complex<int16_t>(int16_t((split << 12u) | (i_lsb << 4u)),
                                int16_t((q_msb << 8u) | (split & 0xf0)));

      //Output location is in the output vector
      if (processed_samples < req_samples) {
        auto* output_location = &static_cast<std::complex<int16_t>*>(
            out_samples.at(ch))[processed_samples];
        *output_location = new_sample;
      }
      //Too many samples, place them in the unpacked holding buffer
      else {
        if (sample_buffer_.empty()) {
          //Set the sample time of the first element
          rx_time_unpacked_ = rx_time + processed_samples;
        }
        sample_buffer_.emplace_back(new_sample);
      }
      ch++;
      if (ch == num_out_dims) {
        //processed_samples are per dimension (!total)
        processed_samples++;
        ch = 0;
      }
      pkt_byte_offset += bytes_per_element_;
      processed_bytes += bytes_per_element_;
      RtAssert(rx_bytes_ >= processed_bytes, "Exceeded rx byte count!");
    }  // end pkt
  }    // end (processed_samples < req_samples) && (processed_bytes < rx_bytes_)
  rx_bytes_ = rx_bytes_ - processed_bytes;
  rx_samples_ = rx_samples_ - (processed_samples * num_out_dims);
  AGORA_LOG_TRACE(
      "UnpackSamples - Rx Bytes %zu, Pending Samples (Packed %zu, Unpacked "
      "%zu)\n",
      rx_bytes_, rx_samples_, sample_buffer_.size());

  //Shift any remaining samples to the front of the rx buffer (this maybe inefficient)
  if (rx_bytes_ > 0) {
    if (rx_samples_ == 0) {
      AGORA_LOG_WARN(
          "Expected Rx samples, but didn't find any with %zu bytes\n",
          rx_bytes_);
      rx_bytes_ = 0;
      while (rx_pkt_byte_count_.empty() == false) {
        rx_pkt_byte_count_.pop();
      }
    } else {
      AGORA_LOG_WARN("Shifting %zu rx bytes to front of buffer\n", rx_bytes_);
      std::memmove(rx_buffer_.data(), &rx_buffer_.at(processed_bytes),
                   rx_bytes_);
    }
  }
  RtAssert(processed_samples >= req_samples,
           "All the requested symbols were not processed");
  return processed_samples;
}

// There is some redundant code between CheckSymbolComplete / UnpackSamples in looking at the UDP packet header.
bool RadioSocket::CheckSymbolComplete(const std::byte* in_data,
                                      const int& in_count) {
  bool finished;
  long long rx_time_ticks;
  size_t burst_count;

  size_t new_samples = InspectRx(in_data, in_count, rx_time_ticks, burst_count);
  //Sample counter / tracker
  const size_t start_sample = (rx_time_ticks & 0xFFFF);

  //This needs to be verified for !cfg_->HwFramer()
  if (start_sample > rx_samples_) {
    AGORA_LOG_WARN("***** Unexpected sample start received %zu:%zu *****\n",
                   start_sample, rx_samples_);
  } else if (start_sample < rx_samples_) {
    AGORA_LOG_ERROR("***** Unexpected sample start received %zu:%zu *****\n",
                    start_sample, rx_samples_);
    throw std::runtime_error("Unexpected sample start received");
  }
  rx_samples_ += new_samples;
  DEBUG_OUTPUT("Rx'd samples %zu:%zu\n", rx_samples_, samples_per_symbol_);

  RtAssert(rx_samples_ <= samples_per_symbol_,
           "Number of samples exceeds samples per symbol");

  RtAssert(false, "Needs to be adjusted to work with multichannel mode");
  if ((rx_samples_ == samples_per_symbol_) || (burst_count > new_samples)) {
    finished = true;
  } else {
    finished = false;
  }
  return finished;
}

void RadioSocket::Flush() {
  static constexpr float kTotalTimeoutSec = 5.0f;
  static constexpr float kRxTimeoutSec = 0.2f;
  int rx_return = 1;

  std::chrono::time_point<std::chrono::system_clock> last_rx_time;
  std::chrono::duration<float> rx_elapsed_seconds;
  std::chrono::duration<float> total_elapsed_seconds;

  const auto start_flush = std::chrono::system_clock::now();
  last_rx_time = start_flush;
  total_elapsed_seconds = last_rx_time - last_rx_time;
  rx_elapsed_seconds = total_elapsed_seconds;

  while ((rx_elapsed_seconds.count() < kRxTimeoutSec) &&
         (total_elapsed_seconds.count() < kTotalTimeoutSec)) {
    rx_return = socket_->Recv(&rx_buffer_.at(0), rx_buffer_.size());
    const auto time_now = std::chrono::system_clock::now();
    if (rx_return > 0) {
      AGORA_LOG_TRACE(
          "Flushing %d bytes from socket - elapsed %2.1f:%2.1f seconds\n",
          rx_return, rx_elapsed_seconds.count(), total_elapsed_seconds.count());
      last_rx_time = time_now;
    }
    rx_elapsed_seconds = time_now - last_rx_time;
    total_elapsed_seconds = time_now - start_flush;
  }

  if (total_elapsed_seconds.count() >= kTotalTimeoutSec) {
    AGORA_LOG_WARN("Flushing exceeded total timeout - elapsed %02.f seconds\n",
                   total_elapsed_seconds.count());
  }
  AGORA_LOG_TRACE("Flushing time %02.2f seconds\n",
                  total_elapsed_seconds.count());

  rx_bytes_ = 0;
  rx_samples_ = 0;
  while (rx_pkt_byte_count_.empty() == false) {
    rx_pkt_byte_count_.pop();
  }
  sample_buffer_.clear();
  rx_time_unpacked_ = 0;
}

//Returns the number of samples placed in 1 stream / channel (not total samples)
size_t RadioSocket::LoadSamples(std::vector<void*>& out_samples,
                                const std::complex<int16_t>* in_samples,
                                size_t num_in_samples) {
  size_t loaded_samples = 0;
  size_t channel = 0;
  const size_t total_channels = out_samples.size();

  if ((num_in_samples % total_channels) == 0) {
    loaded_samples = num_in_samples / total_channels;
    for (size_t ch_sample = 0; ch_sample < num_in_samples; ch_sample++) {
      std::complex<int16_t>* out_sample =
          &reinterpret_cast<std::complex<int16_t>*>(
              out_samples.at(channel))[ch_sample];
      *out_sample = in_samples[ch_sample];
      channel++;
      if (channel == total_channels) {
        channel = 0;
      }
    }
  } else {
    AGORA_LOG_ERROR(
        "Invalid number of samples requested %zu in RadioSocket::LoadSamples "
        "for number of streams %zu\n",
        num_in_samples, total_channels);
  }
  return loaded_samples;
}

//The in_data must be at least header size
//returns rx_time_ticks & sample count
//the sample count returned is the total sample count in the pkt,
//and not the sample count per channel / output dimension
size_t RadioSocket::InspectRx(const std::byte* in_data, size_t in_count,
                              long long& rx_time_ticks,
                              size_t& burst_count) const {
  // unpacker logic for twbw_rx_framer64
  const auto* rx_data = reinterpret_cast<const IrisCommData*>(in_data);
  const size_t header_size = sizeof(rx_data->header_);
  RtAssert(in_count > header_size, "Invalid RX size!");
  rx_time_ticks = static_cast<long long>(rx_data->header_[1u]);

  burst_count = size_t(rx_data->header_[0u] & 0xffff) + 1;
  const size_t payload_bytes = in_count - header_size;
  size_t total_sample_count = payload_bytes / bytes_per_element_;
  RtAssert(((payload_bytes % bytes_per_element_) == 0),
           "Invalid payload size!");

  if (kDebugIrisRx) {
    //Hardware framer only
    const size_t current_frame_id =
        static_cast<size_t>((rx_time_ticks >> 32u) & 0xFFFFFFFF);
    const size_t current_symbol_id =
        static_cast<size_t>((rx_time_ticks >> 16u) & 0xFFFF);

    const bool has_time = (rx_data->header_[0u] & HAS_TIME_RX_bf) != 0;
    const bool error_time = (rx_data->header_[0u] & RX_TIME_ERROR_bf) != 0;
    const bool error_overflow = (rx_data->header_[0u] & RX_OVERFLOW_bf) != 0;
    const bool is_burst = (rx_data->header_[0u] & IS_BURST_bf) != 0;
    const bool is_trigger = (rx_data->header_[0u] & IS_TRIGGER_bf) != 0;
    std::printf(
        "===========================================\n"
        "Received %zu bytes \n"
        "hdr0 %lx     hdr1 %lx\n"
        "Has Time %d  Time Error %d\n"
        "Overflow Error %d\n"
        "Is burst %d? Is Trigger %d?\n"
        "Burst Count   = %zu\n"
        "Payload bytes = %zu, samples = %zu\n"
        "Rx Time: %lld, Sample Start %lld\n"
        "Frame: %zu, Symbol: %zu\n"
        "===========================================\n",
        in_count, rx_data->header_[0u], rx_data->header_[1u],
        static_cast<int>(has_time), static_cast<int>(error_time),
        static_cast<int>(error_overflow), static_cast<int>(is_burst),
        static_cast<int>(is_trigger), burst_count, payload_bytes,
        total_sample_count, rx_time_ticks, (rx_time_ticks & 0xFFFF),
        current_frame_id, current_symbol_id);
  }
  return total_sample_count;
}

//Only set the rx_time if samples were loaded
size_t RadioSocket::GetUnpackedSamples(std::vector<void*>& out_samples,
                                       long long& rx_time,
                                       size_t req_total_samples) {
  size_t loaded_samples_per_channel;
  const size_t unpacked_samples = sample_buffer_.size();
  //Spots to be optimized....
  //Load the unpacked samples first
  if ((req_total_samples > 0) && (unpacked_samples > 0)) {
    const size_t transfer_samples =
        std::min(unpacked_samples, req_total_samples);
    loaded_samples_per_channel =
        LoadSamples(out_samples, sample_buffer_.data(), transfer_samples);

    rx_time = rx_time_unpacked_;
    DEBUG_OUTPUT("Loaded %zu:%zu samples from unpacked memory\n",
                 loaded_samples_per_channel, transfer_samples);

    if (req_total_samples > transfer_samples) {
      sample_buffer_.clear();
      rx_time_unpacked_ = 0;
    } else {
      //Remove the leading samples if some remain (shift left)
      //req_total_samples < transfer_samples
      //Setting to warning to see if this happens.
      AGORA_LOG_WARN(
          "Requested less samples than were previously unpacked %zu:%zu\n",
          req_total_samples, unpacked_samples);

      sample_buffer_.erase(sample_buffer_.begin(),
                           sample_buffer_.begin() + transfer_samples);

      //Adjust the rx_time to account for the transfered samples
      rx_time_unpacked_ += (transfer_samples / out_samples.size());
    }
  } else {
    loaded_samples_per_channel = 0;
  }
  return loaded_samples_per_channel;
}

//Operates on per channel sample counts
size_t RadioSocket::GetPackedSamples(std::vector<void*>& out_samples,
                                     long long& rx_time_out,
                                     size_t sample_offset, size_t req_samples) {
  size_t loaded_samples;
  if (req_samples > 0) {
    long long* rx_time;
    long long garbage;

    std::vector<void*>* output_locations;
    std::vector<void*> out_offset;
    if (sample_offset > 0) {
      //Adjust the output locations in case unpacked samples were already loaded
      out_offset.reserve(out_samples.size());
      for (auto& out_loc : out_samples) {
        out_offset.emplace_back(
            &static_cast<std::complex<int16_t>*>(out_loc)[sample_offset]);
      }
      output_locations = &out_offset;
      rx_time = &garbage;
    } else {
      //Only set the rx time if it has not already been set (above)
      output_locations = &out_samples;
      rx_time = &rx_time_out;
      RtAssert(rx_time_out == 0,
               "Rx time out expected to be 0 when sample offset == 0");
    }

    //This can be larger than the request
    const size_t processed_samples =
        UnpackSamples(*output_locations, req_samples, *rx_time);
    DEBUG_OUTPUT(
        "GetPackedSamples - Processed samples %zu loading %zu. Pending "
        "Samples (Packed %zu, Unpacked %zu)\n",
        processed_samples, req_samples, rx_samples_, sample_buffer_.size());
    RtAssert(processed_samples >= req_samples,
             "Loaded less than the requested samples, unexpected");
    loaded_samples = req_samples;
  } else {
    loaded_samples = 0;
  }
  return loaded_samples;
}