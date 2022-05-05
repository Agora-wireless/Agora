/**
 * @file radio_socket.cc
 * @brief Implementation file for the RadioSocket class.
 */
#include "radio_socket.h"

#include <cassert>

#include "logger.h"
#include "utils.h"

constexpr size_t kMaxMTU = 9000;
constexpr size_t kRxBufferSize = 8192;
//This can be kRxBufferSize / 6 safely (1 sample per 6 bytes)
constexpr size_t kRxSampleRemBufSize = 2048;

constexpr bool kDebugIrisRx = false;
//constexpr bool kDebugIrisTxStatus = false;

#define SOCKET_DEBUG_OUTPUT
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

//bytes_per_element is based on "WIRE" format
//info.options = {SOAPY_SDR_CS16, SOAPY_SDR_CS12, SOAPY_SDR_CS8};
//info.optionNames = {"Complex int16", "Complex int12", "Complex int8"};
// this is also a bit more complex and needs to be reviewed for multiple cases
// 3 bytes == 1 Sample (8 bit + 4 bit) Real (8 bit + 4 bit) Img = 24 bits = 3 Bytes
RadioSocket::RadioSocket()
    : rx_buffer_(kRxBufferSize, std::byte(0)),
      sample_buffer_(kRxSampleRemBufSize, std::complex<int16_t>(0, 0)),
      rx_time_unpacked_(0) {
  rx_buffer_.reserve(kRxBufferSize);
  sample_buffer_.reserve(kRxSampleRemBufSize);
}

void RadioSocket::Create(size_t samples_per_symbol,
                         const std::string& local_addr,
                         const std::string& remote_addr,
                         const std::string& local_port,
                         const std::string& remote_port) {
  samples_per_symbol_ = samples_per_symbol;
  socket_ = std::make_unique<UDPServerIPv6>(local_addr, local_port);

  //Creates a 1:1 connection for DATAGRAM sockets
  size_t ret = socket_->Connect(remote_addr, remote_port);
  if (ret != 0) {
    throw std::runtime_error("RadioSocket::setupStream: Failed to connect to " +
                             remote_addr + " : " + remote_port);
  }
  DEBUG_OUTPUT(" ip6_dst %s\n udp_dst %s\n", socket_->Address().c_str(),
               socket_->Port().c_str());
}

/// returns the number of samples inserted into out_data
/// out_data unpacked symbol data (complex<int16_t>)
/// rx_time_ns rx time from symbol header (t0 of symbol)
int RadioSocket::RxSymbol(std::vector<void*>& out_data, long long& rx_time_ns) {
  size_t num_samples = 0;
  bool try_rx = true;

  while (try_rx) {
    try_rx = false;
    const int rx_return = socket_->Recv(&rx_buffer_.at(rx_bytes_),
                                        (rx_buffer_.size() - rx_bytes_));

    if (rx_return > 0) {
      const size_t new_bytes = static_cast<size_t>(rx_return);
      DEBUG_OUTPUT("Received %zu bytes\n", new_bytes);
      const bool symbol_complete =
          CheckSymbolComplete(&rx_buffer_.at(rx_bytes_), new_bytes);
      rx_bytes_ += new_bytes;

      if (symbol_complete) {
        DEBUG_OUTPUT("Completed Symbol: %zu bytes\n", rx_bytes_);
        //Could be multiple UDP rx calls, all the udp packets + headers that make up a symbol
        //exist in the rx_buffer input.
        const size_t req_samples = std::min(samples_per_symbol_, rx_samples_);
        num_samples = GetRxSamples(out_data, req_samples, rx_time_ns);
        if ((rx_bytes_ != 0) || (rx_samples_ != 0)) {
          std::printf(
              "Unexpected Bytes %zu and Samples %zu Remaining. Requested %zu "
              "Actual %zu\n",
              rx_bytes_, rx_samples_, req_samples, num_samples);
        }
        rx_bytes_ = 0;
        rx_samples_ = 0;
      } else {
        //Keep receiving (rx data but not the end of a symbol)
        try_rx = true;
      }
    } else if (rx_return < 0) {
      if ((errno != EAGAIN) && (errno != EWOULDBLOCK)) {
        throw std::runtime_error("Error in socket receive call!");
      }
    }
  }  // end while (try_rx)
  return num_samples;
}

//Function will return the number of requested samples (rx_samples) placed in out_data OR none.
//it will not return less samples (it will hold them internally)
int RadioSocket::RxSamples(std::vector<void*>& out_data, long long& rx_time_ns,
                           size_t rx_samples) {
  size_t loaded_samples = 0;
  bool try_rx = true;
  rx_time_ns = 0;

  const size_t num_channels = out_data.size();
  const size_t unpacked_samples = sample_buffer_.size() / num_channels;
  //Unpacked samples are the "oldest"
  size_t samples_available = unpacked_samples + rx_samples_;

  //Loop until we have enough samples or there is no pending data at the socket
  while (try_rx && (samples_available < rx_samples)) {
    const size_t udp_rx_size = (rx_buffer_.size() - rx_bytes_);
    //If the requested rec is < the pending udp packet then the remainer will be thrown out without knowledge
    assert(udp_rx_size > kMaxMTU);
    //This function will return 1 UDP packet of up to the requested size.
    const int rx_return = socket_->Recv(&rx_buffer_.at(rx_bytes_), udp_rx_size);
    try_rx = false;

    if (rx_return > 0) {
      const size_t new_bytes = static_cast<size_t>(rx_return);
      DEBUG_OUTPUT("Received %zu bytes\n", new_bytes);
      long long pkt_rx_time;
      size_t burst_count;
      const size_t new_samples = InspectRx(&rx_buffer_.at(rx_bytes_), new_bytes,
                                           pkt_rx_time, burst_count);
      rx_bytes_ += new_bytes;
      rx_samples_ += new_samples;
      samples_available += new_samples;
      try_rx = true;
    } else if (rx_return < 0) {
      if ((errno != EAGAIN) && (errno != EWOULDBLOCK)) {
        throw std::runtime_error("Error in socket receive call!");
      }
    }
  }  // end while (try_rx)

  //If enough data, then load the output
  // and set rx_time_ns to the first time
  if (samples_available >= rx_samples) {
    //Spots to be optimized....
    //Load the unpacked samples first
    if (unpacked_samples) {
      const size_t transfer_samples = std::min(unpacked_samples, rx_samples);
      loaded_samples =
          LoadSamples(out_data, sample_buffer_.data(), transfer_samples);
      rx_time_ns = rx_time_unpacked_;

      //Remove the leading samples if some remain (shift left)
      if (transfer_samples < unpacked_samples) {
        const size_t erase_count = transfer_samples * num_channels;
        sample_buffer_.erase(sample_buffer_.begin(),
                             sample_buffer_.begin() + erase_count);
        if (sample_buffer_.size()) {
          rx_time_unpacked_ = 0;
        }
        //Should we adjust the rx_time by erase_count?
      }
    }

    //Get the remaining samples from the rx_buffer (unpacked + header)
    const size_t remaining_samples = rx_samples - loaded_samples;
    if (remaining_samples > 0) {
      long long rx_time;
      //Adjust the output locations
      std::vector<void*> out_offset;
      out_offset.reserve(out_data.size());
      for (auto& out_loc : out_data) {
        sample_buffer_.emplace_back(
            static_cast<std::complex<int16_t>*>(out_loc)[loaded_samples]);
      }
      //This can be larger than the request
      const size_t processed_samples =
          GetRxSamples(out_offset, remaining_samples, rx_time);
      RtAssert(processed_samples < remaining_samples,
               "Loaded less than the requested samples, unexpected");
      loaded_samples += remaining_samples;
      //Only set the rx time if it has not already been set (above)
      if (rx_time_ns == 0) {
        rx_time_ns = rx_time;
      }
    }
  }
  RtAssert((loaded_samples == 0) || (loaded_samples == rx_samples),
           "The number of loaded samples is incorrect");
  return loaded_samples;
}

size_t RadioSocket::GetRxSamples(std::vector<void*>& out_samples,
                                 size_t req_samples, long long& rx_time) {
  const size_t num_out_dims = out_samples.size();
  RtAssert(req_samples <= rx_samples_,
           "Unexpected and Invalid amount of rx_samples_");

  RtAssert(num_out_dims == 1,
           "Multichannel case needs tested for the number p=!");

  size_t processed_bytes = 0;
  size_t processed_samples = 0;

  //If req_samples > rx_samples_ ... exit with 0 but going to DBC
  while ((processed_samples < req_samples) && (processed_bytes < rx_bytes_)) {
    // unpacker logic for twbw_rx_framer64
    const auto* rx_data =
        reinterpret_cast<const IrisCommData*>(&rx_buffer_.at(processed_bytes));
    const auto* payload = rx_data->data_;

    //return the first rx_time
    if (processed_samples == 0) {
      rx_time = static_cast<long long>(rx_data->header_[1u]);
    }

    const size_t pkt_samples = (size_t(rx_data->header_[0u] & 0xffff) + 1);
    //Not sure if this is per channel or total??????????????????????????
    processed_bytes += sizeof(rx_data->header_);
    //Number of total samples through the end of this data parsing const size_t
    size_t current_total_samples = processed_samples + pkt_samples;
    std::printf(
        "Samples %zu:%zu header size %zu rx bytes %zu:%zu, Samples at end of "
        "pkt %zu\n",
        processed_samples, req_samples, sizeof(rx_data->header_),
        processed_bytes, rx_bytes_, current_total_samples);

    size_t ch = 0;
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
        if (sample_buffer_.size() == 0) {
          rx_time_unpacked_ = static_cast<long long>(rx_data->header_[1u]);
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
  //RtAssert(rx_bytes_ >= processed_bytes, "Exceeded rx byte count!");
  rx_bytes_ = rx_bytes_ - processed_bytes;
  rx_samples_ = rx_samples_ - req_samples;
  //Shift any remaining samples to the front of the rx buffer (this maybe inefficient)
  if (rx_bytes_ > 0) {
    std::printf("Shifting %zu rx bytes to front of buffer\n", rx_bytes_);
    std::memmove(rx_buffer_.data(), &rx_buffer_.at(processed_bytes), rx_bytes_);
  }
  RtAssert(processed_samples >= req_samples,
           "All the requested symbols were not processed");
  return processed_samples;
}

// There is some redundant code between CheckSymbolComplete / GetRxSamples in looking at the UDP packet header.
bool RadioSocket::CheckSymbolComplete(const std::byte* in_data,
                                      const int& in_count) {
  bool finished;
  long long rx_time_ticks;
  size_t burst_count;

  size_t new_samples = InspectRx(in_data, in_count, rx_time_ticks, burst_count);
  const size_t start_sample = (rx_time_ticks & 0xFFFF);

  //This needs to be verified for !cfg_->HwFramer()
  if (start_sample > rx_samples_) {
    DEBUG_OUTPUT("***** Unexpected sample start received %zu:%zu *****\n",
                 start_sample, rx_samples_);
    //rx_samples_ += start_sample;
  } else if (start_sample < rx_samples_) {
    DEBUG_OUTPUT("***** Unexpected sample start received %zu:%zu *****\n",
                 start_sample, rx_samples_);
    throw std::runtime_error("Unexpected sample start received");
  }
  rx_samples_ += new_samples;
  DEBUG_OUTPUT("Rx'd samples %zu:%zu\n", rx_samples_, samples_per_symbol_);

  RtAssert(rx_samples_ <= samples_per_symbol_,
           "Number of samples exceeds samples per symbol");

  if ((rx_samples_ == samples_per_symbol_) || (burst_count > new_samples)) {
    finished = true;
  } else {
    finished = false;
  }
  return finished;
}

void RadioSocket::Flush() {
  int rx_return = 1;

  while (rx_return > 0) {
    rx_return = socket_->Recv(&rx_buffer_.at(0), rx_buffer_.size());
  }
  rx_bytes_ = 0;
  rx_samples_ = 0;
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
        "Invalid number of samples requested %zu in "
        "RadioSocket::LoadSamples for number of streams %zu\n",
        num_in_samples, total_channels);
  }
  return loaded_samples;
}

//The in_data must be at least header size
//returns rx_time_ticks & sample count
size_t RadioSocket::InspectRx(const std::byte* in_data, size_t in_count,
                              long long& rx_time_ticks, size_t& burst_count) {
  // unpacker logic for twbw_rx_framer64
  const auto* rx_data = reinterpret_cast<const IrisCommData*>(in_data);
  const size_t header_size = sizeof(rx_data->header_);
  RtAssert(in_count > header_size, "Invalid RX size!");
  rx_time_ticks = static_cast<long long>(rx_data->header_[1u]);

  burst_count = size_t(rx_data->header_[0u] & 0xffff) + 1;
  const size_t payload_bytes = in_count - header_size;
  size_t sample_count = payload_bytes / bytes_per_element_;
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
        "hdr0 %lx\n"
        "hdr1 %lx\n"
        "Has Time %d Time Error %d\n"
        "Overflow Error %d\n"
        "Is burst %d? Is Trigger %d?\n"
        "Burst Count = %zu\n"
        "Payload bytes = %zu samples = %zu\n"
        "Rx Time: %lld\n"
        "Frame: %zu    Symbol: %zu\n"
        "===========================================\n",
        in_count, rx_data->header_[0u], rx_data->header_[1u],
        static_cast<int>(has_time), static_cast<int>(error_time),
        static_cast<int>(error_overflow), static_cast<int>(is_burst),
        static_cast<int>(is_trigger), burst_count, payload_bytes, sample_count,
        rx_time_ticks, current_frame_id, current_symbol_id);
  }
  return sample_count;
}

//Places the unpacked samples in a 1-dim array
//void Unpack24to32(const std::byte* in_data, int in_count,
//                  std::complex<int16_t>* out_samples, size_t out_count) {}