/**
 * @file radio_socket.cc
 * @brief Implementation file for the RadioSocket class.
 */
#include "radio_socket.h"

#include "utils.h"

constexpr size_t kRxBufferSize = 8192;

constexpr bool kDebugIrisRx = false;
//constexpr bool kDebugIrisTxStatus = false;

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

//bytes_per_element is based on "WIRE" format
//info.options = {SOAPY_SDR_CS16, SOAPY_SDR_CS12, SOAPY_SDR_CS8};
//info.optionNames = {"Complex int16", "Complex int12", "Complex int8"};
// this is also a bit more complex and needs to be reviewed for multiple cases
// 3 bytes == 1 Sample (8 bit + 4 bit) Real (8 bit + 4 bit) Img = 24 bits = 3 Bytes
RadioSocket::RadioSocket()
    : rx_buffer_(kRxBufferSize, std::byte(0)),
      rx_bytes_(0),
      rx_samples_(0),
      samples_per_symbol_(1),
      bytes_per_element_(3u) {}

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
/// out_data unpacked symbol dara
/// rx_time_ns rx time from symbol header (t0 of symbol)
int RadioSocket::RxSymbol(
    std::vector<std::vector<std::complex<int16_t>>>& out_data,
    long long& rx_time_ns) {
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
        num_samples = ParseRxSymbol(out_data, rx_time_ns);
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

// There is some redundant code between CheckSymbolComplete / ParseRxSymbol in looking at the UDP packet header.
bool RadioSocket::CheckSymbolComplete(const std::byte* in_data,
                                      const int& in_count) {
  bool finished;
  // unpacker logic for twbw_rx_framer64
  const auto* rx_data = reinterpret_cast<const IrisCommData*>(in_data);
  const long long rx_time_ticks = static_cast<long long>(rx_data->header_[1u]);

  const size_t burst_count = size_t(rx_data->header_[0u] & 0xffff) + 1;
  const size_t payload_bytes = size_t(in_count) - sizeof(rx_data->header_);
  const size_t samples = payload_bytes / bytes_per_element_;
  RtAssert(((payload_bytes % bytes_per_element_) == 0),
           "Invalid payload size!");

  const size_t current_frame_id =
      static_cast<size_t>((rx_time_ticks >> 32u) & 0xFFFFFFFF);
  const size_t current_symbol_id =
      static_cast<size_t>((rx_time_ticks >> 16u) & 0xFFFF);

  if (kDebugIrisRx) {
    const bool has_time = (rx_data->header_[0u] & HAS_TIME_RX_bf) != 0;
    const bool error_time = (rx_data->header_[0u] & RX_TIME_ERROR_bf) != 0;
    const bool error_overflow = (rx_data->header_[0u] & RX_OVERFLOW_bf) != 0;
    const bool is_burst = (rx_data->header_[0u] & IS_BURST_bf) != 0;
    const bool is_trigger = (rx_data->header_[0u] & IS_TRIGGER_bf) != 0;
    std::printf(
        "===========================================\n"
        "Received %d bytes \n"
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
        in_count, rx_data->header_[0u], rx_data->header_[1u], has_time,
        error_time, error_overflow, is_burst, is_trigger, burst_count,
        payload_bytes, samples, rx_time_ticks, current_frame_id,
        current_symbol_id);
  }
  const size_t start_sample = (rx_time_ticks & 0xFFFF);
  if (start_sample > rx_samples_) {
    DEBUG_OUTPUT("***** Unexpected sample start received %zu:%zu *****\n",
                 start_sample, rx_samples_);
    //rx_samples_ += start_sample;
  } else if (start_sample < rx_samples_) {
    DEBUG_OUTPUT("***** Unexpected sample start received %zu:%zu *****\n",
                 start_sample, rx_samples_);
    throw std::runtime_error("Unexpected sample start received");
  }
  rx_samples_ += samples;
  DEBUG_OUTPUT("Rx'd samples %zu:%zu\n", rx_samples_, samples_per_symbol_);

  RtAssert(rx_samples_ <= samples_per_symbol_,
           "Number of samples exceeds samples per symbol");

  if ((rx_samples_ == samples_per_symbol_) || (burst_count > samples)) {
    finished = true;
  } else {
    finished = false;
  }
  return finished;
}

//Unpacks the symbol data from the udp packets and data format exansion 24->32
size_t RadioSocket::ParseRxSymbol(
    std::vector<std::vector<std::complex<int16_t>>>& out_samples,
    long long& rx_time_ns) {
  size_t processed_bytes = 0;
  size_t processed_samples = 0;

  //Assumes that leading packets are == burst_size samples
  while (processed_bytes < rx_bytes_) {
    // unpacker logic for twbw_rx_framer64
    const auto* rx_data =
        reinterpret_cast<const IrisCommData*>(&rx_buffer_.at(processed_bytes));
    const auto* payload = rx_data->data_;

    const long long rx_time_ticks =
        static_cast<long long>(rx_data->header_[1u]);

    //Set time based on the first packet
    if (processed_bytes == 0) {
      rx_time_ns = rx_time_ticks;
    }

    //Probably best to shift the start sample (start_sample = (rx_time_ticks & 0xFFFF)) byt
    //then the return value will need to be a samples + offset (more complex)
    //Burst count... (assumption)
    const size_t sample_count = size_t(rx_data->header_[0u] & 0xffff) + 1;

    size_t byte_offset = 0;
    size_t ch = 0;
    //Number of total samples through the end of this data parsing
    const size_t current_total_samples =
        std::min(processed_samples + sample_count, rx_samples_);
    RtAssert(((current_total_samples % out_samples.size()) == 0),
             "Unexpected number of received samples");

    DEBUG_OUTPUT("Current total samples %zu : %zu\n", current_total_samples,
                 rx_samples_);

    // Optimization point (multi channel mode may not be trivial)
    while (processed_samples < current_total_samples) {
      const uint16_t i_lsb = uint16_t(payload[byte_offset]);
      const uint16_t split = uint16_t(payload[byte_offset + 1u]);
      const uint16_t q_msb = uint16_t(payload[byte_offset + 2u]);
      //Sign extend the 16 bit value (zero out lsb)
      out_samples.at(ch).at(processed_samples) =
          std::complex<int16_t>(int16_t((split << 12u) | (i_lsb << 4u)),
                                int16_t((q_msb << 8u) | (split & 0xf0)));
      ch++;
      if (ch == out_samples.size()) {
        ch = 0;
      }
      processed_samples++;
      byte_offset += bytes_per_element_;
    }

    auto rx_buffer_address = reinterpret_cast<uintptr_t>(rx_buffer_.data());
    auto current_address = reinterpret_cast<uintptr_t>(&payload[byte_offset]);
    if (rx_buffer_address > current_address) {
      processed_bytes = rx_buffer_address - current_address;
    } else {
      processed_bytes = current_address - rx_buffer_address;
    }
  }
  RtAssert(processed_bytes == rx_bytes_,
           "Did not process the correct number of bytes");
  return processed_samples;
}

void RadioSocket::Flush(void) {
  int rx_return = 1;

  while (rx_return > 0) {
    rx_return = socket_->Recv(&rx_buffer_.at(0), rx_buffer_.size());
  }
  rx_bytes_ = 0;
  rx_samples_ = 0;
}