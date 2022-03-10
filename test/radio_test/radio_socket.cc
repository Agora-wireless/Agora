/**
 * @file radio_socket.cc
 * @brief Implementation file for the RadioSocket class.
 */
#include "radio_socket.h"

#include "SoapyURLUtils.hpp"
#include "utils.h"

constexpr size_t kRxBufferSize = 8192;

//#define MAX_TX_STATUS_DEPTH (64)

// #define RX_SOCKET_BUFFER_BYTES
//   (50 * 1024 * 1024)  // arbitrary and large PC buffer size

// #define ETHERNET_MTU (1500)  // L2 MTU without the 14-byte eth header
// #define ROUTE_HDR_SIZE (16)  // 128-bit transfer for routing header
// #define PADDED_ETH_HDR_SIZE
//   (16)  // 14 bytes + 2 bytes padding (holds size in bytes)
// #define IPv6_UDP_SIZE (40 + 8)  // 40 bytes of IPv6 + 8 bytes of UDP header
// #define TWBW_HDR_SIZE (sizeof(uint64_t) * 4)  // 4 transfers at 64-bits width

constexpr size_t kDebugIrisRx = true;
//constexpr size_t kDebugIrisTxStatus = false;

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
};
static_assert(sizeof(IrisCommData::header_) == 16);

//bytes_per_element is based on "WIRE" format
//info.options = {SOAPY_SDR_CS16, SOAPY_SDR_CS12, SOAPY_SDR_CS8};
//info.optionNames = {"Complex int16", "Complex int12", "Complex int8"};
// this is also a bit more complex and needs to be reviewed for multiple cases
// 3 bytes == 1 Sample (8 bit + 4 bit) Real (8 bit + 4 bit) Img = 24 bits = 3 Bytes
RadioSocket::RadioSocket(size_t samples_per_symbol)
    : socket_(),
      rx_buffer_(kRxBufferSize, std::byte(0)),
      rx_bytes_(0),
      rx_samples_(0),
      samples_per_symbol_(samples_per_symbol),
      bytes_per_element_(3u) {}

void RadioSocket::Create(const std::string& address, const std::string& port) {
  const SoapyURL bindURL("udp", "::", "0");

  int ret = socket_.bind(bindURL.toString());
  if (ret != 0)
    throw std::runtime_error("Iris::setupStream: Failed to bind to " +
                             bindURL.toString() + ": " +
                             socket_.lastErrorMsg());
  const SoapyURL connect_url("udp", address, port);
  ret = socket_.connect(connect_url.toString());
  if (ret != 0)
    throw std::runtime_error("Iris::setupStream: Failed to connect to " +
                             connect_url.toString() + ": " +
                             socket_.lastErrorMsg());

  socket_.setNonBlocking(true);

  //lookup the local mac address to program the framer
  SoapyURL local_endpoint(socket_.getsockname());
  address_ = local_endpoint.getNode();
  port_ = local_endpoint.getService();

  std::printf(" ip6_dst %s\n udp_dst %s\n", address_.c_str(), port_.c_str());
}

int RadioSocket::RxSymbol(
    std::vector<std::vector<std::complex<int16_t>>>& out_data,
    long long& rx_time_ns) {
  size_t num_samples = 0;
  const int rx_return =
      socket_.recv(&rx_buffer_.at(rx_bytes_), (rx_buffer_.size() - rx_bytes_));

  if (rx_return > 0) {
    std::printf("Received %d bytes\n", rx_return);
    const size_t new_bytes = static_cast<size_t>(rx_return);
    const bool symbol_complete = CheckSymbolComplete(new_bytes);
    rx_bytes_ += new_bytes;

    if (symbol_complete) {
      std::printf("Completed Symbol: %zu bytes\n", rx_bytes_);
      num_samples = ParseRxSymbol(out_data, rx_time_ns);
      rx_bytes_ = 0;
      rx_samples_ = 0;
    }
  } else if (rx_return < 0) {
    if ((errno != EAGAIN) && (errno != EWOULDBLOCK)) {
      throw std::runtime_error("Error in scoket receive call!");
    }
  }
  return num_samples;
}

bool RadioSocket::CheckSymbolComplete(const size_t& bytes) {
  bool finished;
  // unpacker logic for twbw_rx_framer64
  const auto* rx_data =
      reinterpret_cast<const IrisCommData*>(&rx_buffer_.at(rx_bytes_));
  const long long rx_time_ticks = static_cast<long long>(rx_data->header_[1u]);

  const size_t burst_count = size_t(rx_data->header_[0u] & 0xffff) + 1;
  const size_t payload_bytes = size_t(bytes) - sizeof(rx_data->header_);

  const size_t samples = payload_bytes / bytes_per_element_;
  RtAssert(((payload_bytes % bytes_per_element_) == 0),
           "Invalid payload size!");

  if (kDebugIrisRx) {
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
        bytes, rx_data->header_[0u], rx_data->header_[1u], has_time, error_time,
        error_overflow, is_burst, is_trigger, burst_count, payload_bytes,
        samples, rx_time_ticks,
        static_cast<size_t>((rx_time_ticks >> 32u) & 0xFFFFFFFF),
        static_cast<size_t>((rx_time_ticks >> 16u) & 0xFFFF));
  }
  const size_t start_sample = (rx_time_ticks & 0xFFFF);
  if (start_sample > rx_samples_) {
    std::printf("***** Unexpected sample start received %zu:%zu *****\n",
                start_sample, rx_samples_);
    //rx_samples_ += start_sample;
  } else if (start_sample < rx_samples_) {
    std::printf("***** Unexpected sample start received %zu:%zu *****\n",
                start_sample, rx_samples_);
    std::fflush(stdout);
    throw std::runtime_error("Unexpected sample start received");
  }
  rx_samples_ += samples;
  std::printf("Rx'd samples %zu:%zu\n", rx_samples_, samples_per_symbol_);

  RtAssert(rx_samples_ <= samples_per_symbol_,
           "Number of samples exceeds samples per symbol");

  if ((rx_samples_ == samples_per_symbol_) || (burst_count > samples)) {
    finished = true;
  } else {
    finished = false;
  }
  return finished;
}

size_t RadioSocket::ParseRxSymbol(
    std::vector<std::vector<std::complex<int16_t>>>& out_samples,
    long long& rx_time_ns) {
  size_t processed_bytes = 0;
  size_t processed_samples = 0;

  //Assumes that leading packets are == burst_size samples
  while (processed_bytes < rx_bytes_) {
    std::printf("Processing bytes %zu:%zu\n", processed_bytes, rx_bytes_);

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
    //const size_t start_sample = (rx_time_ticks & 0xFFFF);
    //Burst count... (assumption)
    const size_t sample_count = size_t(rx_data->header_[0u] & 0xffff) + 1;

    size_t byte_offset = 0;
    size_t ch = 0;
    //Number of total samples through the end of this data parsing
    const size_t current_total_samples =
        std::min(processed_samples + sample_count, rx_samples_);
    RtAssert(((current_total_samples % out_samples.size()) == 0),
             "Unexpected number of received samples");
    std::printf("Current total samples %zu : %zu\n", current_total_samples,
                rx_samples_);
    fflush(stdout);
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
    std::printf("*****Processed Bytes %zu\n", processed_bytes);
  }
  RtAssert(processed_bytes == rx_bytes_,
           "Did not process the correct number of bytes");
  return processed_samples;
}