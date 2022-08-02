#include <gtest/gtest.h>

#include <atomic>
#include <string>
#include <thread>

#include "gettime.h"
#include "udp_client.h"
#include "udp_server.h"
#include "udp_server_ipv6.h"
#include "utils.h"

static constexpr size_t kReceivePort = 3185;
static constexpr size_t kMessageSize = 9000;
static constexpr size_t kNumPackets = 10000;
std::atomic<size_t> server_ready;

static const std::string kLocalHost = "localhost";
static const std::string kIpv6Address = "::1";

void ClientIpv4() {
  std::vector<uint8_t> packet(kMessageSize);
  UDPClient udp_client;

  while (server_ready == 0) {
    // Wait for server to get ready
  }

  for (size_t i = 1; i <= kNumPackets; i++) {
    static_assert(kMessageSize >= sizeof(size_t));
    *reinterpret_cast<size_t*>(&packet[0]) = i;
    udp_client.Send(kLocalHost, kReceivePort, &packet[0], kMessageSize);
  }
}

void ClientIpv6() {
  std::vector<uint8_t> packet(kMessageSize);
  UDPClient udp_client(kIpv6Address, 0);

  while (server_ready == 0) {
    // Wait for server to get ready
  }

  for (size_t i = 1; i <= kNumPackets; i++) {
    static_assert(kMessageSize >= sizeof(size_t));
    *reinterpret_cast<size_t*>(&packet[0]) = i;
    udp_client.Send(kIpv6Address, kReceivePort, &packet[0u], kMessageSize);
  }
}

// Spin until kNumPackets are received
void ServerIpv4() {
  double freq_ghz = GetTime::MeasureRdtscFreq();
  FastRand fast_rand;

  // Without buffer resizing, the server will sometimes drop packets and
  // therefore never return from this function
  UDPServer udp_server(kReceivePort, kMessageSize * kNumPackets);
  std::vector<uint8_t> pkt_buf(kMessageSize);

  server_ready = 1;
  size_t largest_pkt_index = 0;
  const size_t start_time = GetTime::Rdtsc();
  size_t num_pkts_received = 0;
  size_t num_pkts_reordered = 0;
  while (true) {
    const ssize_t ret = udp_server.Recv(&pkt_buf[0], kMessageSize);
    ASSERT_GE(ret, 0);
    if (ret != 0) {
      auto pkt_index = *reinterpret_cast<size_t*>(&pkt_buf[0]);
      if (pkt_index < largest_pkt_index) {
        num_pkts_reordered++;
      }
      largest_pkt_index = std::max(pkt_index, largest_pkt_index);
      num_pkts_received++;

      if (num_pkts_received == kNumPackets) {
        break;
      }
    }
  }
  server_ready = 0;

  std::printf(
      "Ipv4 Bandwidth = %.2f Gbps/s, number of reordered packets = %zu\n",
      (kNumPackets * kMessageSize * 8) /
          GetTime::CyclesToNs(GetTime::Rdtsc() - start_time, freq_ghz),
      num_pkts_reordered);
}

void ServerIpv6() {
  double freq_ghz = GetTime::MeasureRdtscFreq();
  FastRand fast_rand;

  // Without buffer resizing, the server will sometimes drop packets and
  // therefore never return from this function
  UDPServerIPv6 udp_server(kIpv6Address, kReceivePort,
                           kMessageSize * kNumPackets);
  std::vector<std::byte> pkt_buf(kMessageSize);

  server_ready = 1;
  size_t largest_pkt_index = 0;
  const size_t start_time = GetTime::Rdtsc();
  size_t num_pkts_received = 0;
  size_t num_pkts_reordered = 0;
  while (true) {
    const ssize_t ret = udp_server.Recv(&pkt_buf[0], kMessageSize);
    ASSERT_GE(ret, 0);
    if (ret != 0) {
      auto pkt_index = *reinterpret_cast<size_t*>(&pkt_buf[0]);
      if (pkt_index < largest_pkt_index) {
        num_pkts_reordered++;
      }
      largest_pkt_index = std::max(pkt_index, largest_pkt_index);
      num_pkts_received++;

      if (num_pkts_received == kNumPackets) {
        break;
      }
    }
  }
  server_ready = 0;

  std::printf(
      "Ipv6 Bandwidth = %.2f Gbps/s, number of reordered packets = %zu\n",
      (kNumPackets * kMessageSize * 8) /
          GetTime::CyclesToNs(GetTime::Rdtsc() - start_time, freq_ghz),
      num_pkts_reordered);
}

// Test bandwidth between client and server
TEST(UDPClientServer, PerfIpv4) {
  server_ready = 0;
  std::thread server_thread(ServerIpv4);
  std::thread client_thread(ClientIpv4);

  server_thread.join();
  client_thread.join();
}

TEST(UDPClientServer, PerfIpv6) {
  server_ready = 0;
  std::thread server_thread(ServerIpv6);
  std::thread client_thread(ClientIpv6);

  server_thread.join();
  client_thread.join();
}

// Test that the server is actually non-blocking
TEST(UDPClientServer, ServerIsNonBlocking) {
  UDPServer udp_server(kReceivePort);
  std::vector<uint8_t> packet(kMessageSize);

  // If the UDP server is blocking, this call never completes because there is
  // no data to receive
  ssize_t ret = udp_server.Recv(&packet[0], kMessageSize);
  ASSERT_EQ(ret, 0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
