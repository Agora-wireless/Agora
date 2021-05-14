#include <gtest/gtest.h>

#include <atomic>
#include <thread>

#include "gettime.h"
#include "udp_client.h"
#include "udp_server.h"
#include "utils.h"

static constexpr size_t kServerUDPPort = 3185;
static constexpr size_t kMessageSize = 9000;
static constexpr size_t kNumPackets = 10000;
std::atomic<size_t> server_ready;

void ClientFunc() {
  std::vector<uint8_t> packet(kMessageSize);
  UDPClient udp_client;

  while (server_ready == 0) {
    // Wait for server to get ready
  }

  for (size_t i = 1; i <= kNumPackets; i++) {
    static_assert(kMessageSize >= sizeof(size_t));
    *reinterpret_cast<size_t*>(&packet[0]) = i;
    udp_client.Send("localhost", kServerUDPPort, &packet[0], kMessageSize);
  }
}

// Spin until kNumPackets are received
void ServerFunc() {
  double freq_ghz = GetTime::MeasureRdtscFreq();
  FastRand fast_rand;

  // Without buffer resizing, the server will sometimes drop packets and
  // therefore never return from this function
  UDPServer udp_server(kServerUDPPort, kMessageSize * kNumPackets);
  std::vector<uint8_t> pkt_buf(kMessageSize);

  server_ready = 1;
  size_t largest_pkt_index = 0;
  size_t start_time = GetTime::Rdtsc();
  size_t num_pkts_received = 0;
  size_t num_pkts_reordered = 0;
  while (true) {
    ssize_t ret = udp_server.Recv(&pkt_buf[0], kMessageSize);
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

  std::printf("Bandwidth = %.2f Gbps/s, number of reordered packets = %zu\n",
              (kNumPackets * kMessageSize * 8) /
                  GetTime::CyclesToNs(GetTime::Rdtsc() - start_time, freq_ghz),
              num_pkts_reordered);
}

// Test bandwidth between client and server
TEST(UDPClientServer, Perf) {
  server_ready = 0;
  std::thread server_thread(ServerFunc);
  std::thread client_thread(ClientFunc);

  server_thread.join();
  client_thread.join();
}

// Test that the server is actually non-blocking
TEST(UDPClientServer, ServerIsNonBlocking) {
  UDPServer udp_server(kServerUDPPort);
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
