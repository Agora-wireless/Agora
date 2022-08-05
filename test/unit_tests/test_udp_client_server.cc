#include <gtest/gtest.h>

#include <atomic>
#include <string>
#include <thread>

#include "gettime.h"
#include "udp_client.h"
#include "udp_comm.h"
#include "udp_server.h"
#include "utils.h"

static constexpr size_t kSendPort = 3195;
static constexpr size_t kReceivePort = 3185;
static constexpr size_t kMessageSize = 9000;
static constexpr size_t kNumPackets = 10;
static std::atomic<bool> server_ready = false;

static const std::string kIpv4Address = "127.0.0.1";
static const std::string kIpv6Address = "::1";

///Test the non-connection use case (SendTo)
void ClientSendTo(const std::string& src_address, uint16_t src_port,
                  const std::string& dest_address, uint16_t dest_port) {
  unused(src_port);
  std::vector<std::byte> packet(kMessageSize);
  //This will use the default socket type (currently ipv4)
  UDPClient udp_client(src_address);

  while (server_ready == false) {
    // Wait for server to get ready
  }

  for (size_t i = 1; i <= kNumPackets; i++) {
    static_assert(kMessageSize >= sizeof(size_t));
    *reinterpret_cast<size_t*>(&packet[0u]) = i;
    udp_client.Send(dest_address, dest_port, &packet[0u], kMessageSize);
  }
}

///Test the connection use case
void ClientConnect(const std::string& src_address, uint16_t src_port,
                   const std::string& dest_address, uint16_t dest_port) {
  std::vector<std::byte> packet(kMessageSize);
  ///Port will be selected by the system
  UDPClient udp_client(src_address, src_port);

  while (server_ready == false) {
    // Wait for server to get ready
  }
  udp_client.Connect(dest_address, dest_port);

  for (size_t i = 1; i <= kNumPackets; i++) {
    static_assert(kMessageSize >= sizeof(size_t));
    *reinterpret_cast<size_t*>(&packet[0u]) = i;
    udp_client.Send(&packet[0u], kMessageSize);
  }
}

///Test the connection use case
void UDPSend(const std::string& local_address, uint16_t local_port,
             const std::string& remote_address, uint16_t remote_port) {
  std::vector<std::byte> packet(kMessageSize);
  UDPComm udp_client(local_address, local_port, 0, 0);

  while (server_ready == false) {
    // Wait for server to get ready
  }
  udp_client.Connect(remote_address, remote_port);

  for (size_t i = 1; i <= kNumPackets; i++) {
    static_assert(kMessageSize >= sizeof(size_t));
    *reinterpret_cast<size_t*>(&packet[0u]) = i;
    udp_client.Send(&packet[0u], kMessageSize);
  }
}

// Spin until kNumPackets are received
void ServerRecvFrom(const std::string& src_address, uint16_t src_port,
                    const std::string& dest_address, uint16_t dest_port) {
  const double freq_ghz = GetTime::MeasureRdtscFreq();

  // Without buffer resizing, the server will sometimes drop packets and
  // therefore never return from this function
  UDPServer udp_server(dest_address, dest_port, kMessageSize * kNumPackets);
  std::vector<std::byte> pkt_buf(kMessageSize);

  server_ready = true;
  size_t largest_pkt_index = 0;
  const size_t start_time = GetTime::Rdtsc();
  size_t num_pkts_received = 0;
  size_t num_pkts_reordered = 0;
  while (true) {
    const ssize_t ret =
        udp_server.Recv(src_address, src_port, &pkt_buf[0u], kMessageSize);
    ASSERT_GE(ret, 0);
    if (ret != 0) {
      auto pkt_index = *reinterpret_cast<size_t*>(&pkt_buf[0u]);
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
  server_ready = false;
  std::printf("Bandwidth = %.2f Gbps/s, number of reordered packets = %zu\n",
              (kNumPackets * kMessageSize * 8) /
                  GetTime::CyclesToNs(GetTime::Rdtsc() - start_time, freq_ghz),
              num_pkts_reordered);
}

void ServerConnect(const std::string& src_address, const uint16_t src_port,
                   const std::string& dest_address, const uint16_t dest_port) {
  const double freq_ghz = GetTime::MeasureRdtscFreq();

  // Without buffer resizing, the server will sometimes drop packets and
  // therefore never return from this function
  UDPServer udp_server(dest_address, dest_port, kMessageSize * kNumPackets);
  std::vector<std::byte> pkt_buf(kMessageSize);
  //Optional method to create 1:1 connection
  udp_server.Connect(src_address, src_port);

  server_ready = true;
  size_t largest_pkt_index = 0;
  const size_t start_time = GetTime::Rdtsc();
  size_t num_pkts_received = 0;
  size_t num_pkts_reordered = 0;
  while (true) {
    const ssize_t ret = udp_server.Recv(&pkt_buf[0u], kMessageSize);
    ASSERT_GE(ret, 0);
    if (ret != 0) {
      auto pkt_index = *reinterpret_cast<size_t*>(&pkt_buf[0u]);
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
  server_ready = false;
  std::printf("Bandwidth = %.2f Gbps/s, number of reordered packets = %zu\n",
              (kNumPackets * kMessageSize * 8) /
                  GetTime::CyclesToNs(GetTime::Rdtsc() - start_time, freq_ghz),
              num_pkts_reordered);
}

///Test the connection use case
void UDPRecv(const std::string& local_address, uint16_t local_port,
             const std::string& remote_address, uint16_t remote_port) {
  const double freq_ghz = GetTime::MeasureRdtscFreq();

  // Without buffer resizing, the server will sometimes drop packets and
  // therefore never return from this function
  UDPComm udp_server(local_address, local_port, kMessageSize * kNumPackets, 0);
  std::vector<std::byte> pkt_buf(kMessageSize);
  //Optional method to create 1:1 connection
  udp_server.Connect(remote_address, remote_port);

  server_ready = true;
  size_t largest_pkt_index = 0;
  const size_t start_time = GetTime::Rdtsc();
  size_t num_pkts_received = 0;
  size_t num_pkts_reordered = 0;
  while (true) {
    const ssize_t ret = udp_server.Recv(&pkt_buf[0u], kMessageSize);
    ASSERT_GE(ret, 0);
    if (ret != 0) {
      auto pkt_index = *reinterpret_cast<size_t*>(&pkt_buf[0u]);
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
  server_ready = false;
  std::printf("Bandwidth = %.2f Gbps/s, number of reordered packets = %zu\n",
              (kNumPackets * kMessageSize * 8) /
                  GetTime::CyclesToNs(GetTime::Rdtsc() - start_time, freq_ghz),
              num_pkts_reordered);
}

// Test bandwidth between client and server
TEST(UDPClientServer, PerfIpv4) {
  server_ready = false;
  std::thread server_thread(ServerConnect, kIpv4Address, kSendPort,
                            kIpv4Address, kReceivePort);
  std::thread client_thread(ClientConnect, kIpv4Address, kSendPort,
                            kIpv4Address, kReceivePort);

  server_thread.join();
  client_thread.join();
}

TEST(UDPClientServer, PerfIpv6) {
  server_ready = false;
  std::thread server_thread(ServerConnect, kIpv6Address, kSendPort,
                            kIpv6Address, kReceivePort);
  std::thread client_thread(ClientConnect, kIpv6Address, kSendPort,
                            kIpv6Address, kReceivePort);

  server_thread.join();
  client_thread.join();
}

TEST(UDPClientServer, PerfIpv4RecvFrom) {
  server_ready = false;
  std::thread server_thread(ServerRecvFrom, kIpv4Address, kSendPort,
                            kIpv4Address, kReceivePort);
  std::thread client_thread(ClientSendTo, kIpv4Address, kSendPort, kIpv4Address,
                            kReceivePort);

  server_thread.join();
  client_thread.join();
}

TEST(UDPComm, PerfUDP) {
  server_ready = false;
  std::thread receive_thread(UDPRecv, kIpv6Address, kReceivePort, kIpv6Address,
                             kSendPort);
  std::thread send_thread(UDPSend, kIpv6Address, kSendPort, kIpv6Address,
                          kReceivePort);

  receive_thread.join();
  send_thread.join();
}

// Test that the server is actually non-blocking
TEST(UDPClientServer, ServerIsNonBlocking) {
  UDPServer udp_server(kIpv6Address, kReceivePort);
  std::vector<std::byte> packet(kMessageSize);

  // If the UDP server is blocking, this call never completes because there is
  // no data to receive
  const ssize_t ret = udp_server.Recv(&packet[0u], kMessageSize);
  ASSERT_EQ(ret, 0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
