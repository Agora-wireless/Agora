/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */

#include "simple_mac_bs.hpp"

SimpleBSMac::SimpleBSMac(Config* cfg, size_t rx_thread_num, size_t core_offset)
    : rx_thread_num_(rx_thread_num)
    , core_id_(core_offset)
    , cfg(cfg)
{
    // TUN Interface
    ipbridge = new IPbridge();
    data_to_tun = new unsigned char[cfg->mac_data_bytes_num_perframe]();

    // CRC
    crc_up = new DoCRC();
}

SimpleBSMac::~SimpleBSMac() { delete cfg; }

bool SimpleBSMac::startRecv()
{
    std::vector<pthread_t> created_threads;

    for (size_t i = 0; i < rx_thread_num_; i++) {
        pthread_t recv_thread_;
        auto context = new EventHandlerContext<SimpleBSMac>;
        context->obj_ptr = this;
        context->id = i;
        if (pthread_create(&recv_thread_, NULL,
                pthread_fun_wrapper<SimpleBSMac, &SimpleBSMac::loopRecv>,
                context)
            != 0) {
            perror("Socket recv thread create failed");
            return false;
        }
        created_threads.push_back(recv_thread_);
    }
    while (!SignalHandler::gotExitSignal())
        ;
    return true;
}

void* SimpleBSMac::loopRecv(int tid)
{
    size_t core_offset = core_id_;
    pin_to_core_with_offset(ThreadType::kWorkerRX, core_offset, tid);

    int sock_buf_size = 1024 * 1024 * 64 * 8 - 1;
#if USE_IPV4
    struct sockaddr_in remote_addr;
    int socket_local
        = setup_socket_ipv4(cfg->mac_rx_port + tid, true, sock_buf_size);
    setup_sockaddr_remote_ipv4(
        &remote_addr, cfg->mac_tx_port + tid, cfg->tx_addr_to_mac.c_str());
    printf("Set up UDP socket server listening to port %d"
           " with remote address %s:%d  \n",
        cfg->mac_rx_port + tid, cfg->tx_addr_to_mac.c_str(),
        cfg->mac_tx_port + tid);
#else
    int socket_local
        = setup_socket_ipv6(cfg->ue_rx_port + tid, true, sock_buf_size);
    setup_sockaddr_remote_ipv6(
        &remote_addr, cfg->bs_port + tid, "fe80::f436:d735:b04a:864a");
#endif

    // loop recv
    socklen_t addrlen = sizeof(remote_addr);
    int packet_length = cfg->mac_packet_length;
    char* rx_buffer = (char*)malloc(packet_length);
    std::stringstream ss;
    std::stringstream ss1[cfg->mac_packets_perframe];
    while (true) {
        int ret = recvfrom(socket_local, rx_buffer, packet_length, 0,
            (struct sockaddr*)&remote_addr, &addrlen);
        if (ret == -1) {
            if (errno != EAGAIN) {
                perror("recv failed");
                exit(0);
            }
            return (NULL);
        }

        auto* pkt = (struct MacPacket*)rx_buffer;

        // Check CRC
        uint16_t crc
            = (uint16_t)(crc_up->calculateCRC24(
                             (unsigned char*)pkt->data, cfg->mac_payload_length)
                & 0xFFFF);
        if (crc == pkt->crc) {
            // CRC Good, valid packets -> upstream
            printf("Good Packet: CRC Check OK! \n");
            if (kDebugBSReceiver) {
                int symbol_id = pkt->symbol_id;
                // Read information from received packet
                if (symbol_id == 0) {
                    ss << "Header Info:\n"
                       << "FRAME_ID: " << pkt->frame_id
                       << "\nSYMBOL_ID: " << pkt->symbol_id
                       << "\nUE_ID: " << pkt->ue_id
                       << "\nVALID_TUN_DATA: " << pkt->valid_tun_data
                       << "\nDATLEN: " << pkt->datalen << "\nPAYLOAD:\n";
                }

                if (symbol_id >= 0 && symbol_id < cfg->mac_packets_perframe) {
                    for (size_t i = 0; i < cfg->mac_payload_length; i++) {

                        ss1[pkt->symbol_id]
                            << std::setfill('0') << std::setw(2) << std::right
                            << std::hex
                            << (uint32_t) * ((uint8_t*)pkt->data + i) << " ";
                        if (i % 16 == 15)
                            ss1[pkt->symbol_id] << "\n";
                    }
                    if (symbol_id == cfg->mac_packets_perframe - 1) {
                        for (size_t j = 0; j < cfg->mac_packets_perframe; j++) {
                            ss << ss1[j].str();
                            ss1[j].str(std::string());
                        }
                        ss << "\n" << std::endl;
                        std::cout << ss.str();
                        ss.str(std::string());
                    }
                }
            }
            if (cfg->ip_bridge_enable && pkt->valid_tun_data) {
                printf("Valid TUN Data, send up \n");
                // TODO : different data formats (pkt->data and data_to_tun)
                data_to_tun = (unsigned char*)pkt->data;
                ipbridge->write_fragment(
                    data_to_tun, cfg->data_bytes_num_perframe);
            }
        } else {
            printf("Bad Packet: CRC Check Failed! \n");
        }
    }
    return NULL;
}
