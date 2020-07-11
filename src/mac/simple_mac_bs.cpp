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

    // Set up sockets to push MAC data to applications
    int data_sockets[cfg->UE_NUM];
    sockaddr_in data_addr[cfg->UE_NUM];

    for (size_t i = 0; i < cfg->UE_NUM; i++) {
        int port_id = 8090 + i;
        int remote_port_id = 8080 + i;
        std::string remote_addr = "127.0.0.1";
        data_sockets[i] = setup_socket_ipv4(port_id, true, sock_buf_size);
        setup_sockaddr_remote_ipv4(
            &data_addr[i], remote_port_id, remote_addr.c_str());
        printf("Setup (data) UDP socket listening to port %d with remote "
               "address %s:%d\n",
            port_id, remote_addr.c_str(), remote_port_id);
    }

#else
    int socket_local
        = setup_socket_ipv6(cfg->ue_rx_port + tid, true, sock_buf_size);
    setup_sockaddr_remote_ipv6(
        &remote_addr, cfg->bs_port + tid, "fe80::f436:d735:b04a:864a");
#endif

    // loop recv
    socklen_t addrlen = sizeof(remote_addr);
    int packet_length = cfg->data_bytes_num_persymbol;
    packet_length += MacPacket::kOffsetOfData;
    char* rx_buffer = (char*)malloc(packet_length);
    char* frame_data = (char*)malloc(cfg->mac_data_bytes_num_perframe);
    bool data2, data3;
    while (true) {
        auto* pkt = (struct MacPacket*)rx_buffer;
        int ret = recvfrom(socket_local, (char*)pkt, packet_length, 0,
            (struct sockaddr*)&remote_addr, &addrlen);
        if (ret == -1) {
            if (errno != EAGAIN) {
                perror("recv failed");
                exit(0);
            }
            return (NULL);
        }

        // create buffer for frame data
        if (pkt->symbol_id == 2) {
            memcpy(frame_data, (char*)pkt->data, cfg->data_bytes_num_persymbol);
            data2 = true;
        } else if (pkt->symbol_id == 3) {
            memcpy(((char*)frame_data + cfg->data_bytes_num_persymbol),
                (char*)pkt->data, cfg->data_bytes_num_persymbol);
            data3 = true;
        }

        // write packet data to data sockets
        if (data2 && data3) {
            ret = sendto(data_sockets[pkt->ue_id], frame_data,
                cfg->mac_data_bytes_num_perframe, 0,
                (struct sockaddr*)&data_addr[pkt->ue_id],
                sizeof(data_addr[pkt->ue_id]));
            if (ret == -1) {
                if (errno != EAGAIN) {
                    perror("data send failed");
                    exit(0);
                }
            }
            printf("received data for frame %d, ue %d, size %d\n",
                pkt->frame_id, pkt->ue_id, cfg->mac_data_bytes_num_perframe);
            for (size_t i = 0; i < cfg->mac_data_bytes_num_perframe; i++) {
                printf("%i ", *((uint8_t*)frame_data + i));
            }
            printf("\n");
            data2 = false;
            data3 = false;
        }

        if (kDebugBSReceiver) {
            // Read information from received packet
            printf("RX thread %d received frame %d symbol %d/%d, ue %d, size "
                   "%d\n ",
                tid, pkt->frame_id, pkt->symbol_id,
                cfg->ul_data_symbol_num_perframe, pkt->ue_id,
                packet_length - MacPacket::kOffsetOfData);
            if (pkt->symbol_id >= 2) {
                for (size_t i = 0; i < packet_length - MacPacket::kOffsetOfData;
                     i++) {
                    printf("%i ", *((uint8_t*)pkt->data + i));
                }
                printf("\n");
            }
        }
    }
    return NULL;
}
