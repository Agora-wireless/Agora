/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */

#include "dpdk_transport.hpp"

inline const struct rte_eth_conf port_conf_default()
{
    struct rte_eth_conf port_conf = rte_eth_conf();
    port_conf.rxmode.max_rx_pkt_len = JUMBO_FRAME_MAX_SIZE;
    port_conf.rxmode.offloads |= DEV_RX_OFFLOAD_JUMBO_FRAME;
    return port_conf;
}

int DpdkTransport::nic_init(
    uint16_t port, struct rte_mempool* mbuf_pool, int thread_num)
{
    struct rte_eth_conf port_conf = port_conf_default();
    const uint16_t rxRings = thread_num, txRings = 2 * thread_num;
    int retval;
    uint16_t q;
    uint16_t nb_rxd = RX_RING_SIZE;
    uint16_t nb_txd = TX_RING_SIZE;

    struct rte_eth_dev_info dev_info;
    struct rte_eth_rxconf rxconf;
    struct rte_eth_txconf txconf;

    if (rte_eth_dev_count_avail() < port)
        rte_exit(EXIT_FAILURE, "Not Enough NICs\n");

    if (!rte_eth_dev_is_valid_port(port))
        rte_exit(EXIT_FAILURE, "NIC ID is invalid\n");

    rte_eth_dev_set_mtu(port, 9000);
    uint16_t mtu_size = 0;
    rte_eth_dev_get_mtu(port, &mtu_size);
    printf("MTU: %d\n", mtu_size);

    int promiscuous_en = rte_eth_promiscuous_get(port);
    printf("Promiscuous mode: %d\n", promiscuous_en);
    rte_eth_promiscuous_enable(port);
    promiscuous_en = rte_eth_promiscuous_get(port);
    printf("Promiscuous mode: %d\n", promiscuous_en);

    rte_eth_dev_info_get(port, &dev_info);
    if (dev_info.tx_offload_capa & DEV_TX_OFFLOAD_MBUF_FAST_FREE)
        port_conf.txmode.offloads |= DEV_TX_OFFLOAD_MBUF_FAST_FREE;

    port_conf.rxmode.max_rx_pkt_len
        = RTE_MIN(dev_info.max_rx_pktlen, port_conf.rxmode.max_rx_pkt_len);
    // port_conf.rxmode.offloads |= DEV_RX_OFFLOAD_JUMBO_FRAME;

    retval = rte_eth_dev_configure(port, rxRings, txRings, &port_conf);
    if (retval != 0)
        return retval;
    printf("Max packet length: %d, dev max: %d\n",
        port_conf.rxmode.max_rx_pkt_len, dev_info.max_rx_pktlen);
    retval = rte_eth_dev_adjust_nb_rx_tx_desc(port, &nb_rxd, &nb_txd);
    if (retval != 0)
        return retval;

    rxconf = dev_info.default_rxconf;
    rxconf.offloads = port_conf.rxmode.offloads;
    uint32_t mbp_buf_size = rte_pktmbuf_data_room_size(mbuf_pool);
    printf("size of mbuf_pool: %u\n", mbp_buf_size);

    for (q = 0; q < rxRings; q++) {
        retval = rte_eth_rx_queue_setup(
            port, q, nb_rxd, rte_eth_dev_socket_id(port), &rxconf, mbuf_pool);
        if (retval < 0)
            return retval;
    }

    txconf = dev_info.default_txconf;
    txconf.offloads = port_conf.txmode.offloads;

    for (q = 0; q < txRings; q++) {
        retval = rte_eth_tx_queue_setup(
            port, q, nb_txd, rte_eth_dev_socket_id(port), &txconf);
        if (retval < 0)
            return retval;
    }

    retval = rte_eth_dev_start(port);
    if (retval < 0)
        return retval;

    struct rte_ether_addr addr;
    rte_eth_macaddr_get(port, &addr);
    printf("NIC %u MAC: %02" PRIx8 " %02" PRIx8 " %02" PRIx8 " %02" PRIx8
           " %02" PRIx8 " %02" PRIx8 " \n",
        port, addr.addr_bytes[0], addr.addr_bytes[1], addr.addr_bytes[2],
        addr.addr_bytes[3], addr.addr_bytes[4], addr.addr_bytes[5]);

    struct rte_eth_link link;
    rte_eth_link_get_nowait(port, &link);
    while (!link.link_status) {
        printf("Waiting for link up on NIC %" PRIu16 "\n", port);
        sleep(1);
        rte_eth_link_get_nowait(port, &link);
    }
    if (!link.link_status) {
        printf("Link down on NIC %" PRIx16 "\n", port);
        return 0;
    }

    return 0;
}

void DpdkTransport::fastMemcpy(void* pvDest, void* pvSrc, size_t nBytes)
{
    // printf("pvDest: 0x%lx, pvSrc: 0x%lx, Dest: %lx, Src,
    // %lx\n",intptr_t(pvDest), intptr_t(pvSrc), (intptr_t(pvDest) & 31),
    // (intptr_t(pvSrc) & 31) ); assert(nBytes % 32 == 0);
    // assert((intptr_t(pvDest) & 31) == 0);
    // assert((intptr_t(pvSrc) & 31) == 0);
    const __m256i* pSrc = reinterpret_cast<const __m256i*>(pvSrc);
    __m256i* pDest = reinterpret_cast<__m256i*>(pvDest);
    int64_t nVects = nBytes / sizeof(*pSrc);
    for (; nVects > 0; nVects--, pSrc++, pDest++) {
        const __m256i loaded = _mm256_stream_load_si256(pSrc);
        _mm256_stream_si256(pDest, loaded);
    }
    _mm_sfence();
}

void DpdkTransport::print_pkt(int src_ip, int dst_ip, uint16_t src_port,
    uint16_t dst_port, int len, int tid)
{
    uint8_t b[12];
    uint16_t sp, dp;

    b[0] = src_ip & 0xFF;
    b[1] = (src_ip >> 8) & 0xFF;
    b[2] = (src_ip >> 16) & 0xFF;
    b[3] = (src_ip >> 24) & 0xFF;
    b[4] = src_port & 0xFF;
    b[5] = (src_port >> 8) & 0xFF;
    sp = ((b[4] << 8) & 0xFF00) | (b[5] & 0x00FF);
    b[6] = dst_ip & 0xFF;
    b[7] = (dst_ip >> 8) & 0xFF;
    b[8] = (dst_ip >> 16) & 0xFF;
    b[9] = (dst_ip >> 24) & 0xFF;
    b[10] = dst_port & 0xFF;
    b[11] = (dst_port >> 8) & 0xFF;
    dp = ((b[10] << 8) & 0xFF00) | (b[11] & 0x00FF);
    printf("In RX thread %d: rx: %u.%u.%u.%u:%u -> %u.%u.%u.%u:%u (%d bytes)\n",
        tid, b[0], b[1], b[2], b[3], sp, b[6], b[7], b[8], b[9], dp, len);
}

struct rte_flow* DpdkTransport::generate_ipv4_flow(uint16_t port_id,
    uint16_t rx_q, uint32_t src_ip, uint32_t src_mask, uint32_t dest_ip,
    uint32_t dest_mask, uint16_t src_port, uint16_t src_port_mask,
    uint16_t dst_port, uint16_t dst_port_mask, struct rte_flow_error* error)
{
    struct rte_flow_attr attr;
    struct rte_flow_item pattern[4];
    struct rte_flow_action action[2];
    struct rte_flow* flow = NULL;
    struct rte_flow_action_queue queue = { .index = rx_q };
    struct rte_flow_item_ipv4 ip_spec;
    struct rte_flow_item_ipv4 ip_mask;
    struct rte_flow_item_udp udp_spec;
    struct rte_flow_item_udp udp_mask;
    struct rte_flow_item udp_item;
    int res;
    memset(pattern, 0, sizeof(pattern));
    memset(action, 0, sizeof(action));
    /*
     * set the rule attribute.
     * in this case only ingress packets will be checked.
     */
    memset(&attr, 0, sizeof(struct rte_flow_attr));
    attr.ingress = 1;
    attr.priority = 0;
    /*
     * create the action sequence.
     * one action only,  move packet to queue
     */
    action[0].type = RTE_FLOW_ACTION_TYPE_QUEUE;
    action[0].conf = &queue;
    action[1].type = RTE_FLOW_ACTION_TYPE_END;
    /*
     * set the first level of the pattern (ETH).
     * since in this example we just want to get the
     * ipv4 we set this level to allow all.
     */
    pattern[0].type = RTE_FLOW_ITEM_TYPE_ETH;

    /* the final level must be always type end */
    pattern[3].type = RTE_FLOW_ITEM_TYPE_END;
    /*
     * setting the second level of the pattern (IP).
     * in this example this is the level we care about
     * so we set it according to the parameters.
     */
    memset(&ip_spec, 0, sizeof(struct rte_flow_item_ipv4));
    memset(&ip_mask, 0, sizeof(struct rte_flow_item_ipv4));
    // ip_spec.hdr.next_proto_id = IPPROTO_UDP;
    // ip_mask.hdr.next_proto_id = 0xf; // protocol mask

    ip_spec.hdr.dst_addr = dest_ip; // htonl(dest_ip);
    ip_mask.hdr.dst_addr = dest_mask;
    ip_spec.hdr.src_addr = src_ip; // htonl(src_ip);
    ip_mask.hdr.src_addr = src_mask;

    pattern[1].type = RTE_FLOW_ITEM_TYPE_IPV4;
    pattern[1].spec = &ip_spec;
    pattern[1].mask = &ip_mask;

    udp_spec.hdr.src_port = src_port;
    udp_spec.hdr.dst_port = dst_port;
    udp_spec.hdr.dgram_len = 0;
    udp_spec.hdr.dgram_cksum = 0;

    udp_mask.hdr.src_port = src_port_mask;
    udp_mask.hdr.dst_port = dst_port_mask;
    udp_mask.hdr.dgram_len = 0;
    udp_mask.hdr.dgram_cksum = 0;

    udp_item.type = RTE_FLOW_ITEM_TYPE_UDP;
    udp_item.spec = &udp_spec;
    udp_item.mask = &udp_mask;
    udp_item.last = NULL;

    pattern[2] = udp_item;

    res = rte_flow_validate(port_id, &attr, pattern, action, error);
    if (!res)
        flow = rte_flow_create(port_id, &attr, pattern, action, error);
    return flow;
}
