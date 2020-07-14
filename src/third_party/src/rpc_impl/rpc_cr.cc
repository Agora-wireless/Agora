#include "rpc.h"

namespace erpc {

template <class TTr>
void Rpc<TTr>::enqueue_cr_st(SSlot *sslot, const pkthdr_t *req_pkthdr) {
  assert(in_dispatch());

  MsgBuffer *ctrl_msgbuf = &ctrl_msgbufs[ctrl_msgbuf_head];
  ctrl_msgbuf_head++;
  if (ctrl_msgbuf_head == 2 * TTr::kUnsigBatch) ctrl_msgbuf_head = 0;

  // Fill in the CR packet header. Avoid copying req_pkthdr's headroom.
  pkthdr_t *cr_pkthdr = ctrl_msgbuf->get_pkthdr_0();
  cr_pkthdr->req_type = req_pkthdr->req_type;
  cr_pkthdr->msg_size = 0;
  cr_pkthdr->dest_session_num = sslot->session->remote_session_num;
  cr_pkthdr->pkt_type = kPktTypeExplCR;
  cr_pkthdr->pkt_num = req_pkthdr->pkt_num;
  cr_pkthdr->req_num = req_pkthdr->req_num;
  cr_pkthdr->magic = kPktHdrMagic;

  enqueue_hdr_tx_burst_st(sslot, ctrl_msgbuf, nullptr);
}

template <class TTr>
void Rpc<TTr>::process_expl_cr_st(SSlot *sslot, const pkthdr_t *pkthdr,
                                  size_t rx_tsc) {
  assert(in_dispatch());
  assert(pkthdr->req_num <= sslot->cur_req_num);

  // Handle reordering
  if (unlikely(!in_order_client(sslot, pkthdr))) {
    ERPC_REORDER(
        "Rpc %u, lsn %u (%s): Received out-of-order CR. "
        "Packet %zu/%zu, sslot: %zu/%s. Dropping.\n",
        rpc_id, sslot->session->local_session_num,
        sslot->session->get_remote_hostname().c_str(), pkthdr->req_num,
        pkthdr->pkt_num, sslot->cur_req_num, sslot->progress_str().c_str());
    return;
  }

  // Update client tracking metadata
  if (kCcRateComp) update_timely_rate(sslot, pkthdr->pkt_num, rx_tsc);
  bump_credits(sslot->session);
  sslot->client_info.num_rx++;
  sslot->client_info.progress_tsc = ev_loop_tsc;

  // If we've transmitted all request pkts, there's nothing more to TX yet
  if (req_pkts_pending(sslot)) kick_req_st(sslot);  // credits >= 1
}

FORCE_COMPILE_TRANSPORTS

}  // namespace erpc
