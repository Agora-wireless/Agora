/**
 * @file txrx_worker_usrp.cc
 * @brief Implementation of PacketTxRx datapath functions for communicating
 * with usrp hardware
 */

#include "txrx_worker_usrp.h"

#include <cassert>

#include "gettime.h"
#include "logger.h"
#include "message.h"

static constexpr size_t kFirstBeaconFrameAdvance = 400;
static constexpr size_t kBeaconFrameAdvance = 0;

TxRxWorkerUsrp::TxRxWorkerUsrp(
    size_t core_offset, size_t tid, size_t radio_hi, size_t radio_lo,
    Config* const config, size_t* rx_frame_start,
    moodycamel::ConcurrentQueue<EventData>* event_notify_q,
    moodycamel::ConcurrentQueue<EventData>* tx_pending_q,
    moodycamel::ProducerToken& tx_producer,
    moodycamel::ProducerToken& notify_producer,
    std::vector<RxPacket>& rx_memory, std::byte* const tx_memory,
    std::mutex& sync_mutex, std::condition_variable& sync_cond,
    std::atomic<bool>& can_proceed, RadioUHDConfig& radio_config)
    : TxRxWorker(core_offset, tid, radio_hi, radio_lo, config->NumChannels(),
                 config, rx_frame_start, event_notify_q, tx_pending_q,
                 tx_producer, notify_producer, rx_memory, tx_memory, sync_mutex,
                 sync_cond, can_proceed),
      radio_config_(radio_config) {}

TxRxWorkerUsrp::~TxRxWorkerUsrp() = default;

//Main Thread Execution loop
void TxRxWorkerUsrp::DoTxRx() {
  PinToCoreWithOffset(ThreadType::kWorkerTXRX, core_offset_, tid_);

  AGORA_LOG_INFO("TxRxWorkerUsrp[%zu] has %zu:%zu total radios %zu\n", tid_,
                 interface_offset_, (interface_offset_ + num_interfaces_) - 1,
                 num_interfaces_);

  /// For multi-uhd    num_interfaces_ = 1. (all radios are handled with 1 rx call)
  /// Determine the number of channels (A + B) / radios
  /// cfg_->NumRadios() * cfg_->NumChannels()?
  const size_t number_bs_radios = Configuration()->NumChannels();
  // const size_t number_bs_radios = 1;

  //Allocate a tx vector of zeros;
  //Overallocated by 1 (replaced by the beacon)
  std::vector<std::vector<std::complex<short>>> tx_zero_memory(
      number_bs_radios,
      std::vector<std::complex<short>>(Configuration()->SampsPerSymbol(),
                                       std::complex<short>(0, 0)));
  std::vector<void*> tx_locs(number_bs_radios);
  for (size_t i = 0; i < number_bs_radios; i++) {
    if (i == 0) {
      tx_locs.at(i) = Configuration()->BeaconCi16().data();
    } else {
      tx_locs.at(i) = tx_zero_memory.at(i).data();
    }
  }

  std::vector<void*> rx_locs;
  std::vector<std::vector<std::complex<short>>> rx_ignore_memory(
      number_bs_radios,
      std::vector<std::complex<short>>(Configuration()->SampsPerSymbol()));
  for (size_t i = 0; i < number_bs_radios; i++) {
    rx_locs.emplace_back(rx_ignore_memory.at(i).data());
  }

  rx_time_bs_ = 0;
  tx_time_bs_ = 0;
  const size_t radio_id = 0;

  running_ = true;
  WaitSync();

  //Should this be end rx?
  Radio::RxFlags rx_flags = Radio::RxFlags::kRxFlagNone;
  std::cout << "Sync BS host and FGPA timestamp..." << std::endl;
  auto rx_status = radio_config_.RadioRx(radio_id, rx_locs,
                                         Configuration()->SampsPerSymbol(),
                                         rx_flags, rx_time_bs_);
  std::cout << "First Rx Status " << rx_status << std::endl;
  if (rx_status <= 0) {
    AGORA_LOG_WARN("DoTxRx: Rx status is unexpected %zu\n", rx_status);
  }

  // Schedule the first beacon in the future
  //Assumes there is only 1 beacon and it is the first symbol... fix this when time allows
  tx_time_bs_ = rx_time_bs_ + ((Configuration()->SampsPerSymbol() *
                                Configuration()->Frame().NumTotalSyms()) *
                               kFirstBeaconFrameAdvance);
  std::cout << "delay time is: "
            << Configuration()->SampsPerSymbol() *
                   Configuration()->Frame().NumTotalSyms() * kBeaconFrameAdvance
            << std::endl;
  std::cout << "Tx Time BS: (tx_time_bs) " << tx_time_bs_ << std::endl;
  auto tx_status = radio_config_.RadioTx(
      radio_id, tx_locs.data(), Radio::TxFlags::kEndTransmit, tx_time_bs_);
  if (rx_status <= 0) {
    AGORA_LOG_WARN("DoTxRx: Tx status is unexpected %zu\n", tx_status);
  }

  //Wait until the radio is about to tx the beacon
  //todo -- Verify there shouldn't be a minus 1 here................
  const size_t read_symbols =
      Configuration()->Frame().NumTotalSyms() * kFirstBeaconFrameAdvance;
  for (size_t i = 0; i < read_symbols; i++) {
    rx_status = radio_config_.RadioRx(radio_id, rx_locs,
                                      Configuration()->SampsPerSymbol(),
                                      rx_flags, rx_time_bs_);

    if (rx_status <= 0) {
      AGORA_LOG_WARN("DoTxRx:Rx status is unexpected %zu\n", rx_status);
    }
  }

  // rx_time_bs_ should be 1 symbol / slot time before the tx beacon time;
  if (rx_time_bs_ >= tx_time_bs_) {
    AGORA_LOG_ERROR("Rx time is greater than the tx beacon time");
    throw std::runtime_error("Rx time is greater than the tx beacon time");
  }

  //Schedule tx beacons advanced.
  for (size_t i = 0; i < kBeaconFrameAdvance; i++) {
    //Schedule kBeaconFrameAdvance -1 frames ahead
    tx_time_bs_ += (Configuration()->SampsPerSymbol() *
                    Configuration()->Frame().NumTotalSyms());
    const int tx_ret = radio_config_.RadioTx(
        radio_id, tx_locs.data(), Radio::TxFlags::kEndTransmit, tx_time_bs_);
    if (tx_ret != static_cast<int>(Configuration()->SampsPerSymbol())) {
      AGORA_LOG_ERROR("BAD Transmit(%d:%zu) at Time %lld and frame count %zu\n",
                      tx_ret, Configuration()->SampsPerSymbol(), tx_time_bs_,
                      i);
    }
  }
  AGORA_LOG_INFO("Start BS main recv loop...\n");

  size_t global_frame_id = 0;
  size_t global_symbol_id = 0;
  size_t local_interface = 0;

  int count = 0;
  while (Configuration()->Running()) {
    // receive data
    RecvEnqueue(local_interface, global_frame_id, global_symbol_id, rx_locs,
                count);

    //if rx is successful than update the counter / times
    // Schedule the next beacon
    if (global_symbol_id == Configuration()->Frame().GetBeaconSymbol(0)) {
      tx_time_bs_ = rx_time_bs_ + Configuration()->SampsPerSymbol() *
                                      Configuration()->Frame().NumTotalSyms() *
                                      kBeaconFrameAdvance;
      std::cout << "delay time is: "
                << Configuration()->SampsPerSymbol() *
                       Configuration()->Frame().NumTotalSyms() *
                       kBeaconFrameAdvance
                << std::endl;
      const int tx_ret = radio_config_.RadioTx(
          radio_id, tx_locs.data(), Radio::TxFlags::kEndTransmit, tx_time_bs_);
      std::cout << "TX BS time  (tx_time_bs): " << tx_time_bs_ << std::endl;
      if (tx_ret != static_cast<int>(Configuration()->SampsPerSymbol())) {
        AGORA_LOG_ERROR(
            "BAD Transmit(%d:%zu) at Time %lld and frame count %zu\n", tx_ret,
            Configuration()->SampsPerSymbol(), tx_time_bs_, global_frame_id);
      }
    }

    // If we received the first symbol from the first interface
    if (kIsWorkerTimingEnabled && (local_interface == 0) &&
        (global_symbol_id == 0)) {
      rx_frame_start_[global_frame_id % kNumStatsFrames] = GetTime::Rdtsc();
    }

    local_interface++;
    if (local_interface == num_interfaces_) {
      local_interface = 0;
      // Update global frame_id and symbol_id
      global_symbol_id++;
      if (global_symbol_id == Configuration()->Frame().NumTotalSyms()) {
        global_symbol_id = 0;
        global_frame_id++;
      }
    }  // interface rollover
  }
  running_ = false;
}

//RX data and pass the samples to the scheduler
std::vector<Packet*> TxRxWorkerUsrp::RecvEnqueue(
    size_t radio_id, size_t frame_id, size_t symbol_id,
    const std::vector<void*>& discard_locs, int& count) {
  std::vector<Packet*> rx_packets;
  std::vector<RxPacket*> memory_tracking;

  const size_t total_channels = discard_locs.size();
  const bool publish_symbol = Configuration()->IsPilot(frame_id, symbol_id) ||
                              Configuration()->IsUplink(frame_id, symbol_id);

  std::vector<void*> rx_locs(total_channels);
  // std::cout<<"in txrx worker usrp recv enqueue, total number of channel is: " << total_channels << std::endl;
  for (size_t ch = 0; ch < total_channels; ch++) {
    if (publish_symbol) {
      //Allocate memory if we care about the results
      RxPacket& rx = GetRxPacket();
      memory_tracking.push_back(&rx);
      rx_locs.at(ch) = rx.RawPacket()->data_;
    } else {
      rx_locs.at(ch) = discard_locs.at(ch);
    }
  }

  Radio::RxFlags rx_flags;
  const int rx_status = radio_config_.RadioRx(radio_id, rx_locs,
                                              Configuration()->SampsPerSymbol(),
                                              rx_flags, rx_time_bs_);
  // added to test
  count++;
  if (Configuration()->IsUplink(frame_id, symbol_id) == true &&
      count >= 1000000) {
    // added to test if recv is getting the correct samples:

    std::ofstream outfile;
    const std::string& file = "ul_buff1.dat";
    outfile.open(file.c_str(), std::ofstream::binary);
    if (outfile.is_open()) {
      std::cout << "opened" << std::endl;
      outfile.write((const char*)rx_locs.front(),
                    Configuration()->SampsPerSymbol() *
                        sizeof(std::vector<std::complex<int16_t>>));
    } else {
      std::cout << "not opened";
    }
    if (outfile.is_open()) {
      outfile.close();
    }
    exit(111);
  }

  if (rx_status == static_cast<int>(Configuration()->SampsPerSymbol())) {
    //Process the rx data
    if (publish_symbol) {
      std::cout << "this is a pilot or uplink symbol" << std::endl;
      const size_t ant_offset = radio_id * total_channels;
      for (size_t ch = 0; ch < total_channels; ++ch) {
        RxPacket& rx = *memory_tracking.at(ch);
        new (rx.RawPacket()) Packet(frame_id, symbol_id, 0, ant_offset + ch);
        rx_packets.push_back(rx.RawPacket());
        EventData rx_message(EventType::kPacketRX, rx_tag_t(rx).tag_);
        NotifyComplete(rx_message);
      }
    }
  } else {
    AGORA_LOG_ERROR(
        "TxRxWorkerUsrp::RecvEnqueue: Unexpected Rx return status %dn\n",
        rx_status);
    throw std::runtime_error(
        "TxRxWorkerUsrp::RecvEnqueue:Unexpected Rx return status");
  }
  return rx_packets;
}

//Tx data
int TxRxWorkerUsrp::DequeueSend() {
  auto events = GetPendingTxEvents(1);
  if (events.empty()) {
    return -1;
  } else if (events.size() > 1) {
    throw std::runtime_error("DequeueSend returned too many events");
    return -1;
  }
  EventData event = events.at(0);
  assert(event.event_type_ == EventType::kPacketTX);

  const size_t ant_id = gen_tag_t(event.tags_[0]).ant_id_;
  size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
  size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;

  const size_t offset =
      (Configuration()->GetTotalDataSymbolIdx(frame_id, symbol_id) *
       Configuration()->BsAntNum()) +
      ant_id;

  symbol_id += Configuration()->UeAntNum();
  frame_id += TX_FRAME_DELTA;

  const size_t n_channels = Configuration()->NumChannels();
  const size_t radio_id = ant_id / n_channels;
  const size_t ch = ant_id % n_channels;

  std::vector<const void*> txbuffs(n_channels, nullptr);

  if (kDebugDownlink == true) {
    const size_t cell_id = Configuration()->CellId().at(radio_id);
    const std::vector<std::complex<int16_t>> zeros(
        Configuration()->SampsPerSymbol(), std::complex<int16_t>(0, 0));
    size_t dl_symbol_idx = Configuration()->Frame().GetDLSymbolIdx(symbol_id);
    if (ant_id != Configuration()->RefAnt(cell_id)) {
      txbuffs.at(ch) = zeros.data();
    } else if (dl_symbol_idx <
               Configuration()->Frame().ClientDlPilotSymbols()) {
      txbuffs.at(ch) =
          reinterpret_cast<void*>(Configuration()->UeSpecificPilotT()[0]);
    } else {
      txbuffs.at(ch) = reinterpret_cast<void*>(
          Configuration()
              ->DlIqT()[dl_symbol_idx -
                        Configuration()->Frame().ClientDlPilotSymbols()]);
    }
  } else {
    auto* pkt = GetTxPacket(frame_id, symbol_id, ant_id + ch);
    txbuffs.at(ch) = reinterpret_cast<void*>(pkt->data_);
  }

  size_t last = Configuration()->Frame().GetDLSymbolLast();
  Radio::TxFlags flags = (symbol_id != last) ? Radio::TxFlags::kTxFlagNone
                                             : Radio::TxFlags::kEndTransmit;
  long long frame_time = ((long long)frame_id << 32) | (symbol_id << 16);
  radio_config_.RadioTx(radio_id, txbuffs.data(), flags, frame_time);

  if (kDebugPrintInTask == true) {
    std::printf(
        "TxRxWorkerUsrp[%zu]: Transmitted frame %zu, symbol %zu, "
        "ant %zu, offset: %zu\n",
        tid_, frame_id, symbol_id, ant_id, offset);
  }

  auto complete_event = EventData(EventType::kPacketTX, event.tags_[0]);
  NotifyComplete(complete_event);
  return event.tags_[0];
}

int TxRxWorkerUsrp::DequeueSend(int frame_id, int symbol_id) {
  auto events = GetPendingTxEvents(1);
  if (events.empty()) {
    return -1;
  } else if (events.size() > 1) {
    throw std::runtime_error("DequeueSend returned too many events");
    return -1;
  }
  EventData event = events.at(0);
  std::cout << "DDD" << std::endl;
  assert(event.event_type_ == EventType::kPacketTX);

  const size_t ant_id = gen_tag_t(event.tags_[0]).ant_id_;
  // size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
  // size_t symbol_id = gen_tag_t(event.tags[0]).symbol_id;

  size_t offset = (Configuration()->GetTotalDataSymbolIdx(frame_id, symbol_id) *
                   Configuration()->BsAntNum()) +
                  ant_id;

  // symbol_id += Configuration()->UeAntNum();
  // frame_id += TX_FRAME_DELTA;

  std::cout << "ant_id: " << ant_id << ", frame_id: " << frame_id
            << ", symbol_id: " << symbol_id;
  std::cout << "offset: " << offset << std::endl;

  const size_t n_channels = Configuration()->NumChannels();
  const size_t radio_id = ant_id / n_channels;
  const size_t ch = ant_id % n_channels;

  std::vector<const void*> txbuffs(n_channels, nullptr);

  if (kDebugDownlink) {
    const size_t cell_id = Configuration()->CellId().at(radio_id);
    std::vector<std::complex<int16_t>> zeros(Configuration()->SampsPerSymbol());
    size_t dl_symbol_idx = Configuration()->Frame().GetDLSymbolIdx(symbol_id);
    if (ant_id != Configuration()->RefAnt(cell_id)) {
      txbuffs.at(ch) = zeros.data();
    } else if (dl_symbol_idx <
               Configuration()->Frame().ClientDlPilotSymbols()) {
      txbuffs.at(ch) =
          reinterpret_cast<void*>(Configuration()->UeSpecificPilotT()[0]);
    } else {
      txbuffs.at(ch) = reinterpret_cast<void*>(
          Configuration()
              ->DlIqT()[dl_symbol_idx -
                        Configuration()->Frame().ClientDlPilotSymbols()]);
    }
  } else {
    auto* pkt = GetTxPacket(frame_id, symbol_id, ant_id + ch);
    txbuffs.at(ch) = reinterpret_cast<void*>(pkt->data_);
  }

  const size_t last = Configuration()->Frame().GetDLSymbolLast();
  Radio::TxFlags flags = (symbol_id != static_cast<int>(last))
                             ? Radio::TxFlags::kTxFlagNone
                             : Radio::TxFlags::kEndTransmit;
  long long frame_time = ((long long)frame_id << 32) | (symbol_id << 16);
  radio_config_.RadioTx(radio_id, txbuffs.data(), flags, frame_time);

  if (kDebugPrintInTask) {
    std::printf(
        "TxRxWorkerUsrp[%zu]: Transmitted frame %d, symbol %d, "
        "ant %zu, offset: %zu\n",
        tid_, frame_id, symbol_id, ant_id, offset);
  }

  auto complete_event = EventData(EventType::kPacketTX, event.tags_[0]);
  NotifyComplete(complete_event);
  return event.tags_[0];
}
