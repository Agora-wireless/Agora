/**
 * @file txrx_worker_argos.cc
 * @brief Implementation of PacketTxRx datapath functions for communicating
 * with real Argos hardware
 */

#include "txrx_worker_argos.h"

#include <cassert>

#include "logger.h"

static constexpr bool kDebugDownlink = false;

TxRxWorkerArgos::TxRxWorkerArgos(
    size_t core_offset, size_t tid, size_t interface_count,
    size_t interface_offset, Config* const config, size_t* rx_frame_start,
    moodycamel::ConcurrentQueue<EventData>* event_notify_q,
    moodycamel::ConcurrentQueue<EventData>* tx_pending_q,
    moodycamel::ProducerToken& tx_producer,
    moodycamel::ProducerToken& notify_producer,
    std::vector<RxPacket>& rx_memory, std::byte* const tx_memory,
    RadioConfig* const radio_config)
    : TxRxWorker(core_offset, tid, interface_count, interface_offset, config,
                 rx_frame_start, event_notify_q, tx_pending_q, tx_producer,
                 notify_producer, rx_memory, tx_memory),
      radio_config_(radio_config) {}

TxRxWorkerArgos::~TxRxWorkerArgos() {}

//Main Thread Execution loop
void TxRxWorkerArgos::DoTxRx() {
  PinToCoreWithOffset(ThreadType::kWorkerTXRX, core_offset_, tid_);

  MLPD_INFO("TxRxWorkerArgos[%zu] has %zu:%zu total radios %zu\n", tid_,
            interface_offset_, (interface_offset_ + num_interfaces_) - 1,
            num_interfaces_);

  running_ = true;
  started_ = true;

  if (num_interfaces_ == 0) {
    MLPD_WARN("TxRxWorkerArgos[%zu] has no interfaces, exiting\n", tid_);
    running_ = false;
    return;
  }

  long long time0 = 0;
  // When Hw Framer is enabled, frame time keeping is done by hardware.
  // In particular, the Hw only forward useful receive (no Guard) symbols.
  // In this mode, the format of rx timestamp (64 bits)is frame number at
  // 32-bits msb and symbol number at 16-bit msb of 32-bit lsb.
  // When Hw Framer is disabled (false), frame time keeping in done entirely
  // in software as it is implemented here. All time domain symbols are transfered
  // to the host by HW where software will decide how to handle each symbol.
  if (Configuration()->HwFramer() == false) {
    const int kBeacontxFlags = 2;  // END BURST
    // Read some dummy symbols to get the hardware time and then schedule
    // TX/RX accordingly
    const size_t beacon_radio =
        Configuration()->BeaconAnt() / Configuration()->NumChannels();
    const size_t beacon_chan =
        Configuration()->BeaconAnt() % Configuration()->NumChannels();

    // Can probably remove the NumChannels dimension (just point to same location)
    const std::vector<std::vector<std::complex<int16_t>>> zeros(
        Configuration()->NumChannels(),
        std::vector<std::complex<int16_t>>(Configuration()->SampsPerSymbol(),
                                           std::complex<int16_t>(0, 0)));

    // prepare BS beacon in host buffer
    std::vector<const void*> beaconbuffs(Configuration()->NumChannels(),
                                         zeros.at(0).data());
    beaconbuffs.at(beacon_chan) = Configuration()->BeaconCi16().data();

    std::vector<std::vector<std::complex<int16_t>>> samp_buffer(
        Configuration()->NumChannels(),
        std::vector<std::complex<int16_t>>(Configuration()->SampsPerSymbol(),
                                           std::complex<int16_t>(0, 0)));

    long long rx_time_bs = 0;
    long long tx_time_bs = 0;
    size_t frame_time = Configuration()->SampsPerSymbol() *
                        Configuration()->Frame().NumTotalSyms();

    std::cout << "Sync BS host and FGPA timestamp..." << std::endl;

    //RX / TX all symbols in tx frame delta
    for (size_t frm = 0; frm < TX_FRAME_DELTA; frm++) {
      int rx_status = -1;
      int tx_status = -1;
      size_t rx_symbol = 0;
      //Handle First frame / symbol special
      if (frm == 0) {
        const size_t rx_radio_id = interface_offset_;
        while (rx_status < 0) {
          rx_status =
              radio_config_->RadioRx(rx_radio_id, samp_buffer, rx_time_bs);
        }
        //First Frame has been rx'd by the first radio
        tx_time_bs = rx_time_bs + frame_time * TX_FRAME_DELTA;
        time0 = tx_time_bs;
        std::cout << "Received first data at time " << rx_time_bs
                  << " on thread " << tid_ << std::endl;
        std::cout << "Transmit first beacon at time " << tx_time_bs
                  << " on thread " << tid_ << std::endl;

        //Finish rx'ing symbol 0 on remaining radios
        for (size_t radio_id = rx_radio_id + 1;
             radio_id < rx_radio_id + num_interfaces_; radio_id++) {
          rx_status = radio_config_->RadioRx(radio_id, samp_buffer, rx_time_bs);
          //---------------What to do about errors?
        }
        //Symbol complete
        rx_symbol++;
      } else {
        tx_time_bs += frame_time;
      }

      // Schedule the tx
      for (size_t radio_id = interface_offset_;
           radio_id < interface_offset_ + num_interfaces_; radio_id++) {
        if (radio_id == beacon_radio) {
          tx_status = radio_config_->RadioTx(radio_id, beaconbuffs.data(),
                                             kBeacontxFlags, tx_time_bs);
        } else {
          tx_status = radio_config_->RadioTx(radio_id, zeros, kBeacontxFlags,
                                             tx_time_bs);
        }
        if (tx_status != static_cast<int>(Configuration()->SampsPerSymbol())) {
          std::cerr << "BAD Transmit(" << tx_status << "/"
                    << Configuration()->SampsPerSymbol() << ") at Time "
                    << tx_time_bs << std::endl;
        }
      }  // end TX schedule

      // Rx all symbols on remaining radios
      for (size_t sym = rx_symbol;
           sym < Configuration()->Frame().NumTotalSyms(); sym++) {
        for (size_t radio_id = interface_offset_;
             radio_id < interface_offset_ + num_interfaces_; radio_id++) {
          rx_status = radio_config_->RadioRx(radio_id, samp_buffer, rx_time_bs);
          //---------------Check status?
        }
      }
    }
    std::cout << "Start BS main recv loop..." << std::endl;
  }  // HwFramer == false

  size_t rx_slot = 0;
  ssize_t prev_frame_id = -1;
  size_t local_interface = 0;

  size_t global_frame_id = 0;
  size_t global_symbol_id = 0;
  while (Configuration()->Running() == true) {
    if (0 == DequeueSend(time0)) {
      // receive data
      auto pkts = RecvEnqueue(local_interface, rx_slot, global_frame_id,
                              global_symbol_id);
      if (!pkts.empty()) {
        rx_slot = (rx_slot + pkts.size()) % rx_memory_.size();
        RtAssert(pkts.size() == channels_per_interface_,
                 "Received data but it was the wrong dimension");

        if (kIsWorkerTimingEnabled) {
          const auto frame_id = pkts.front()->frame_id_;
          if (frame_id > prev_frame_id) {
            rx_frame_start_[frame_id % kNumStatsFrames] = GetTime::Rdtsc();
            prev_frame_id = frame_id;
          }
        }
      }  // if (pkt.size() > 0)

      local_interface++;
      if (local_interface == num_interfaces_) {
        local_interface = 0;
        if (Configuration()->HwFramer() == false) {
          // Update global frame_id and symbol_id
          global_symbol_id++;
          if (global_symbol_id == Configuration()->Frame().NumTotalSyms()) {
            global_symbol_id = 0;
            global_frame_id++;
            if (Configuration()->Frame().NumDLSyms() == 0) {
              for (size_t interface = 0; interface < num_interfaces_;
                   interface++) {
                this->TxBeaconHw(global_frame_id, interface, time0);
              }
            }
          }
        }  // HwFramer == false
      }
    }  // DequeueSendArgos(time0) == 0
  }    // Configuration()->Running() == true
  running_ = false;
}

//RX data
std::vector<Packet*> TxRxWorkerArgos::RecvEnqueue(size_t interface_id,
                                                  size_t rx_slot,
                                                  size_t global_frame_id,
                                                  size_t global_symbol_id) {
  const size_t radio_id = interface_id + interface_offset_;
  const size_t ant_id = radio_id * channels_per_interface_;
  const size_t packet_length = Configuration()->DlPacketLength();
  const size_t cell_id = Configuration()->CellId().at(radio_id);

  std::vector<size_t> ant_ids(channels_per_interface_);
  std::vector<void*> samp(channels_per_interface_);
  std::vector<size_t> symbol_ids(channels_per_interface_, global_symbol_id);
  std::vector<Packet*> result_packets;

  for (size_t ch = 0; ch < channels_per_interface_; ++ch) {
    RxPacket& rx = rx_memory_.at(rx_slot + ch);

    // if rx_buffer is full, exit
    if (rx.Empty() == false) {
      MLPD_ERROR("TxRxWorkerArgos[%zu] rx_buffer full, rx slot: %zu\n", tid_,
                 rx_slot);
      Configuration()->Running(false);
      break;
    }
    ant_ids.at(ch) = ant_id + ch;
    samp.at(ch) = rx.RawPacket()->data_;
  }

  long long frame_time;
  bool cal_rx =
      (radio_id != Configuration()->RefRadio(cell_id) &&
       Configuration()->IsCalUlPilot(global_frame_id, global_symbol_id)) ||
      (radio_id == Configuration()->RefRadio(cell_id) &&
       Configuration()->IsCalDlPilot(global_frame_id, global_symbol_id));
  bool dummy_read =
      (Configuration()->HwFramer() == false) &&
      (!Configuration()->IsPilot(global_frame_id, global_symbol_id) &&
       !Configuration()->IsUplink(global_frame_id, global_symbol_id) &&
       !cal_rx);

  //std::vector<std::vector<std::complex<int16_t>>> dummmy_rx(
  //    Configuration()->NumChannels(),
  //    std::vector<std::complex<int16_t>>(Configuration()->SampsPerSymbol(), 0));

  //Ok to read into sample memory for dummy read
  int rx_status = radio_config_->RadioRx(radio_id, samp.data(), frame_time);

  if (dummy_read) {
    return result_packets;
  } else if (rx_status == 0) {
    return result_packets;
  } else if (rx_status < 0) {
    MLPD_ERROR("RX status = %d is less than 0\n", rx_status);
  } else if (static_cast<size_t>(rx_status) !=
             Configuration()->SampsPerSymbol()) {
    MLPD_ERROR("RX status = %d is not the expected value\n", rx_status);
    // WHAT TO DO HERE?????????????????
    return result_packets;
  }
  size_t frame_id = global_frame_id;
  size_t symbol_id = global_symbol_id;

  if (Configuration()->HwFramer() == true) {
    frame_id = (size_t)(frame_time >> 32);
    symbol_id = (size_t)((frame_time >> 16) & 0xFFFF);
    for (size_t ch = 0; ch < channels_per_interface_; ch++) {
      symbol_ids.at(ch) = symbol_id;
    }

    // Additional read for DL Calibration in two-chan case.
    // In HW framer mode we need to be specific about
    // which symbols to read a.o.t SW framer mode where
    // we read all symbols and discard unused ones later.
    if ((Configuration()->Frame().IsRecCalEnabled() == true) &&
        (Configuration()->IsCalDlPilot(frame_id, symbol_id) == true) &&
        (radio_id == Configuration()->RefRadio(cell_id)) &&
        (Configuration()->AntPerGroup() > 1)) {
      if (Configuration()->AntPerGroup() > channels_per_interface_) {
        symbol_ids.resize(Configuration()->AntPerGroup());
        ant_ids.resize(Configuration()->AntPerGroup());
      }
      for (size_t s = 1; s < Configuration()->AntPerGroup(); s++) {
        RxPacket& rx = rx_memory_.at(rx_slot + s);

        std::vector<void*> tmp_samp(channels_per_interface_);
        std::vector<char> dummy_buff(packet_length);
        tmp_samp.at(0) = rx.RawPacket()->data_;
        tmp_samp.at(1) = dummy_buff.data();
        if ((Configuration()->Running() == false) ||
            radio_config_->RadioRx(radio_id, tmp_samp.data(), frame_time) <=
                0) {
          std::vector<Packet*> empty_pkt;
          return empty_pkt;
        }
        symbol_ids.at(s) = Configuration()->Frame().GetDLCalSymbol(s);
        ant_ids.at(s) = ant_id;
      }
    }
  }

  for (size_t ch = 0; ch < symbol_ids.size(); ++ch) {
    RxPacket& rx = rx_memory_.at(rx_slot + ch);
    result_packets.push_back(rx.RawPacket());
    new (rx.RawPacket())
        Packet(frame_id, symbol_ids.at(ch), cell_id, ant_ids.at(ch));

    rx.Use();
    // Push kPacketRX event into the queue.
    EventData rx_message(EventType::kPacketRX, rx_tag_t(rx).tag_);

    if (event_notify_q_->enqueue(notify_producer_token_, rx_message) == false) {
      std::printf("TxRxWorkerArgos[%zu]: socket message enqueue failed\n",
                  tid_);
      throw std::runtime_error(
          "TxRxWorkerArgos: socket message enqueue failed");
    }
  }
  return result_packets;
}

void TxRxWorkerArgos::TxBeaconHw(size_t frame_id, size_t interface_id,
                                 long long time0) {
  RtAssert(Configuration()->HwFramer() == false,
           "HwFramer == true when TxBeaconHw was called");
  const auto beacon_symbol_id = Configuration()->Frame().GetBeaconSymbol(0);
  const size_t radio_id = interface_id + interface_offset_;
  const std::vector<std::complex<int16_t>> zeros(
      std::vector<std::complex<int16_t>>(Configuration()->SampsPerSymbol(),
                                         std::complex<int16_t>(0, 0)));

  //We can just point the tx to the same zeros location, no need to make more
  std::vector<const void*> tx_buffs(Configuration()->NumChannels(),
                                    zeros.data());

  const size_t beacon_radio =
      Configuration()->BeaconAnt() / Configuration()->NumChannels();
  const size_t beacon_ch =
      Configuration()->BeaconAnt() % Configuration()->NumChannels();

  if (beacon_radio == radio_id) {
    tx_buffs.at(beacon_ch) = Configuration()->BeaconCi16().data();
  }
  // assuming beacon is first symbol
  long long frame_time = time0 + (Configuration()->SampsPerSymbol() *
                                  (((frame_id + TX_FRAME_DELTA) *
                                    Configuration()->Frame().NumTotalSyms()) +
                                   beacon_symbol_id));

  int tx_ret = radio_config_->RadioTx(radio_id, tx_buffs.data(),
                                      GetTxFlags(radio_id, beacon_symbol_id),
                                      frame_time);
  if (tx_ret != static_cast<int>(Configuration()->SampsPerSymbol())) {
    std::cerr << "BAD BEACON Transmit(" << tx_ret << "/"
              << Configuration()->SampsPerSymbol() << ") at Time " << frame_time
              << ", frame count " << frame_id << std::endl;
  }
}

//Called when finished with the last antenna of the given radio
// C / L symbols must occur before the downlink D symbols.
// Could move this to the main agora tx scheduler which should simplify the tx logic
void TxRxWorkerArgos::TxReciprocityCalibPilots(size_t frame_id, size_t radio_id,
                                               long long time0) {
  const size_t cell_id = Configuration()->CellId().at(radio_id);
  const std::vector<std::complex<int16_t>> zeros(
      Configuration()->SampsPerSymbol(), std::complex<int16_t>(0, 0));

  //Schedule the Calibration Uplink 'L' Symbol on the reference radio
  if (radio_id == Configuration()->RefRadio(cell_id)) {
    const size_t tx_symbol_id = Configuration()->Frame().GetULCalSymbol(0);
    std::vector<const void*> calultxbuf(Configuration()->NumChannels(),
                                        zeros.data());

    const size_t ref_ant = Configuration()->RefAnt(cell_id);
    const size_t ant_idx = ref_ant % Configuration()->NumChannels();
    // We choose to use the reference antenna to tx from the reference radio
    calultxbuf.at(ant_idx) = Configuration()->PilotCi16().data();
    long long frame_time = 0;
    if (Configuration()->HwFramer() == false) {
      frame_time = time0 + (Configuration()->SampsPerSymbol() *
                            (((frame_id + TX_FRAME_DELTA) *
                              Configuration()->Frame().NumTotalSyms()) +
                             tx_symbol_id));
    } else {
      frame_time =
          ((long long)(frame_id + TX_FRAME_DELTA) << 32) | (tx_symbol_id << 16);
    }
    //Check to see if the next symbol is a Tx symbol for the reference node
    radio_config_->RadioTx(radio_id, calultxbuf.data(),
                           GetTxFlags(radio_id, tx_symbol_id), frame_time);
  } else {
    // ! ref_radio -- Transmit downlink calibration (array to ref) pilot
    // Send all CalDl symbols 'C'
    std::vector<const void*> caldltxbuf(Configuration()->NumChannels(),
                                        zeros.data());
    const size_t num_dl_cal = Configuration()->AntPerGroup();
    assert(Configuration()->AntPerGroup() ==
           Configuration()->Frame().NumDLCalSyms());
    const size_t calib_ant_complete = (frame_id * num_dl_cal);

    // bf_ant_num_ == total number bs antenna's minus any external reference nodes
    //AntPerGroup() = frame_.NumDLCalSyms();                                         2
    //AntGroupNum() = frame_.IsRecCalEnabled() ? (bf_ant_num_ / ant_per_group_) : 0; 12

    // For each C only 1 channel / antenna can send pilots.  All others send zeros for now, per schedule.
    for (size_t cal_tx_pilot = 0; cal_tx_pilot < num_dl_cal; cal_tx_pilot++) {
      //Check to see if the pilot antenna is on radio_id
      const size_t calib_antenna =
          (calib_ant_complete + cal_tx_pilot) % Configuration()->AntGroupNum();
      const size_t calib_radio = calib_antenna / channels_per_interface_;
      const size_t pilot_idx = calib_antenna % channels_per_interface_;
      const size_t tx_symbol_id =
          Configuration()->Frame().GetDLCalSymbol(cal_tx_pilot);

      if (calib_radio == radio_id) {
        caldltxbuf.at(pilot_idx) = Configuration()->PilotCi16().data();
      }

      long long frame_time = 0;
      if (Configuration()->HwFramer() == false) {
        frame_time = time0 + (Configuration()->SampsPerSymbol() *
                              ((frame_id + TX_FRAME_DELTA) *
                                   Configuration()->Frame().NumTotalSyms() +
                               tx_symbol_id));
      } else {
        frame_time = ((long long)(frame_id + TX_FRAME_DELTA) << 32) |
                     (tx_symbol_id << 16);
      }
      radio_config_->RadioTx(radio_id, caldltxbuf.data(),
                             GetTxFlags(radio_id, tx_symbol_id), frame_time);

      //Reset the caldltxbuf to zeros for next loop
      if (calib_radio == radio_id) {
        caldltxbuf.at(pilot_idx) = zeros.data();
      }
    }  // end s < Configuration()->RadioPerGroup()
  }    // ! ref radio
}

//Tx data
size_t TxRxWorkerArgos::DequeueSend(long long time0) {
  const size_t channels_per_interface = Configuration()->NumChannels();
  const size_t max_dequeue_items = num_interfaces_ * channels_per_interface;
  std::vector<EventData> events(max_dequeue_items);

  //Single producer ordering in q is preserved
  const size_t dequeued_items = tx_pending_q_->try_dequeue_bulk_from_producer(
      tx_producer_token_, events.data(), max_dequeue_items);

  for (size_t item = 0; item < dequeued_items; item++) {
    EventData& current_event = events.at(item);

    // std::printf("tx queue length: %d\n", task_queue_->size_approx());
    assert(current_event.event_type_ == EventType::kPacketTX);

    size_t frame_id = gen_tag_t(current_event.tags_[0u]).frame_id_;
    const size_t symbol_id = gen_tag_t(current_event.tags_[0u]).symbol_id_;
    const size_t ant_id = gen_tag_t(current_event.tags_[0u]).ant_id_;
    const size_t radio_id = ant_id / channels_per_interface;

    RtAssert((radio_id >= interface_offset_) &&
             (radio_id <= (interface_offset_ + num_interfaces_)));

    //See if this is the last antenna on the radio.  Assume that we receive the last one
    // last (and all the others came before).  No explicit tracking
    const bool last_antenna =
        ((ant_id % channels_per_interface) + 1) == (channels_per_interface);

    //std::printf("TxRxWorkerArgos[%zu]: tx antenna %zu radio %zu is last %d\n",
    //            tid_, ant_id, radio_id, last_antenna);

    const size_t dl_symbol_idx =
        Configuration()->Frame().GetDLSymbolIdx(symbol_id);
    const size_t offset =
        (Configuration()->GetTotalDataSymbolIdxDl(frame_id, dl_symbol_idx) *
         Configuration()->BsAntNum()) +
        ant_id;

    // All antenna data is ready to tx for a given symbol, if last then TX out the data
    if (last_antenna) {
      //When the first Tx symbol of the frame is ready, schedule beacon and cals
      if (symbol_id == Configuration()->Frame().GetDLSymbol(0)) {
        // Schedule beacon in the future
        if (Configuration()->HwFramer() == false) {
          this->TxBeaconHw(frame_id, radio_id, time0);
        }

        if (Configuration()->Frame().IsRecCalEnabled() == true) {
          this->TxReciprocityCalibPilots(frame_id, radio_id, time0);
        }
      }
      std::vector<const void*> txbuf(channels_per_interface);
      if (kDebugDownlink == true) {
        const std::vector<std::complex<int16_t>> zeros(
            Configuration()->SampsPerSymbol(), std::complex<int16_t>(0, 0));
        for (size_t ch = 0; ch < channels_per_interface; ch++) {
          // Not exactly sure why 0 index was selected here.  Could it be a beacon ant?
          if (ant_id != 0) {
            txbuf.at(ch) = zeros.data();
          } else if (dl_symbol_idx <
                     Configuration()->Frame().ClientDlPilotSymbols()) {
            txbuf.at(ch) =
                reinterpret_cast<void*>(Configuration()->UeSpecificPilotT()[0]);
          } else {
            txbuf.at(ch) = reinterpret_cast<void*>(
                Configuration()->DlIqT()[dl_symbol_idx]);
          }
        }
      } else {
        for (size_t ch = 0; ch < channels_per_interface; ch++) {
          auto* pkt = reinterpret_cast<Packet*>(
              &tx_memory_[((offset + ch) * Configuration()->DlPacketLength())]);
          txbuf.at(ch) = reinterpret_cast<void*>(pkt->data_);
        }
      }

      long long frame_time = 0;
      if (Configuration()->HwFramer() == false) {
        frame_time = time0 + (Configuration()->SampsPerSymbol() *
                              (((frame_id + TX_FRAME_DELTA) *
                                Configuration()->Frame().NumTotalSyms()) +
                               symbol_id));
      } else {
        frame_time =
            ((long long)(frame_id + TX_FRAME_DELTA) << 32) | (symbol_id << 16);
      }
      radio_config_->RadioTx(radio_id, txbuf.data(),
                             GetTxFlags(radio_id, symbol_id), frame_time);
    }

    if (kDebugPrintInTask == true) {
      std::printf(
          "TxRxWorkerArgos[%zu]: Transmitted frame %zu, symbol %zu, "
          "ant %zu, offset: %zu, msg_queue_length: %zu\n",
          tid_, frame_id, symbol_id, ant_id, offset,
          event_notify_q_->size_approx());
    }

    RtAssert(event_notify_q_->enqueue(
                 notify_producer_token_,
                 EventData(EventType::kPacketTX, current_event.tags_[0])),
             "Socket message enqueue failed\n");
  }
  return dequeued_items;
}

//Checks to see if the current symbol is followed by another tx symbol
//Need the radio id, to check against the reference radio
bool TxRxWorkerArgos::IsTxSymbolNext(size_t radio_id, size_t current_symbol) {
  bool tx_symbol_next = false;
  const auto cell_id = Configuration()->CellId().at(radio_id);
  const auto reference_radio = Configuration()->RefRadio(cell_id);

  if (current_symbol != Configuration()->Frame().NumTotalSyms()) {
    auto next_symbol = Configuration()->GetSymbolType(current_symbol + 1);
    if ((next_symbol == SymbolType::kDL) ||
        (next_symbol == SymbolType::kBeacon)) {
      tx_symbol_next = true;
    } else {
      if ((radio_id == reference_radio) &&
          (next_symbol == SymbolType::kCalUL)) {
        tx_symbol_next = true;
      } else if ((radio_id != reference_radio) &&
                 (next_symbol == SymbolType::kCalDL)) {
        tx_symbol_next = true;
      }
    }
  }
  return tx_symbol_next;
}

int TxRxWorkerArgos::GetTxFlags(size_t radio_id, size_t tx_symbol_id) {
  int tx_flags;
  //Flags == 1   // HAS_TIME
  //Flags == 2;  // HAS_TIME & END_BURST, name me
  const auto tx_again = IsTxSymbolNext(radio_id, tx_symbol_id);
  if (tx_again) {
    tx_flags = 1;
  } else {
    tx_flags = 2;
  }
  return tx_flags;
}