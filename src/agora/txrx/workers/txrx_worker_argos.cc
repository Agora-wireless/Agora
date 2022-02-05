/**
 * @file txrx_worker_argos.cc
 * @brief Implementation of PacketTxRx datapath functions for communicating
 * with real Argos hardware
 */

#include "txrx_worker_argos.h"

#include <cassert>

#include "logger.h"

static constexpr bool kSymbolTimingEnabled = false;

TxRxWorkerArgos::TxRxWorkerArgos(
    size_t core_offset, size_t tid, size_t interface_count,
    size_t interface_offset, Config* const config, size_t* rx_frame_start,
    moodycamel::ConcurrentQueue<EventData>* event_notify_q,
    moodycamel::ConcurrentQueue<EventData>* tx_pending_q,
    moodycamel::ProducerToken& tx_producer,
    moodycamel::ProducerToken& notify_producer,
    std::vector<RxPacket>& rx_memory, std::byte* const tx_memory,
    std::mutex& sync_mutex, std::condition_variable& sync_cond,
    std::atomic<bool>& can_proceed, RadioConfig& radio_config)
    : TxRxWorker(core_offset, tid, interface_count, interface_offset, config,
                 rx_frame_start, event_notify_q, tx_pending_q, tx_producer,
                 notify_producer, rx_memory, tx_memory, sync_mutex, sync_cond,
                 can_proceed),
      radio_config_(radio_config),
      program_start_ticks_(0),
      freq_ghz_(GetTime::MeasureRdtscFreq()) {}

TxRxWorkerArgos::~TxRxWorkerArgos() = default;

//Main Thread Execution loop
void TxRxWorkerArgos::DoTxRx() {
  PinToCoreWithOffset(ThreadType::kWorkerTXRX, core_offset_, tid_);

  MLPD_INFO("TxRxWorkerArgos[%zu] has %zu:%zu total radios %zu\n", tid_,
            interface_offset_, (interface_offset_ + num_interfaces_) - 1,
            num_interfaces_);

  running_ = true;
  WaitSync();

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
    constexpr int kBeacontxFlags = 2;  // END BURST
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
              radio_config_.RadioRx(rx_radio_id, samp_buffer, rx_time_bs);
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
          rx_status = radio_config_.RadioRx(radio_id, samp_buffer, rx_time_bs);
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
          tx_status = radio_config_.RadioTx(radio_id, beaconbuffs.data(),
                                            kBeacontxFlags, tx_time_bs);
        } else {
          tx_status = radio_config_.RadioTx(radio_id, zeros, kBeacontxFlags,
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
          rx_status = radio_config_.RadioRx(radio_id, samp_buffer, rx_time_bs);
          //---------------Check status?
        }
      }
    }
    std::cout << "Start BS main recv loop..." << std::endl;
  }  // HwFramer == false

  ssize_t prev_frame_id = -1;

  TxRxWorkerRx::RxParameters receive_attempt;
  receive_attempt = UpdateRxInterface(receive_attempt);
  MLPD_INFO(
      "TxRxWorkerArgos[%zu]: Starting rx interface id %zu looking for symbol "
      "%zu\n",
      tid_, receive_attempt.interface_, receive_attempt.symbol_);

  size_t global_frame_id = 0;
  size_t global_symbol_id = 0;
  std::vector<TxRxWorkerRx::RxTimeTracker> rx_times(num_interfaces_);
  program_start_ticks_ = GetTime::Rdtsc();
  while (Configuration()->Running() == true) {
    if (0 == DequeueSend(time0)) {
      // attempt to receive symbol (might be good to reduce the rx timeout to allow for tx during dead time)
      if (kSymbolTimingEnabled) {
        rx_times.at(receive_attempt.interface_).start_ticks_ = GetTime::Rdtsc();
      }
      auto pkts = RecvEnqueue(receive_attempt.interface_, global_frame_id,
                              global_symbol_id);

      const size_t rx_time_ticks = GetTime::Rdtsc();
      if (kSymbolTimingEnabled) {
        rx_times.at(receive_attempt.interface_).end_ticks_ = rx_time_ticks;
      }

      if (!pkts.empty()) {
        RtAssert(pkts.size() == channels_per_interface_,
                 "Received data but it was the wrong dimension");
        size_t rx_symbol_id = pkts.front()->symbol_id_;

        if (unlikely(rx_symbol_id != receive_attempt.symbol_)) {
          MLPD_ERROR(
              "TxRxWorkerArgos[%zu]: Frame %d - Expected symbol %zu but "
              "received %zu\n",
              tid_, pkts.front()->frame_id_, receive_attempt.symbol_,
              rx_symbol_id);
        }

        RtAssert(
            rx_symbol_id == receive_attempt.symbol_,
            "The expected receive symbol does not match the actual symbol");

        // Symbol received, change the rx interface
        TxRxWorkerRx::RxParameters successful_receive = receive_attempt;
        receive_attempt = UpdateRxInterface(successful_receive);
        MLPD_TRACE(
            "TxRxWorkerArgos[%zu]: Last Interface %zu - symbol %zu:%zu, next "
            "interface %zu - symbol %zu\n",
            tid_, successful_receive.interface_, successful_receive.symbol_,
            rx_symbol_id, receive_attempt.interface_, receive_attempt.symbol_);

        if (kIsWorkerTimingEnabled) {
          const auto frame_id = pkts.front()->frame_id_;
          if (frame_id > prev_frame_id) {
            rx_frame_start_[frame_id % kNumStatsFrames] = rx_time_ticks;
            prev_frame_id = frame_id;
          }
        }

        PrintRxSymbolTiming(rx_times, pkts.front()->frame_id_,
                            successful_receive.symbol_,
                            receive_attempt.symbol_);
      }  // if (pkt.size() > 0)
    }    // DequeueSendArgos(time0) == 0
  }      // Configuration()->Running() == true
  running_ = false;
}

//RX data, should return channel number of packets || 0
std::vector<Packet*> TxRxWorkerArgos::RecvEnqueue(size_t interface_id,
                                                  size_t global_frame_id,
                                                  size_t global_symbol_id) {
  const size_t radio_id = interface_id + interface_offset_;
  const size_t ant_id = radio_id * channels_per_interface_;
  const size_t cell_id = Configuration()->CellId().at(radio_id);

  std::vector<size_t> ant_ids(channels_per_interface_);
  std::vector<void*> samp(channels_per_interface_);
  std::vector<RxPacket*> memory_tracking;

  //Return value
  std::vector<Packet*> result_packets;

  //Allocate memory
  for (size_t ch = 0; ch < channels_per_interface_; ch++) {
    RxPacket& rx = GetRxPacket();
    memory_tracking.push_back(&rx);
    ant_ids.at(ch) = ant_id + ch;
    samp.at(ch) = rx.RawPacket()->data_;
    MLPD_TRACE("TxRxWorkerArgos[%zu]: Using Packet at location %zu\n", tid_,
               reinterpret_cast<size_t>(&rx));
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

  //Ok to read into sample memory for dummy read
  const int rx_status =
      radio_config_.RadioRx(radio_id, samp.data(), frame_time);

  if ((dummy_read == false) && (rx_status > 0)) {
    //     (rx_status == static_cast<int>(Configuration()->SampsPerSymbol()))) {
    size_t frame_id = global_frame_id;
    size_t symbol_id = global_symbol_id;

    if (Configuration()->HwFramer() == true) {
      frame_id = static_cast<size_t>(frame_time >> 32);
      symbol_id = static_cast<size_t>((frame_time >> 16) & 0xFFFF);
    }

    if (rx_status != static_cast<int>(Configuration()->SampsPerSymbol())) {
      MLPD_WARN(
          "TxRxWorkerArgos[%zu]: Interface %zu | Radio %zu  - Attempted "
          "Frame: %zu, Symbol: %zu, RX status = %d is not the expected value\n",
          tid_, interface_id, interface_id + interface_offset_, frame_id,
          symbol_id, rx_status);
    } else {
      MLPD_FRAME(
          "TxRxWorkerArgos[%zu]: Interface %zu | Radio %zu  - Attempted "
          "Frame: %zu, Symbol: %zu, RX status = %d\n",
          tid_, interface_id, interface_id + interface_offset_, frame_id,
          symbol_id, rx_status);
    }

    for (size_t ant = 0; ant < ant_ids.size(); ant++) {
      auto* rx_packet = memory_tracking.at(ant);
      auto* raw_pkt = rx_packet->RawPacket();
      new (raw_pkt) Packet(frame_id, symbol_id, cell_id, ant_ids.at(ant));
      result_packets.push_back(raw_pkt);

      // Push kPacketRX event into the queue.
      EventData rx_message(EventType::kPacketRX, rx_tag_t(*rx_packet).tag_);
      NotifyComplete(rx_message);
      memory_tracking.at(ant) = nullptr;
    }
  } else if (rx_status < 0) {
    MLPD_ERROR(
        "TxRxWorkerArgos[%zu], Interface %zu | Radio %zu - Rx failure RX "
        "status = %d is less than 0\n",
        tid_, interface_id, interface_id + interface_offset_, rx_status);
  } else if ((rx_status != 0) && (static_cast<size_t>(rx_status) !=
                                  Configuration()->SampsPerSymbol())) {
    const size_t rx_frame = static_cast<size_t>(frame_time >> 32);
    const size_t rx_symbol = static_cast<size_t>((frame_time >> 16) & 0xFFFF);
    MLPD_ERROR(
        "TxRxWorkerArgos[%zu]: Interface %zu | Radio %zu  - Attempted Frame: "
        "%zu, Symbol: %zu, RX status = %d is not the expected value\n",
        tid_, interface_id, interface_id + interface_offset_, rx_frame,
        rx_symbol, rx_status);
  }

  //Free memory from most recent allocated to latest
  MLPD_TRACE("TxRxWorkerArgos[%zu]: Memory allocation %zu\n", tid_,
             memory_tracking.size());
  for (ssize_t idx = (memory_tracking.size() - 1); idx > -1; idx--) {
    auto* memory_location = memory_tracking.at(idx);
    MLPD_TRACE("TxRxWorkerArgos[%zu]: Checking location %zu\n", tid_,
               (size_t)memory_location);
    if (memory_location != nullptr) {
      MLPD_TRACE("TxRxWorkerArgos[%zu]: Returning Packet at location %zu\n",
                 tid_, (size_t)memory_location);
      ReturnRxPacket(*memory_location);
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

  int tx_ret =
      radio_config_.RadioTx(radio_id, tx_buffs.data(),
                            GetTxFlags(radio_id, beacon_symbol_id), frame_time);
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

  MLPD_FRAME(
      "TxRxWorkerArgos[%zu]: TxReciprocityCalibPilots (Frame %zu,         , "
      "Radio %zu\n",
      tid_, frame_id, radio_id);

  //Schedule the Calibration Uplink 'L' Symbol on the reference radio
  // assumes there is only 1 ULCalSymbol
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

    MLPD_TRACE(
        "TxRxWorkerArgos[%zu]: TxReciprocityCalibPilots (Frame %zu, Symbol "
        "%zu, Radio %zu) is reference tx on channel %zu\n",
        tid_, frame_id, tx_symbol_id, radio_id, ant_idx);
    // Check to see if the next symbol is a Tx symbol for the reference node
    radio_config_.RadioTx(radio_id, calultxbuf.data(),
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
      radio_config_.RadioTx(radio_id, caldltxbuf.data(),
                            GetTxFlags(radio_id, tx_symbol_id), frame_time);

      MLPD_TRACE(
          "TxRxWorkerArgos[%zu]: TxReciprocityCalibPilots (Frame %zu, Symbol "
          "%zu, Radio %zu) dl pilot tx has data %d on channel %zu\n",
          tid_, frame_id, tx_symbol_id, radio_id, calib_radio == radio_id,
          pilot_idx);

      //Reset the caldltxbuf to zeros for next loop
      if (calib_radio == radio_id) {
        caldltxbuf.at(pilot_idx) = zeros.data();
      }
    }  // end s < Configuration()->RadioPerGroup()
  }    // ! ref radio
}

//Tx data
size_t TxRxWorkerArgos::DequeueSend(long long time0) {
  auto tx_events = GetPendingTxEvents();

  //Process each pending tx event
  for (const EventData& current_event : tx_events) {
    // std::printf("tx queue length: %d\n", task_queue_->size_approx());
    assert(current_event.event_type_ == EventType::kPacketTX);

    size_t frame_id = gen_tag_t(current_event.tags_[0u]).frame_id_;
    const size_t symbol_id = gen_tag_t(current_event.tags_[0u]).symbol_id_;
    const size_t ant_id = gen_tag_t(current_event.tags_[0u]).ant_id_;
    const size_t radio_id = ant_id / channels_per_interface_;

    RtAssert((radio_id >= interface_offset_) &&
             (radio_id <= (interface_offset_ + num_interfaces_)));

    //See if this is the last antenna on the radio.  Assume that we receive the last one
    // last (and all the others came before).  No explicit tracking
    const bool last_antenna =
        ((ant_id % channels_per_interface_) + 1) == (channels_per_interface_);

    const size_t dl_symbol_idx =
        Configuration()->Frame().GetDLSymbolIdx(symbol_id);

    // All antenna data is ready to tx for a given symbol, if last then TX out the data
    if (last_antenna) {
      MLPD_TRACE(
          "TxRxWorkerArgos[%zu]: (Frame %zu, Symbol %zu) last tx antenna %zu "
          "for radio %zu has tx data for all antennas / channels\n",
          tid_, frame_id, symbol_id, ant_id, radio_id);

      // When the first Tx symbol of the frame is ready (for a specific radio), schedule beacon and cals
      // assumes the symbols are in the correct order (ie get symbol 0 before symbol 1)
      if (symbol_id == Configuration()->Frame().GetDLSymbol(0)) {
        // Schedule beacon in the future
        if (Configuration()->HwFramer() == false) {
          this->TxBeaconHw(frame_id, radio_id, time0);
        }

        if (Configuration()->Frame().IsRecCalEnabled() == true) {
          this->TxReciprocityCalibPilots(frame_id, radio_id, time0);
        }
      }
      std::vector<const void*> txbuf(channels_per_interface_);
      if (kDebugDownlink == true) {
        const std::vector<std::complex<int16_t>> zeros(
            Configuration()->SampsPerSymbol(), std::complex<int16_t>(0, 0));
        for (size_t ch = 0; ch < channels_per_interface_; ch++) {
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
        for (size_t ch = 0; ch < channels_per_interface_; ch++) {
          auto* pkt = GetTxPacket(frame_id, symbol_id, ant_id + ch);
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
      radio_config_.RadioTx(radio_id, txbuf.data(),
                            GetTxFlags(radio_id, symbol_id), frame_time);
    }

    if (kDebugPrintInTask == true) {
      std::printf(
          "TxRxWorkerArgos[%zu]: Transmitted frame %zu, symbol %zu, ant %zu\n",
          tid_, frame_id, symbol_id, ant_id);
    }
    auto complete_event =
        EventData(EventType::kPacketTX, current_event.tags_[0]);
    NotifyComplete(complete_event);
  }
  return tx_events.size();
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

/// Returns the next symbol and interface
TxRxWorkerRx::RxParameters TxRxWorkerArgos::UpdateRxInterface(
    const TxRxWorkerRx::RxParameters& last_rx) {
  TxRxWorkerRx::RxParameters next_rx;

  const size_t total_symbols = Configuration()->Frame().NumTotalSyms();
  size_t search_symbol = last_rx.symbol_;
  size_t start_interface = last_rx.interface_ + 1;
  bool interface_found = false;
  //This search could probably be optimized (by creating a map in init)
  while (interface_found == false) {
    // For each symbol interate through all interfaces
    for (size_t interface = start_interface; interface < num_interfaces_;
         interface++) {
      bool is_rx = IsRxSymbol(interface, search_symbol);
      if (is_rx) {
        interface_found = true;
        next_rx.interface_ = interface;
        next_rx.symbol_ = search_symbol;
        break;
      }
    }
    search_symbol = (search_symbol + 1) % total_symbols;
    //Start at the first interface for each new symbol
    start_interface = 0;
    ///\todo Need to add an infinate loop catcher
  }
  return next_rx;
}

/*
  if (Configuration()->HwFramer() == false) {
    // Update global frame_id and symbol_id
    global_symbol_id++;
    if (global_symbol_id == Configuration()->Frame().NumTotalSyms()) {
      global_symbol_id = 0;
      global_frame_id++;
      if (Configuration()->Frame().NumDLSyms() == 0) {
        for (size_t interface = 0; interface < num_interfaces_; interface++) {
          this->TxBeaconHw(global_frame_id, interface, time0);
        }
      }
    }
  }  // HwFramer == false
*/

bool TxRxWorkerArgos::IsRxSymbol(size_t interface, size_t symbol_id) {
  auto symbol_type = Configuration()->GetSymbolType(symbol_id);
  const auto cell_id =
      Configuration()->CellId().at(interface + interface_offset_);
  const auto reference_radio = Configuration()->RefRadio(cell_id);
  const size_t radio_id = interface + interface_offset_;
  bool is_rx;

  if ((symbol_type == SymbolType::kPilot) || (symbol_type == SymbolType::kUL)) {
    is_rx = true;
  } else if ((reference_radio == radio_id) &&
             (symbol_type == SymbolType::kCalDL)) {
    is_rx = true;
  } else if ((reference_radio != radio_id) &&
             (symbol_type == SymbolType::kCalUL)) {
    is_rx = true;
  } else {
    is_rx = false;
  }
  return is_rx;
}

void TxRxWorkerArgos::PrintRxSymbolTiming(
    std::vector<TxRxWorkerRx::RxTimeTracker>& rx_times, size_t current_frame,
    size_t current_symbol, size_t next_symbol) {
  if (kSymbolTimingEnabled) {
    //Check to see if symbol receiption is complete
    if (current_symbol != next_symbol) {
      std::vector<double> rx_us(num_interfaces_, 0.0f);
      double total_symbol_rx_time = 0.0f;
      size_t symbol_start_ticks = 0;
      for (size_t i = 0; i < num_interfaces_; i++) {
        if (symbol_start_ticks == 0 && rx_times.at(i).start_ticks_ > 0) {
          symbol_start_ticks = rx_times.at(i).start_ticks_;
        }
        rx_us.at(i) = GetTime::CyclesToUs(
            (rx_times.at(i).end_ticks_ - rx_times.at(i).start_ticks_),
            freq_ghz_);
        total_symbol_rx_time += rx_us.at(i);
        MLPD_TRACE(
            "TxRxWorkerArgos[%zu]: Radio %zu Frame %d Symbol %zu Rx "
            "Start uS %f for %f uS\n",
            tid_, i + interface_offset_, pkts.front()->frame_id_,
            successful_receive.symbol_,
            GetTime::CyclesToUs(rx_times.at(i).start_ticks_, freq_ghz_),
            rx_us.at(i));

        rx_times.at(i).start_ticks_ = 0;
        rx_times.at(i).end_ticks_ = 0;
      }

      const size_t ticks_now = GetTime::Rdtsc();
      const double avg_rx_time = total_symbol_rx_time / num_interfaces_;
      std::ostringstream result_message;
      //Add any radio that is > 2x the average
      result_message << std::fixed << std::setprecision(2) << "TxRxWorkerArgos["
                     << tid_ << "]: Frame " << current_frame << " Symbol "
                     << current_symbol << " started rx @ "
                     << GetTime::CyclesToUs(
                            symbol_start_ticks - program_start_ticks_,
                            freq_ghz_)
                     << " radio rx time (total:total:average) "
                     << total_symbol_rx_time << ":"
                     << GetTime::CyclesToUs(ticks_now - symbol_start_ticks,
                                            freq_ghz_)
                     << ":" << avg_rx_time;

      for (size_t i = 0; i < num_interfaces_; i++) {
        if (rx_us.at(i) > (avg_rx_time * 2)) {
          result_message << " Radio " << (i + interface_offset_)
                         << " rx time us " << rx_us.at(i);
        }
      }
      MLPD_INFO("%s\n", result_message.str().c_str());
    }

    //Frame RX complete -- print summary
    if (current_symbol > next_symbol) {
      MLPD_INFO("TxRxWorkerArgos[%zu]: Frame %zu rx complete @ %.2f\n", tid_,
                current_frame,
                GetTime::CyclesToUs(GetTime::Rdtsc() - program_start_ticks_,
                                    freq_ghz_));
    }
  }  //  if (kSymbolTimingEnabled)
}