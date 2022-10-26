/**
 * @file txrx_worker_hw.cc
 * @brief Implementation of PacketTxRx datapath functions for communicating
 * with real iris / faros hardware
 */

#include "txrx_worker_hw.h"

#include <cassert>

#include "comms-lib.h"
#include "gettime.h"
#include "logger.h"
#include "message.h"

static constexpr bool kSymbolTimingEnabled = false;
static constexpr bool kBeamsweepData = false;

TxRxWorkerHw::TxRxWorkerHw(
    size_t core_offset, size_t tid, size_t interface_count,
    size_t interface_offset, Config* const config, size_t* rx_frame_start,
    moodycamel::ConcurrentQueue<EventData>* event_notify_q,
    moodycamel::ConcurrentQueue<EventData>* tx_pending_q,
    moodycamel::ProducerToken& tx_producer,
    moodycamel::ProducerToken& notify_producer,
    std::vector<RxPacket>& rx_memory, std::byte* const tx_memory,
    std::mutex& sync_mutex, std::condition_variable& sync_cond,
    std::atomic<bool>& can_proceed, RadioConfig& radio_config)
    : TxRxWorker(core_offset, tid, interface_count, interface_offset,
                 config->NumChannels(), config, rx_frame_start, event_notify_q,
                 tx_pending_q, tx_producer, notify_producer, rx_memory,
                 tx_memory, sync_mutex, sync_cond, can_proceed),
      radio_config_(radio_config),
      program_start_ticks_(0),
      freq_ghz_(GetTime::MeasureRdtscFreq()),
      zeros_(config->SampsPerSymbol(), std::complex<int16_t>(0u, 0u)),
      first_symbol_(interface_count, true) {
  InitRxStatus();
}

TxRxWorkerHw::~TxRxWorkerHw() = default;

//Main Thread Execution loop
void TxRxWorkerHw::DoTxRx() {
  PinToCoreWithOffset(ThreadType::kWorkerTXRX, core_offset_, tid_);

  AGORA_LOG_INFO("TxRxWorkerHw[%zu] has %zu:%zu total radios %zu\n", tid_,
                 interface_offset_, (interface_offset_ + num_interfaces_) - 1,
                 num_interfaces_);

  running_ = true;
  WaitSync();

  if (num_interfaces_ == 0) {
    AGORA_LOG_WARN("TxRxWorkerHw[%zu] has no interfaces, exiting\n", tid_);
    running_ = false;
    return;
  }

  program_start_ticks_ = GetTime::Rdtsc();

  long long time0 = 0ul;
  time0 = GetHwTime();
  ssize_t prev_frame_id = -1;

  TxRxWorkerRx::RxParameters receive_attempt;
  receive_attempt = UpdateRxInterface(receive_attempt);
  AGORA_LOG_INFO(
      "TxRxWorkerHw[%zu]: Starting rx interface id %zu looking for symbol "
      "%zu\n",
      tid_, receive_attempt.interface_, receive_attempt.symbol_);

  size_t global_frame_id = 0;
  size_t global_symbol_id = 0;
  std::vector<TxRxWorkerRx::RxTimeTracker> rx_times(num_interfaces_);

  // Schedule TX_FRAME_DELTA transmit frames ( B + C + L + D )
  ScheduleTxInit(TX_FRAME_DELTA, time0);
  AGORA_LOG_FRAME("TxRxWorkerHw[%zu]: Tx init\n", tid_);

  // Agora will generate Tx data only after the first Rx............. (Typically the pilots from the UEs)
  //  The first Rx will happen based on a Hw trigger
  while (Configuration()->Running() == true) {
    const size_t tx_items = DoTx(time0);
    // If no items transmitted, then try to receive
    if (0 == tx_items) {
      if (kSymbolTimingEnabled) {
        rx_times.at(receive_attempt.interface_).start_ticks_ = GetTime::Rdtsc();
      }
      auto pkts =
          DoRx(receive_attempt.interface_, global_frame_id, global_symbol_id);

      const size_t rx_time_ticks = GetTime::Rdtsc();
      if (kSymbolTimingEnabled) {
        rx_times.at(receive_attempt.interface_).end_ticks_ = rx_time_ticks;
      }

      if (pkts.empty() == false) {
        bool ignore = false;
        RtAssert(pkts.size() == channels_per_interface_,
                 "Received data but it was the wrong dimension");
        const size_t rx_symbol_id = pkts.front()->RawPacket()->symbol_id_;
        const size_t rx_frame_id = pkts.front()->RawPacket()->frame_id_;

        bool is_first = first_symbol_.at(receive_attempt.interface_);

        //This first symbol tracking is only necessary for bad data (at the beginning) for hw_framer mode
        if (is_first) {
          //We will ignore the data until we get an acceptable first symbol / frame
          if ((rx_symbol_id == receive_attempt.symbol_) && (rx_frame_id == 0)) {
            //Match, don't allow any more incorrect symbol data
            first_symbol_.at(receive_attempt.interface_) = false;
            AGORA_LOG_TRACE(
                "TxRxWorkerHw[%zu]: Interface %zu Matched First Symbol "
                "%zu:%zu\n",
                tid_, receive_attempt.interface_, rx_symbol_id,
                receive_attempt.symbol_);
          } else {
            ignore = true;
            AGORA_LOG_WARN(
                "TxRxWorkerHw[%zu]: Ignoring Rx Data on Interface %zu (Frame "
                "%zu, Symbol %zu) - Expected symbol %zu\n",
                tid_, receive_attempt.interface_, rx_frame_id, rx_symbol_id,
                receive_attempt.symbol_);
          }
        } else if (rx_symbol_id != receive_attempt.symbol_) {
          //Hard error, getting unexpected data
          AGORA_LOG_ERROR(
              "TxRxWorkerHw[%zu]: Frame %zu - Expected symbol %zu but "
              "received %zu on interface %zu\n",
              tid_, rx_frame_id, receive_attempt.symbol_, rx_symbol_id,
              receive_attempt.interface_);
          RtAssert(
              rx_symbol_id == receive_attempt.symbol_,
              "The expected receive symbol does not match the actual symbol");
        }

        if (ignore == false) {
          //Publish the symbols to the scheduler
          for (auto* packet : pkts) {
            const EventData rx_message(EventType::kPacketRX,
                                       rx_tag_t(packet).tag_);
            NotifyComplete(rx_message);
          }

          // Symbol received, change the rx interface
          TxRxWorkerRx::RxParameters successful_receive = receive_attempt;
          receive_attempt = UpdateRxInterface(successful_receive);
          AGORA_LOG_TRACE(
              "TxRxWorkerHw[%zu]: Last Interface %zu - symbol %zu:%zu, next "
              "interface %zu - symbol %zu\n",
              tid_, successful_receive.interface_, successful_receive.symbol_,
              rx_symbol_id, receive_attempt.interface_,
              receive_attempt.symbol_);

          if (kIsWorkerTimingEnabled) {
            if (static_cast<ssize_t>(rx_frame_id) > prev_frame_id) {
              rx_frame_start_[rx_frame_id % kNumStatsFrames] = rx_time_ticks;
              prev_frame_id = rx_frame_id;
            }
          }

          PrintRxSymbolTiming(rx_times, rx_frame_id, successful_receive.symbol_,
                              receive_attempt.symbol_);
        }  //!ignore
        else {
          //Return the Packets (must be in reverse order)
          for (size_t i = pkts.size(); i > 0; i--) {
            ReturnRxPacket(*pkts.at(i - 1));
          }
        }
      }  // if (pkt.size() > 0)
    }    // DoTx(time0) == 0
  }      // Configuration()->Running() == true
  running_ = false;
}

//RX data, should return channel number of packets || 0
// global_frame_id will be used and updated
// global_symbol_id will be used and updated
std::vector<RxPacket*> TxRxWorkerHw::DoRx(size_t interface_id,
                                          size_t& global_frame_id,
                                          size_t& global_symbol_id) {
  const size_t radio_id = interface_id + interface_offset_;

  //Return value
  std::vector<RxPacket*> result_packets;

  auto& rx_info = rx_status_.at(interface_id);
  const size_t request_samples =
      Configuration()->SampsPerSymbol() - rx_info.SamplesAvailable();
  RtAssert(Configuration()->SampsPerSymbol() > rx_info.SamplesAvailable(),
           "Rx Samples must be > 0");

  auto rx_locations = rx_info.GetRxPtrs();
  long long frame_time;

  Radio::RxFlags out_flags;
  //Ok to read into sample memory for dummy read
  const int rx_status = radio_config_.RadioRx(
      radio_id, rx_locations, request_samples, out_flags, frame_time);

  if (rx_status > 0) {
    const size_t new_samples = static_cast<size_t>(rx_status);
    rx_info.Update(new_samples, frame_time);

    //Check for successful finish
    if ((new_samples == request_samples) ||
        (out_flags == Radio::RxFlags::kEndReceive)) {
      frame_time = rx_info.StartTime();
      const size_t ant_id = radio_id * channels_per_interface_;
      const size_t cell_id = Configuration()->CellId().at(radio_id);

      //Finished successfully
      bool invalid_rx_symbol = false;
      if (Configuration()->HwFramer() == true) {
        global_frame_id = static_cast<size_t>(frame_time >> 32);
        const size_t rx_symbol_id =
            static_cast<size_t>((frame_time >> 16) & 0xFFFF);

        if (rx_symbol_id > Configuration()->Frame().NumTotalSyms()) {
          invalid_rx_symbol = true;
        } else {
          global_symbol_id = rx_symbol_id;
        }
      }

      const size_t frame_id = global_frame_id;
      const size_t symbol_id = global_symbol_id;

      if (new_samples == request_samples) {
        RtAssert(
            rx_info.SamplesAvailable() == Configuration()->SampsPerSymbol(),
            "Samples Available should match SampsPerSymbol");

        AGORA_LOG_FRAME(
            "TxRxWorkerHw[%zu]: Interface %zu | Radio %zu  - Attempted "
            "Frame: %zu, Symbol: %zu, RX status = %d\n",
            tid_, interface_id, interface_id + interface_offset_, frame_id,
            symbol_id, rx_status);
      } else {
        AGORA_LOG_WARN(
            "TxRxWorkerHw[%zu]: Interface %zu | Radio %zu  - Attempted "
            "Frame: %zu, Symbol: %zu, RX samples %zu is less than the desired "
            "amount %zu\n",
            tid_, interface_id, interface_id + interface_offset_, frame_id,
            symbol_id, rx_info.SamplesAvailable(),
            Configuration()->SampsPerSymbol());
      }

      const bool cal_rx =
          (radio_id != Configuration()->RefRadio(cell_id) &&
           Configuration()->IsCalUlPilot(global_frame_id, global_symbol_id)) ||
          (radio_id == Configuration()->RefRadio(cell_id) &&
           Configuration()->IsCalDlPilot(global_frame_id, global_symbol_id));
      const bool ignore_rx_data =
          (invalid_rx_symbol == true) ||
          ((Configuration()->HwFramer() == false) &&
           (!Configuration()->IsPilot(global_frame_id, global_symbol_id) &&
            !Configuration()->IsUplink(global_frame_id, global_symbol_id) &&
            !cal_rx));

      // Update global frame_id and symbol_id
      global_symbol_id++;
      if (global_symbol_id == Configuration()->Frame().NumTotalSyms()) {
        global_symbol_id = 0;
        global_frame_id++;
      }

      if (ignore_rx_data == false) {
        auto packets = rx_info.GetRxPackets();
        for (size_t ch = 0; ch < channels_per_interface_; ch++) {
          auto* rx_packet = packets.at(ch);
          result_packets.emplace_back(rx_packet);
          auto* raw_pkt = rx_packet->RawPacket();
          new (raw_pkt) Packet(frame_id, symbol_id, cell_id, ant_id + ch);
          AGORA_LOG_TRACE(
              "TxRxWorkerHw[%zu]: Frame %zu Symbol %zu Ant %zu - Radio %zu - "
              "Received Symbol\n",
              tid_, frame_id, symbol_id, ant_id + ch, radio_id);
        }
      }
      ResetRxStatus(interface_id, ignore_rx_data);
    } else if (new_samples > request_samples) {
      RtAssert(
          false,
          "Received more samples than requested - possible memory overrun\n");
    }
    //new_samples < request_samples (do nothing)
  } else if (rx_status < 0) {
    AGORA_LOG_ERROR(
        "TxRxWorkerHw[%zu]: Interface %zu | Radio %zu - Rx failure RX "
        "status = %d is less than 0\n",
        tid_, interface_id, interface_id + interface_offset_, rx_status);
  }
  return result_packets;
}

void TxRxWorkerHw::TxBeaconHw(size_t frame_id, size_t interface_id,
                              long long time0) {
  RtAssert(Configuration()->HwFramer() == false,
           "HwFramer == true when TxBeaconHw was called");
  const auto beacon_symbol_id = Configuration()->Frame().GetBeaconSymbol(0);
  const size_t radio_id = interface_id + interface_offset_;

  //We can just point the tx to the same zeros location, no need to make more
  std::vector<const void*> tx_buffs(Configuration()->NumChannels(),
                                    zeros_.data());

  const size_t beacon_radio =
      Configuration()->BeaconAnt() / Configuration()->NumChannels();
  const size_t beacon_ch =
      Configuration()->BeaconAnt() % Configuration()->NumChannels();

  if (beacon_radio == radio_id) {
    tx_buffs.at(beacon_ch) = Configuration()->BeaconCi16().data();
  }
  // assuming beacon is first symbol
  long long frame_time =
      time0 + (Configuration()->SampsPerSymbol() *
               ((frame_id * Configuration()->Frame().NumTotalSyms()) +
                beacon_symbol_id));

  const int tx_ret =
      radio_config_.RadioTx(radio_id, tx_buffs.data(),
                            GetTxFlags(radio_id, beacon_symbol_id), frame_time);

  if (tx_ret != static_cast<int>(Configuration()->SampsPerSymbol())) {
    std::cerr << "BAD Transmit on radio " << radio_id << " - status " << tx_ret
              << ",  expected " << Configuration()->SampsPerSymbol()
              << " at Time " << frame_time << std::endl;
  }
}

//Called when finished with the last antenna of the given radio
// C / L symbols must occur before the downlink D symbols.
// Could move this to the main agora tx scheduler which should simplify the tx logic
void TxRxWorkerHw::TxReciprocityCalibPilots(size_t frame_id, size_t radio_id,
                                            long long time0) {
  const size_t cell_id = Configuration()->CellId().at(radio_id);

  AGORA_LOG_FRAME(
      "TxRxWorkerHw[%zu]: TxReciprocityCalibPilots (Frame %zu,         , Radio "
      "%zu\n",
      tid_, frame_id, radio_id);

  //Schedule the Calibration Uplink 'L' Symbol(s) on the reference radio
  if (radio_id == Configuration()->RefRadio(cell_id)) {
    for (size_t ul_cal_sym_idx = 0;
         ul_cal_sym_idx < Configuration()->Frame().NumULCalSyms();
         ul_cal_sym_idx++) {
      const size_t tx_symbol_id =
          Configuration()->Frame().GetULCalSymbol(ul_cal_sym_idx);
      std::vector<const void*> calultxbuf(Configuration()->NumChannels(),
                                          zeros_.data());

      const size_t ref_ant = Configuration()->RefAnt(cell_id);
      const size_t ant_idx = ref_ant % Configuration()->NumChannels();
      // We choose to use the reference antenna to tx from the reference radio
      calultxbuf.at(ant_idx) = Configuration()->PilotCi16().data();
      long long frame_time = 0;
      if (Configuration()->HwFramer() == false) {
        frame_time =
            time0 + (Configuration()->SampsPerSymbol() *
                     ((frame_id * Configuration()->Frame().NumTotalSyms()) +
                      tx_symbol_id));
      } else {
        frame_time = ((long long)(frame_id) << 32) | (tx_symbol_id << 16);
      }

      AGORA_LOG_SYMBOL(
          "TxRxWorkerHw[%zu]: TxReciprocityCalibPilots (Frame %zu, Symbol "
          "%zu, Ant %zu) - transmit ref pilot for uplink recip cal\n",
          tid_, frame_id, tx_symbol_id,
          (radio_id * Configuration()->NumChannels()) + ant_idx);
      // Check to see if the next symbol is a Tx symbol for the reference node
      const int tx_ret =
          radio_config_.RadioTx(radio_id, calultxbuf.data(),
                                GetTxFlags(radio_id, tx_symbol_id), frame_time);
      if (tx_ret != static_cast<int>(Configuration()->SampsPerSymbol())) {
        std::cerr << "BAD Transmit rep pilot on radio " << radio_id
                  << " - status " << tx_ret << ",  expected "
                  << Configuration()->SampsPerSymbol() << " at Time "
                  << frame_time << std::endl;
      }
    }
  } else {
    // ! ref_radio -- Transmit downlink calibration (array to ref) pilot
    // Send all CalDl symbols 'C'
    std::vector<const void*> caldltxbuf(Configuration()->NumChannels(),
                                        zeros_.data());

    // For each C only 1 channel / antenna can send pilots.  All others send zeros for now, per schedule.
    for (size_t dl_cal_sym_idx = 0;
         dl_cal_sym_idx < Configuration()->Frame().NumDLCalSyms();
         dl_cal_sym_idx++) {
      const size_t tx_symbol_id =
          Configuration()->Frame().GetDLCalSymbol(dl_cal_sym_idx);

      //Check to see if the pilot antenna is on radio_id
      const size_t calib_antenna =
          Configuration()->RecipCalDlAnt(frame_id, tx_symbol_id);

      const size_t calib_radio = calib_antenna / channels_per_interface_;
      const size_t channel_offset = calib_antenna % channels_per_interface_;

      AGORA_LOG_FRAME(
          "TxRxWorkerHw[%zu]: TxReciprocityCalibPilots (Frame %zu, Symbol "
          "%zu, Radio %zu) dl pilot tx has data %d on channel %zu\n",
          tid_, frame_id, tx_symbol_id, radio_id, calib_radio == radio_id,
          channel_offset);

      if (calib_radio == radio_id) {
        caldltxbuf.at(channel_offset) = Configuration()->PilotCi16().data();

        AGORA_LOG_SYMBOL(
            "TxRxWorkerHw[%zu]: TxReciprocityCalibPilots (Frame %zu, Symbol "
            "%zu, Ant %zu) - transmit pilot for downlink recip cal\n",
            tid_, frame_id, tx_symbol_id,
            (radio_id * channels_per_interface_) + channel_offset);
      }

      long long frame_time = 0;
      if (Configuration()->HwFramer() == false) {
        frame_time =
            time0 + (Configuration()->SampsPerSymbol() *
                     ((frame_id * Configuration()->Frame().NumTotalSyms()) +
                      tx_symbol_id));
      } else {
        frame_time = ((long long)(frame_id) << 32) | (tx_symbol_id << 16);
      }
      const int tx_status =
          radio_config_.RadioTx(radio_id, caldltxbuf.data(),
                                GetTxFlags(radio_id, tx_symbol_id), frame_time);
      if (tx_status != static_cast<int>(Configuration()->SampsPerSymbol())) {
        std::cerr << "BAD Transmit on radio " << radio_id << " - status "
                  << tx_status << ",  expected "
                  << Configuration()->SampsPerSymbol() << " at Time "
                  << frame_time << std::endl;
      }

      // Reset the caldltxbuf to zeros for next loop
      if (calib_radio == radio_id) {
        caldltxbuf.at(channel_offset) = zeros_.data();
      }
    }  // end s < Configuration()->RadioPerGroup()
  }    // ! ref radio
}

//Tx data
size_t TxRxWorkerHw::DoTx(long long time0) {
  auto tx_events = GetPendingTxEvents();

  //Process each pending tx event
  for (const EventData& current_event : tx_events) {
    assert(current_event.event_type_ == EventType::kPacketTX);

    const size_t frame_id = gen_tag_t(current_event.tags_[0u]).frame_id_;
    const size_t symbol_id = gen_tag_t(current_event.tags_[0u]).symbol_id_;
    const size_t ant_id = gen_tag_t(current_event.tags_[0u]).ant_id_;
    const size_t radio_id = ant_id / channels_per_interface_;
    const size_t tx_frame_id = (frame_id + TX_FRAME_DELTA);

    AGORA_LOG_TRACE(
        "TxRxWorkerHw[%zu]: (Frame %zu, Symbol %zu, Ant %zu) - Transmit ready "
        "for radio %zu [%zu:%zu]\n",
        tid_, frame_id, symbol_id, ant_id, radio_id, interface_offset_,
        (interface_offset_ + num_interfaces_) - 1);
    RtAssert((radio_id >= interface_offset_) &&
                 (radio_id <= ((interface_offset_ + num_interfaces_) - 1)),
             "Radio id does not match the radio for this worker");

    //See if this is the last antenna on the radio.  Assume that we receive the last one
    // last (and all the others came before).  No explicit tracking
    const bool last_antenna =
        ((ant_id % channels_per_interface_) + 1) == (channels_per_interface_);

    const size_t dl_symbol_idx =
        Configuration()->Frame().GetDLSymbolIdx(symbol_id);

    // All antenna data is ready to tx for a given symbol, if last then TX out the data
    if (last_antenna) {
      AGORA_LOG_TRACE(
          "TxRxWorkerHw[%zu]: (Frame %zu, Symbol %zu) last tx antenna %zu "
          "for radio %zu has tx data for all antennas / channels\n",
          tid_, frame_id, symbol_id, ant_id, radio_id);

      // When the first Tx symbol of the frame is ready (for a specific radio), schedule beacon and cals
      // assumes the symbols are in the correct order (ie get symbol 0 before symbol 1)
      if (symbol_id == Configuration()->Frame().GetDLSymbol(0)) {
        // Schedule beacon in the future
        if (Configuration()->HwFramer() == false) {
          TxBeaconHw(tx_frame_id, radio_id, time0);
        }

        if (Configuration()->Frame().IsRecCalEnabled()) {
          TxReciprocityCalibPilots(tx_frame_id, radio_id, time0);
        }
      }

      std::vector<const void*> txbuf(channels_per_interface_);
      if (kDebugDownlink == true) {
        for (size_t ch = 0; ch < channels_per_interface_; ch++) {
          if (!kBeamsweepData && ant_id != Configuration()->BeaconAnt()) {
            txbuf.at(ch) = zeros_.data();
          } else if (dl_symbol_idx <
                     Configuration()->Frame().ClientDlPilotSymbols()) {
            std::vector<std::complex<int16_t>> pilot(
                Configuration()->UeSpecificPilotT()[0],
                Configuration()->UeSpecificPilotT()[0] +
                    Configuration()->SampsPerSymbol());
            std::vector<std::complex<int16_t>> nt_pilot(pilot);
            for (auto& v : nt_pilot) {
              v = -v;
            }
            size_t fr_id = tx_frame_id % Configuration()->BfAntNum();
            txbuf.at(ch) =
                kBeamsweepData && CommsLib::Hadamard2(ant_id, fr_id) == -1
                    ? nt_pilot.data()
                    : pilot.data();
          } else {
            std::vector<std::complex<int16_t>> data_t(
                Configuration()->DlIqT()[dl_symbol_idx],
                Configuration()->DlIqT()[dl_symbol_idx] +
                    Configuration()->SampsPerSymbol());
            std::vector<std::complex<int16_t>> nt_data_t(data_t);
            for (auto& v : nt_data_t) {
              v = -v;
            }
            size_t fr_id = tx_frame_id % Configuration()->BfAntNum();
            txbuf.at(ch) =
                kBeamsweepData && CommsLib::Hadamard2(ant_id, fr_id) == -1
                    ? nt_data_t.data()
                    : data_t.data();
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
        frame_time =
            time0 + (Configuration()->SampsPerSymbol() *
                     ((tx_frame_id * Configuration()->Frame().NumTotalSyms()) +
                      symbol_id));
      } else {
        frame_time = ((long long)(tx_frame_id) << 32) | (symbol_id << 16);
      }
      const int radio_status = radio_config_.RadioTx(
          radio_id, txbuf.data(), GetTxFlags(radio_id, symbol_id), frame_time);
      if (radio_status != static_cast<int>(Configuration()->SampsPerSymbol())) {
        std::cerr << "BAD Transmit on radio " << radio_id << " - status "
                  << radio_status << ",  expected "
                  << Configuration()->SampsPerSymbol() << " at Time "
                  << frame_time << std::endl;
      }
    }

    if (kDebugPrintInTask == true) {
      std::printf(
          "TxRxWorkerHw[%zu]: Transmitted frame %zu, symbol %zu, ant %zu\n",
          tid_, frame_id, symbol_id, ant_id);
    }
    const auto complete_event =
        EventData(EventType::kPacketTX, current_event.tags_[0]);
    NotifyComplete(complete_event);
  }
  return tx_events.size();
}

//Checks to see if the current symbol is followed by another tx symbol
//Need the radio id, to check against the reference radio
bool TxRxWorkerHw::IsTxSymbolNext(size_t radio_id, size_t current_symbol) {
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

Radio::TxFlags TxRxWorkerHw::GetTxFlags(size_t radio_id, size_t tx_symbol_id) {
  Radio::TxFlags tx_flags;
  const auto tx_again = IsTxSymbolNext(radio_id, tx_symbol_id);
  if (tx_again) {
    tx_flags = Radio::TxFlags::kTxFlagNone;
  } else {
    tx_flags = Radio::TxFlags::kEndTransmit;
  }
  return tx_flags;
}

/// Returns the next symbol and interface
TxRxWorkerRx::RxParameters TxRxWorkerHw::UpdateRxInterface(
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

bool TxRxWorkerHw::IsRxSymbol(size_t interface, size_t symbol_id) {
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

void TxRxWorkerHw::PrintRxSymbolTiming(
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
        AGORA_LOG_TRACE(
            "TxRxWorkerHw[%zu]: Radio %zu Frame %zu Symbol %zu Rx "
            "Start uS %f for %f uS\n",
            tid_, i + interface_offset_, current_frame, current_symbol,
            GetTime::CyclesToUs(rx_times.at(i).start_ticks_, freq_ghz_),
            rx_us.at(i));

        rx_times.at(i).start_ticks_ = 0;
        rx_times.at(i).end_ticks_ = 0;
      }

      const size_t ticks_now = GetTime::Rdtsc();
      const double avg_rx_time = total_symbol_rx_time / num_interfaces_;
      std::ostringstream result_message;
      //Add any radio that is > 2x the average
      result_message << std::fixed << std::setprecision(2) << "TxRxWorkerHw["
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
      AGORA_LOG_INFO("%s\n", result_message.str().c_str());
    }

    //Frame RX complete -- print summary
    if (current_symbol > next_symbol) {
      AGORA_LOG_INFO("TxRxWorkerHw[%zu]: Frame %zu rx complete @ %.2f\n", tid_,
                     current_frame,
                     GetTime::CyclesToUs(
                         GetTime::Rdtsc() - program_start_ticks_, freq_ghz_));
    }
  }  //  if (kSymbolTimingEnabled)
}

// When Hw Framer is enabled, frame time keeping is done by hardware.
// In particular, the Hw only forward useful receive (no Guard) symbols.
// In this mode, the format of rx timestamp (64 bits)is frame number at
// 32-bits msb and symbol number at 16-bit msb of 32-bit lsb.
// When Hw Framer is disabled (false), frame time keeping in done entirely
// in software as it is implemented here. All time domain symbols are transfered
// to the host by HW where software will decide how to handle each symbol.
long long int TxRxWorkerHw::GetHwTime() {
  long long hw_time = 0;
  if (Configuration()->HwFramer() == false) {
    constexpr auto kBeacontxFlags = Radio::TxFlags::kEndTransmit;
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

    Radio::RxFlags out_flags;
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
          rx_status = radio_config_.RadioRx(rx_radio_id, samp_buffer,
                                            Configuration()->SampsPerSymbol(),
                                            out_flags, rx_time_bs);
        }
        //First Frame has been rx'd by the first radio
        tx_time_bs = rx_time_bs + frame_time * TX_FRAME_DELTA;
        hw_time = tx_time_bs;
        std::cout << "Received first data at time " << rx_time_bs
                  << " on thread " << tid_ << std::endl;
        std::cout << "Transmit first beacon at time " << tx_time_bs
                  << " on thread " << tid_ << std::endl;

        //Finish rx'ing symbol 0 on remaining radios
        for (size_t radio_id = rx_radio_id + 1;
             radio_id < rx_radio_id + num_interfaces_; radio_id++) {
          rx_status = radio_config_.RadioRx(radio_id, samp_buffer,
                                            Configuration()->SampsPerSymbol(),
                                            out_flags, rx_time_bs);
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
          std::cerr << "BAD Transmit on radio " << radio_id << " - status "
                    << tx_status << ",  expected "
                    << Configuration()->SampsPerSymbol() << " at Time "
                    << tx_time_bs << std::endl;
        }
      }  // end TX schedule

      // Rx all symbols on remaining radios
      for (size_t sym = rx_symbol;
           sym < Configuration()->Frame().NumTotalSyms(); sym++) {
        for (size_t radio_id = interface_offset_;
             radio_id < interface_offset_ + num_interfaces_; radio_id++) {
          rx_status = radio_config_.RadioRx(radio_id, samp_buffer,
                                            Configuration()->SampsPerSymbol(),
                                            out_flags, rx_time_bs);
          //---------------Check status?
        }
      }
    }
    std::cout << "Start BS main recv loop..." << std::endl;
  }  // HwFramer == false
  return hw_time;
}

//Maybe only schedule frame 0, then let the TX_FRAME_DELTA happen after the RX
void TxRxWorkerHw::ScheduleTxInit(size_t frames_to_schedule, long long time0) {
  for (size_t frame = 0; frame < frames_to_schedule; frame++) {
    for (size_t radio = interface_offset_;
         radio < (interface_offset_ + num_interfaces_); radio++) {
      AGORA_LOG_TRACE(
          "TxRxWorkerHw[%zu]: Scheduling frame %zu on interface %zu\n", tid_,
          frame, radio);
      if (Configuration()->HwFramer() == false) {
        TxBeaconHw(frame, radio, time0);
      }
      //Keep the assumption that Cal is before an 'D' symbols
      // Maybe a good idea to combine / optimize the schedule by iterating through the entire frame symbol by symbol
      if (Configuration()->Frame().IsRecCalEnabled() == true) {
        TxReciprocityCalibPilots(frame, radio, time0);
      }
      TxDownlinkZeros(frame, radio, time0);
    }  // for each radio
  }    // fpr each frame
}

// All DL symbols
void TxRxWorkerHw::TxDownlinkZeros(size_t frame_id, size_t radio_id,
                                   long long time0) {
  AGORA_LOG_FRAME(
      "TxRxWorkerHw[%zu]: TxDownlinkZeros frame %zu, interface %zu, time "
      "%lld\n",
      tid_, frame_id, radio_id, time0);
  //Pointing to the same tx location
  std::vector<const void*> tx_buffs(Configuration()->NumChannels(),
                                    zeros_.data());

  for (size_t dl_sym_idx = 0; dl_sym_idx < Configuration()->Frame().NumDLSyms();
       dl_sym_idx++) {
    const size_t tx_symbol_id =
        Configuration()->Frame().GetDLSymbol(dl_sym_idx);

    AGORA_LOG_TRACE(
        "TxRxWorkerHw[%zu]: TxDownlinkZeros frame %zu, symbol %zu:%zu, "
        "interface %zu, time %lld\n",
        tid_, frame_id, dl_sym_idx, tx_symbol_id, radio_id, time0);
    long long frame_time = 0;
    if (Configuration()->HwFramer() == false) {
      frame_time =
          time0 + (Configuration()->SampsPerSymbol() *
                   (((frame_id)*Configuration()->Frame().NumTotalSyms()) +
                    tx_symbol_id));
    } else {
      frame_time =
          (static_cast<long long>(frame_id) << 32) | (tx_symbol_id << 16);
    }

    const auto tx_flags = GetTxFlags(radio_id, tx_symbol_id);

    const int tx_ret =
        radio_config_.RadioTx(radio_id, tx_buffs.data(), tx_flags, frame_time);

    if (tx_ret != static_cast<int>(Configuration()->SampsPerSymbol())) {
      std::cerr << "BAD Transmit on radio " << radio_id << " - status "
                << tx_ret << ",  expected " << Configuration()->SampsPerSymbol()
                << " at Time " << frame_time << " with flags " << tx_flags
                << std::endl;
    }
  }  // end for all symbols
}

void TxRxWorkerHw::InitRxStatus() {
  rx_status_.resize(num_interfaces_,
                    TxRxWorkerRx::RxStatusTracker(channels_per_interface_));
  std::vector<RxPacket*> rx_packets(channels_per_interface_);
  for (auto& status : rx_status_) {
    for (auto& new_packet : rx_packets) {
      new_packet = &GetRxPacket();
      AGORA_LOG_TRACE("InitRxStatus[%zu]: Using Packet at location %d\n", tid_,
                      reinterpret_cast<intptr_t>(new_packet));
    }
    //Allocate memory for each interface / channel
    status.Reset(rx_packets);
  }
}

void TxRxWorkerHw::ResetRxStatus(size_t interface, bool reuse_memory) {
  auto& prev_status = rx_status_.at(interface);

  std::vector<RxPacket*> rx_packets;
  if (reuse_memory) {
    rx_packets = rx_status_.at(interface).GetRxPackets();
  } else {
    for (size_t packets = 0; packets < prev_status.NumChannels(); packets++) {
      rx_packets.emplace_back(&GetRxPacket());
    }
  }
  prev_status.Reset(rx_packets);
}