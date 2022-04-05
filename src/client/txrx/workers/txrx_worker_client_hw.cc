/**
 * @file txrx_worker_hw.cc
 * @brief Implementation of PacketTxRx datapath functions for communicating
 * with real iris / faros hardware
 */

#include "txrx_worker_client_hw.h"

#include <cassert>
#include <complex>

#include "logger.h"

static constexpr size_t kSyncDetectChannel = 0;
static constexpr bool kVerifyFirstSync = true;
static constexpr size_t kReSyncRetryCount = 100u;

TxRxWorkerClientHw::TxRxWorkerClientHw(
    size_t core_offset, size_t tid, size_t interface_count,
    size_t interface_offset, Config* const config, size_t* rx_frame_start,
    moodycamel::ConcurrentQueue<EventData>* event_notify_q,
    moodycamel::ConcurrentQueue<EventData>* tx_pending_q,
    moodycamel::ProducerToken& tx_producer,
    moodycamel::ProducerToken& notify_producer,
    std::vector<RxPacket>& rx_memory, std::byte* const tx_memory,
    std::mutex& sync_mutex, std::condition_variable& sync_cond,
    std::atomic<bool>& can_proceed, ClientRadioConfig& radio_config)
    : TxRxWorker(core_offset, tid, interface_count, interface_offset,
                 config->NumUeChannels(), config, rx_frame_start,
                 event_notify_q, tx_pending_q, tx_producer, notify_producer,
                 rx_memory, tx_memory, sync_mutex, sync_cond, can_proceed),
      radio_(radio_config),
      program_start_ticks_(0),
      frame_zeros_(
          config->NumUeChannels(),
          std::vector<std::complex<int16_t>>(
              (config->SampsPerSymbol() * config->Frame().NumTotalSyms()),
              std::complex<int16_t>(0, 0))),
      frame_storage_(
          config->NumUeChannels(),
          std::vector<std::complex<int16_t>>(
              (config->SampsPerSymbol() * config->Frame().NumTotalSyms()),
              std::complex<int16_t>(0, 0))),
      rx_frame_(config->NumUeChannels()) {
  for (size_t channel = 0; channel < config->NumUeChannels(); channel++) {
    rx_frame_.at(channel) = frame_storage_.at(channel).data();
  }
  RtAssert(interface_count == 1,
           "Interface count must be set to 1 for use with this class");

  RtAssert(config->UeHwFramer() == false, "Must have ue hw framer disabled");
}

TxRxWorkerClientHw::~TxRxWorkerClientHw() = default;

//Main Thread Execution loop
void TxRxWorkerClientHw::DoTxRx() {
  PinToCoreWithOffset(ThreadType::kWorkerTXRX, core_offset_, tid_);

  AGORA_LOG_INFO("TxRxWorkerClientHw[%zu] has %zu:%zu total radios %zu\n", tid_,
                 interface_offset_, (interface_offset_ + num_interfaces_) - 1,
                 num_interfaces_);

  running_ = true;

  const size_t samples_per_symbol = Configuration()->SampsPerSymbol();
  const size_t samples_per_frame =
      samples_per_symbol * Configuration()->Frame().NumTotalSyms();

  //Thread Sync
  WaitSync();
  program_start_ticks_ = GetTime::Rdtsc();

  if (num_interfaces_ == 0) {
    AGORA_LOG_WARN("TxRxWorkerClientHw[%zu] has no interfaces, exiting\n",
                   tid_);
    running_ = false;
    return;
  } else if (num_interfaces_ > 1) {
    throw std::runtime_error(
        "TxRxWorkerClientHw does not support multiple interfaces per thread");
  }
  const size_t local_interface = 0;

  // Keep receiving one frame of data until a beacon is found
  // Perform initial beacon detection every kBeaconDetectInterval frames
  long long rx_time = 0;

  ssize_t rx_adjust_samples = 0;

  const size_t beacon_detect_window =
      static_cast<size_t>(static_cast<float>(samples_per_symbol) * 2.33f);
  const size_t alignment_samples = samples_per_frame - beacon_detect_window;
  RtAssert(beacon_detect_window < samples_per_frame,
           "Frame must be greater than 3 Symbols");

  //Scope the variables
  {
    const ssize_t sync_index =
        SyncBeacon(local_interface, beacon_detect_window);
    if (sync_index >= 0) {
      rx_adjust_samples = sync_index - Configuration()->BeaconLen() -
                          Configuration()->OfdmTxZeroPrefix();
      AGORA_LOG_INFO(
          "TxRxWorkerClientHw [%zu]: Beacon detected for radio %zu, "
          "sync_index: %ld, rx sample offset: %ld\n",
          tid_, local_interface + interface_offset_, sync_index,
          rx_adjust_samples);

      AdjustRx(local_interface, alignment_samples + rx_adjust_samples);
      rx_adjust_samples = 0;
    } else if (Configuration()->Running()) {
      AGORA_LOG_WARN(
          "TxRxWorkerClientHw [%zu]: Beacon could not be detected on interface "
          "%zu - sync_index: %ld\n",
          tid_, local_interface, sync_index);
      throw std::runtime_error("rx sample offset is less than 0");
    }
  }
  long long time0 = 0;
  //Set initial frame and symbol to max value so we start at 0
  size_t rx_frame_id = SIZE_MAX;
  size_t rx_symbol_id = Configuration()->Frame().NumTotalSyms() - 1;

  bool resync = false;
  size_t resync_retry_cnt = 0;
  size_t resync_success = 0;
  const size_t max_cfo = 200;  // in ppb, For Iris
  // If JSON input if not default (0),
  // Else calculate based of ppb and frame length
  const size_t frame_sync_period =
      Configuration()->UeResyncPeriod() > 0
          ? Configuration()->UeResyncPeriod()
          : static_cast<size_t>(
                std::floor(1e9 / (max_cfo * Configuration()->SampsPerFrame())));

  std::stringstream sout;
  rx_adjust_samples = 0;

  //Establish time0 from symbol = 0 (beacon), frame 0
  while (Configuration()->Running() && (time0 == 0)) {
    const auto rx_pkts = DoRx(local_interface, rx_frame_id, rx_symbol_id,
                              rx_time, rx_adjust_samples);
    if (rx_pkts.size() == channels_per_interface_) {
      time0 = rx_time;

      if (kVerifyFirstSync) {
        for (size_t ch = 0; ch < channels_per_interface_; ch++) {
          const ssize_t sync_index = FindSyncBeacon(
              reinterpret_cast<std::complex<int16_t>*>(rx_pkts.at(ch)->data_),
              samples_per_symbol);
          if (sync_index >= 0) {
            rx_adjust_samples = sync_index - Configuration()->BeaconLen() -
                                Configuration()->OfdmTxZeroPrefix();
            AGORA_LOG_INFO(
                "TxRxWorkerClientHw [%zu]: Initial Sync - radio %zu, frame "
                "%zu, symbol %zu sync_index: %ld, rx sample offset: %ld time0 "
                "%lld\n",
                tid_, (local_interface + interface_offset_) + ch, rx_frame_id,
                rx_symbol_id, sync_index, rx_adjust_samples, time0);
          } else {
            throw std::runtime_error(
                "No Beacon Detected at Frame 0 / Symbol 0");
          }
        }
      }  // end verify first sync
    }    // received Frame 0 Symbol 0
  }      // end - establish time0 for a given interface

  //No Need to preschedule the TX_FRAME_DELTA init in software framer mode

  //Beacon sync detected run main rx routines
  while (Configuration()->Running()) {
    if ((Configuration()->FramesToTest() > 0) &&
        (rx_frame_id > Configuration()->FramesToTest())) {
      Configuration()->Running(false);
      break;
    }

    //Attempt Tx
    const size_t tx_status = DoTx(time0);
    if (tx_status == 0) {
      const auto rx_pkts = DoRx(local_interface, rx_frame_id, rx_symbol_id,
                                rx_time, rx_adjust_samples);
      if (rx_pkts.size() == channels_per_interface_) {
        if (kDebugPrintInTask) {
          AGORA_LOG_INFO(
              "DoTxRx[%zu]: radio %zu received frame id %zu, symbol id %zu at "
              "time %lld\n",
              tid_, local_interface + interface_offset_, rx_frame_id,
              rx_symbol_id, rx_time);
        }
        // resync every frame_sync_period frames:
        // Only sync on beacon symbols
        if ((rx_symbol_id == Configuration()->Frame().GetBeaconSymbolLast()) &&
            ((rx_frame_id / frame_sync_period) > 0) &&
            ((rx_frame_id % frame_sync_period) == 0)) {
          resync = true;
        }

        //If we have a beacon and we would like to resync
        if (resync &&
            (rx_symbol_id == Configuration()->Frame().GetBeaconSymbolLast())) {
          //This is adding a race condition on this data, it is ok for now but we should fix this
          const ssize_t sync_index =
              FindSyncBeacon(reinterpret_cast<std::complex<int16_t>*>(
                                 rx_pkts.at(kSyncDetectChannel)->data_),
                             samples_per_symbol);
          if (sync_index >= 0) {
            rx_adjust_samples = sync_index - Configuration()->BeaconLen() -
                                Configuration()->OfdmTxZeroPrefix();
            AGORA_LOG_INFO(
                "TxRxWorkerClientHw [%zu]: Re-syncing radio %zu, sync_index: "
                "%ld, rx sample offset: %ld tries %zu\n",
                tid_, local_interface + interface_offset_, sync_index,
                rx_adjust_samples, resync_retry_cnt);
            resync_success++;
            resync = false;

            //Adjust the transmit time offset
            time0 += rx_adjust_samples;
            resync_retry_cnt = 0;
          } else {
            resync_retry_cnt++;
            if (resync_retry_cnt > kReSyncRetryCount) {
              AGORA_LOG_ERROR(
                  "TxRxWorkerClientHw [%zu]: Exceeded resync retry limit (%zu) "
                  "for client %zu reached after %zu resync successes at frame: "
                  "%zu.  Stopping!\n",
                  tid_, kReSyncRetryCount, local_interface + interface_offset_,
                  resync_success, rx_frame_id);
              Configuration()->Running(false);
              break;
            }
          }
        }  // end resync
      } else if (!rx_pkts.empty()) {
        throw std::runtime_error(
            "Received data but it was not the correct dimension");
      }
    }
  }  // end main while loop
  running_ = false;
}

//RX data, should return channel number of packets || 0
// global_frame_id  in - frame id of the last rx packet
//                 out - frame id of the current rx packet
// global_symbol_id in - symbol id of the last rx packet
//                 out - symbol id of the current rx packet
//
std::vector<Packet*> TxRxWorkerClientHw::DoRx(size_t interface_id,
                                              size_t& global_frame_id,
                                              size_t& global_symbol_id,
                                              long long& receive_time,
                                              ssize_t& sample_offset) {
  const size_t radio_id = interface_id + interface_offset_;
  const size_t first_ant_id = radio_id * channels_per_interface_;
  std::vector<Packet*> result_packets;

  size_t num_rx_samps = Configuration()->SampsPerSymbol();

  std::vector<RxPacket*> memory_tracking(channels_per_interface_);
  std::vector<size_t> ant_ids(channels_per_interface_);
  std::vector<void*> rx_samples(channels_per_interface_);

  //Allocate memory for reception
  for (size_t ch = 0; ch < channels_per_interface_; ch++) {
    RxPacket& rx = GetRxPacket();
    memory_tracking.at(ch) = &rx;
    ant_ids.at(ch) = first_ant_id + ch;
    //Align current symbol, and read less for remaining (+2 is for I/Q)
    if (sample_offset < 0) {
      const size_t padding = (-2 * sample_offset);
      std::memset(rx.RawPacket()->data_, 0u, padding);
      rx_samples.at(ch) = &rx.RawPacket()->data_[padding];
    } else {
      rx_samples.at(ch) = rx.RawPacket()->data_;
    }
    AGORA_LOG_TRACE("TxRxWorkerClientHw[%zu]: Using Packet at location %zu\n",
                    tid_, reinterpret_cast<size_t>(&rx));
  }

  //Sample offset alignment
  if (sample_offset < 0) {
    //Don't read an entire symbol due to the offset
    num_rx_samps = num_rx_samps + sample_offset;
  } else if (sample_offset > 0) {
    //Otherwise throw out the offset (could just add this to the next symbol but our buffers are not large enough)
    num_rx_samps = sample_offset;
  }

  const int rx_status =
      radio_.RadioRx(radio_id, rx_samples.data(), num_rx_samps, receive_time);
  if (rx_status < 0) {
    AGORA_LOG_ERROR(
        "TxRxWorkerClientHw[%zu]: Interface %zu | Radio %zu - Rx failure RX "
        "status = %d is less than 0\n",
        tid_, interface_id, interface_id + interface_offset_, rx_status);
  } else if (static_cast<size_t>(rx_status) != num_rx_samps) {
    AGORA_LOG_ERROR(
        "TxRxWorkerClientHw[%zu]: Interface %zu | Radio %zu - Rx failure RX "
        "status = %d is less than num samples %zu\n",
        tid_, interface_id, interface_id + interface_offset_, rx_status,
        num_rx_samps);
  } else {
    //sample_offset > 0 means we ignore the rx'd data (don't update the symbol / frame tracking)
    if (sample_offset <= 0) {
      // Expected rx, Set the symbol / frame id's
      if (Configuration()->UeHwFramer()) {
        global_frame_id = static_cast<size_t>(receive_time >> 32);
        global_symbol_id = static_cast<size_t>((receive_time >> 16) & 0xFFFF);
      } else {
        //Update the rx-symbol
        global_symbol_id++;
        if (global_symbol_id == Configuration()->Frame().NumTotalSyms()) {
          global_symbol_id = 0;
          global_frame_id++;
        }
      }

      if (kDebugPrintInTask) {
        AGORA_LOG_INFO(
            "TxRxWorkerClientHw [%zu]: Rx (Frame %zu, Symbol %zu, Radio "
            "%zu) - at time %lld\n",
            tid_, global_frame_id, global_symbol_id, radio_id, receive_time);
      }

      if (IsRxSymbol(global_symbol_id)) {
        for (size_t ant = 0; ant < ant_ids.size(); ant++) {
          auto* rx_packet = memory_tracking.at(ant);
          auto* raw_pkt = rx_packet->RawPacket();
          new (raw_pkt)
              Packet(global_frame_id, global_symbol_id, 0, ant_ids.at(ant));
          result_packets.push_back(raw_pkt);

          AGORA_LOG_FRAME(
              "TxRxWorkerClientHw [%zu]: Rx Downlink (Frame %zu, Symbol %zu, "
              "Ant %zu) from Radio %zu at time %lld\n",
              tid_, global_frame_id, global_symbol_id, ant_ids.at(ant),
              radio_id, receive_time);

          // Push kPacketRX event into the queue.
          EventData rx_message(EventType::kPacketRX, rx_tag_t(*rx_packet).tag_);
          NotifyComplete(rx_message);
          memory_tracking.at(ant) = nullptr;
        }
      }  // end is RxSymbol
    }    // sample offset <= 0
    sample_offset = 0;
  }

  //Free memory from most recent allocated to latest
  AGORA_LOG_TRACE("TxRxWorkerClientHw[%zu]: Memory allocation %zu\n", tid_,
                  memory_tracking.size());
  for (ssize_t idx = (memory_tracking.size() - 1); idx > -1; idx--) {
    auto* memory_location = memory_tracking.at(idx);
    AGORA_LOG_TRACE("TxRxWorkerClientHw[%zu]: Checking location %zu\n", tid_,
                    (intptr_t)memory_location);
    if (memory_location != nullptr) {
      AGORA_LOG_TRACE(
          "TxRxWorkerClientHw[%zu]: Returning Packet at location %zu\n", tid_,
          (intptr_t)memory_location);
      ReturnRxPacket(*memory_location);
    }
  }
  return result_packets;
}

//Tx data
size_t TxRxWorkerClientHw::DoTx(const long long time0) {
  auto tx_events = GetPendingTxEvents();

  for (const EventData& current_event : tx_events) {
    RtAssert((current_event.event_type_ == EventType::kPacketTX) ||
                 (current_event.event_type_ == EventType::kPacketPilotTX),
             "Wrong Event Type in TX Queue!");

    //Assuming the 1 message per radio per frame
    const size_t frame_id = gen_tag_t(current_event.tags_[0u]).frame_id_;
    const size_t ue_ant = gen_tag_t(current_event.tags_[0u]).ue_id_;
    const size_t interface_id = ue_ant / channels_per_interface_;
    const size_t ant_offset = ue_ant % channels_per_interface_;

    AGORA_LOG_FRAME(
        "TxRxWorkerClientHw::DoTx[%zu]: Request to Transmit (Frame %zu, "
        "User %zu, Ant %zu) time0 %lld\n",
        tid_, frame_id, interface_id, ue_ant, time0);

    RtAssert((interface_id >= interface_offset_) &&
                 (interface_id <= (num_interfaces_ + interface_offset_)),
             "Invalid Tx interface Id");
    RtAssert(interface_id == tid_,
             "TxRxWorkerClientHw::DoTx - Ue id was not the expected values");

    //For Tx we need all channels_per_interface_ antennas before we can transmit
    //we will assume that if you get the last antenna, you have already received all
    //other antennas (enforced in the passing utility)
    if ((ant_offset + 1) == channels_per_interface_) {
      // Transmit pilot(s)
      if (!Configuration()->UeHwFramer()) {
        for (size_t ch = 0; ch < channels_per_interface_; ch++) {
          const size_t pilot_ant =
              (interface_id * channels_per_interface_) + ch;
          //Each pilot will be in a different tx slot (called for each pilot)
          TxPilot(pilot_ant, frame_id, time0);

          //Pilot transmit complete for pilot ue
          if (current_event.event_type_ == EventType::kPacketPilotTX) {
            auto complete_event =
                EventData(EventType::kPacketPilotTX,
                          gen_tag_t::FrmSymUe(frame_id, 0, pilot_ant).tag_);
            NotifyComplete(complete_event);
          }
        }  //For each channel
      }

      if (current_event.event_type_ == EventType::kPacketTX) {
        // Transmit data for all symbols (each cannel transmits for each symbol)
        TxUplinkSymbols(interface_id, frame_id, time0);
        //Notify the tx is complete for all antennas on the interface
        for (size_t ch = 0; ch < channels_per_interface_; ch++) {
          const size_t tx_ant = (interface_id * channels_per_interface_) + ch;
          //Frame transmit complete
          auto complete_event =
              EventData(EventType::kPacketTX,
                        gen_tag_t::FrmSymUe(frame_id, 0, tx_ant).tag_);
          NotifyComplete(complete_event);
        }
        AGORA_LOG_TRACE(
            "TxRxWorkerClientHw::DoTx[%zu]: Frame %zu Transmit Complete for Ue "
            "%zu\n",
            tid_, frame_id, interface_id);
      }
    }
  }  // End all events
  return tx_events.size();
}

void TxRxWorkerClientHw::AdjustRx(size_t local_interface,
                                  size_t discard_samples) {
  const size_t radio_id = local_interface + interface_offset_;
  long long rx_time = 0;
  while (Configuration()->Running() && (discard_samples > 0)) {
    const int rx_status =
        radio_.RadioRx(radio_id, rx_frame_.data(), discard_samples, rx_time);

    if (rx_status < 0) {
      std::cerr << "RadioTxRx [" << radio_id << "]: BAD SYNC Receive("
                << rx_status << "/" << discard_samples << ") at Time "
                << rx_time << std::endl;
    } else {
      discard_samples = discard_samples - rx_status;
    }
  }
}

ssize_t TxRxWorkerClientHw::SyncBeacon(size_t local_interface,
                                       size_t sample_window) {
  const size_t radio_id = local_interface + interface_offset_;
  ssize_t sync_index = -1;
  long long rx_time = 0;
  assert(sample_window <= (Configuration()->SampsPerSymbol() *
                           Configuration()->Frame().NumTotalSyms()));

  //\todo add a retry exit.
  while ((Configuration()->Running() == true) && (sync_index < 0)) {
    const int rx_status =
        radio_.RadioRx(radio_id, rx_frame_.data(), sample_window, rx_time);

    if (rx_status != static_cast<int>(sample_window)) {
      std::cerr << "RadioTxRx [" << radio_id << "]: BAD SYNC Receive("
                << rx_status << "/" << sample_window << ") at Time " << rx_time
                << std::endl;
    } else {
      sync_index = FindSyncBeacon(reinterpret_cast<std::complex<int16_t>*>(
                                      rx_frame_.at(kSyncDetectChannel)),
                                  sample_window);
    }
  }  // end while sync_index < 0
  return sync_index;
}

ssize_t TxRxWorkerClientHw::FindSyncBeacon(std::complex<int16_t>* check_data,
                                           size_t sample_window) {
  ssize_t sync_index = -1;
  assert(sample_window <= (Configuration()->SampsPerSymbol() *
                           Configuration()->Frame().NumTotalSyms()));

  //Allocate memory, only used for beacon detection
  std::vector<std::complex<float>> sync_compare(
      sample_window, std::complex<float>(0.0f, 0.0f));

  // convert entire frame data to complex float for sync detection
  for (size_t i = 0; i < sample_window; i++) {
    sync_compare.at(i) = (std::complex<float>(
        static_cast<float>(check_data[i].real()) / 32768.0f,
        static_cast<float>(check_data[i].imag()) / 32768.0f));
  }
  sync_index =
      CommsLib::FindBeaconAvx(sync_compare, Configuration()->GoldCf32());
  return sync_index;
}

bool TxRxWorkerClientHw::IsRxSymbol(size_t symbol_id) {
  auto symbol_type = Configuration()->GetSymbolType(symbol_id);
  bool is_rx;

  if ((symbol_type == SymbolType::kBeacon) ||
      (symbol_type == SymbolType::kDL)) {
    is_rx = true;
  } else {
    is_rx = false;
  }
  return is_rx;
}

// All UL symbols
void TxRxWorkerClientHw::TxUplinkSymbols(size_t radio_id, size_t frame_id,
                                         long long time0) {
  const size_t tx_frame_id = frame_id + TX_FRAME_DELTA;
  const size_t samples_per_symbol = Configuration()->SampsPerSymbol();
  const size_t samples_per_frame =
      samples_per_symbol * Configuration()->Frame().NumTotalSyms();
  long long tx_time;
  int flags_tx = 1;  // HAS_TIME

  std::vector<void*> tx_data(channels_per_interface_);
  for (size_t ul_symbol_idx = 0;
       ul_symbol_idx < Configuration()->Frame().NumULSyms(); ul_symbol_idx++) {
    const size_t tx_symbol_id =
        Configuration()->Frame().GetULSymbol(ul_symbol_idx);

    for (size_t ch = 0; ch < channels_per_interface_; ch++) {
      const size_t tx_ant = (radio_id * channels_per_interface_) + ch;
      auto* pkt = GetUlTxPacket(frame_id, tx_symbol_id, tx_ant);
      tx_data.at(ch) = reinterpret_cast<void*>(pkt->data_);
    }

    if (Configuration()->UeHwFramer()) {
      tx_time = ((long long)tx_frame_id << 32) | (tx_symbol_id << 16);
    } else {
      tx_time = time0 + (tx_frame_id * samples_per_frame) +
                (tx_symbol_id * samples_per_symbol) -
                Configuration()->ClTxAdvance().at(radio_id);
    }

    if (tx_symbol_id == Configuration()->Frame().GetULSymbolLast()) {
      flags_tx = 2;  // HAS_TIME & END_BURST, fixme
    }
    const int tx_status = radio_.RadioTx(radio_id, tx_data.data(),
                                         samples_per_symbol, flags_tx, tx_time);
    if (tx_status < static_cast<int>(samples_per_symbol)) {
      std::cout << "BAD Write (UL): For Ue " << radio_id << " " << tx_status
                << "/" << samples_per_symbol << std::endl;
    }
    if (kDebugPrintInTask) {
      AGORA_LOG_INFO(
          "TxRxWorkerClientHw::DoTx[%zu]: Transmitted Symbol (Frame "
          "%zu:%zu, Symbol %zu, Ue %zu) at time %lld flags %d\n",
          tid_, frame_id, tx_frame_id, tx_symbol_id, radio_id, tx_time,
          flags_tx);
    }
  }
}

void TxRxWorkerClientHw::TxPilot(size_t pilot_ant, size_t frame_id,
                                 long long time0) {
  const size_t tx_frame_id = frame_id + TX_FRAME_DELTA;
  const size_t pilot_channel = (pilot_ant % channels_per_interface_);
  const size_t radio = pilot_ant / channels_per_interface_;
  const size_t samples_per_symbol = Configuration()->SampsPerSymbol();
  const size_t samples_per_frame =
      samples_per_symbol * Configuration()->Frame().NumTotalSyms();
  long long tx_time;

  std::vector<void*> tx_data(channels_per_interface_);
  for (size_t ch = 0; ch < channels_per_interface_; ch++) {
    if (ch == pilot_channel) {
      tx_data.at(ch) = Configuration()->PilotCi16().data();
    } else {
      tx_data.at(ch) = frame_zeros_.at(ch).data();
    }
  }

  const size_t pilot_symbol_id =
      Configuration()->Frame().GetPilotSymbol(pilot_ant);

  int flags_tx = 2;  // END_BURST
  //See if we need to set end burst for the last channel
  // (see if the next symbol is an uplink symbol)
  if ((pilot_channel + 1) == channels_per_interface_) {
    if (Configuration()->Frame().NumULSyms() > 0) {
      const size_t first_ul_symbol = Configuration()->Frame().GetULSymbol(0);
      if ((pilot_symbol_id + 1) == (first_ul_symbol)) {
        flags_tx = 1;
      }
    }
  } else {
    flags_tx = 1;
  }

  if (Configuration()->UeHwFramer()) {
    tx_time = ((long long)tx_frame_id << 32) | (pilot_symbol_id << 16);
  } else {
    tx_time = time0 + (tx_frame_id * samples_per_frame) +
              (pilot_symbol_id * samples_per_symbol) -
              Configuration()->ClTxAdvance().at(radio);
  }

  const int tx_status = radio_.RadioTx(radio, tx_data.data(),
                                       samples_per_symbol, flags_tx, tx_time);

  if (tx_status < 0) {
    std::cout << "BAD Radio Tx: (PILOT)" << tx_status << "For Ue Radio "
              << radio << "/" << samples_per_symbol << std::endl;
  } else if (tx_status != static_cast<int>(samples_per_symbol)) {
    std::cout << "BAD Write: (PILOT)" << tx_status << "For Ue Radio " << radio
              << "/" << samples_per_symbol << std::endl;
  }

  if (kDebugPrintInTask) {
    AGORA_LOG_INFO(
        "TxRxWorkerClientHw::DoTx[%zu]: Transmitted Pilot  (Frame "
        "%zu:%zu, Symbol %zu, Ue %zu, Ant %zu:%zu) at time %lld flags "
        "%d\n",
        tid_, frame_id, tx_frame_id, pilot_symbol_id, radio, pilot_channel,
        pilot_ant, tx_time, flags_tx);
  }
}
