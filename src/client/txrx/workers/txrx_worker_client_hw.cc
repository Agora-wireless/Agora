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
static constexpr size_t kFrameSync = 1000u;

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
      rx_frame_(config->NumUeChannels(), nullptr) {
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

  MLPD_INFO("TxRxWorkerClientHw[%zu] has %zu:%zu total radios %zu\n", tid_,
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
    MLPD_WARN("TxRxWorkerClientHw[%zu] has no interfaces, exiting\n", tid_);
    running_ = false;
    return;
  }

  const size_t radio_id = tid_;

  // Keep receiving one frame of data until a beacon is found
  // Perform initial beacon detection every kBeaconDetectInterval frames
  long long rx_time = 0;

  ssize_t rx_adjust_samples = 0;
  //Scope the variables
  {
    const ssize_t sync_index = SyncBeacon(radio_id, samples_per_frame);
    if (sync_index >= 0) {
      rx_adjust_samples = sync_index - Configuration()->BeaconLen() -
                          Configuration()->OfdmTxZeroPrefix();
      MLPD_INFO(
          "TxRxWorkerClientHw [%zu]: Beacon detected, sync_index: %ld, rx "
          "sample offset: %ld\n",
          radio_id, sync_index, rx_adjust_samples);
    } else {
      MLPD_WARN(
          "TxRxWorkerClientHw [%zu]: Beacon could not be detected - "
          "sync_index: %ld\n",
          radio_id, sync_index);
      throw std::runtime_error("rx sample offset is less than 0");
    }
  }
  long long time0 = 0;
  size_t rx_frame_id = 0;
  size_t rx_symbol_id = 0;

  bool resync = false;
  size_t resync_retry_cnt = 0;
  static constexpr size_t kReSyncRetryCount = 100;
  size_t resync_success = 0;

  std::stringstream sout;

  //Establish time0 from symbol = 0 (beacon), frame 0
  while (Configuration()->Running() && (time0 == 0)) {
    const auto rx_pkts =
        DoRx(radio_id, rx_frame_id, rx_symbol_id, rx_time, rx_adjust_samples);
    if (rx_pkts.size() == channels_per_interface_) {
      time0 = rx_time;
    }
  }

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
      const auto rx_pkts =
          DoRx(radio_id, rx_frame_id, rx_symbol_id, rx_time, rx_adjust_samples);
      if (rx_pkts.size() == channels_per_interface_) {
        if (kDebugPrintInTask) {
          std::printf(
              "DoTxRx [%zu]: radio %zu received frame id %zu, symbol id "
              "%zu at time %lld\n",
              tid_, radio_id, rx_frame_id, rx_symbol_id, rx_time);
        }
        // resync every kFrameSync frames:
        // Only sync on beacon symbols
        ///\todo: kFrameSync should be a function of sample rate and max CFO
        if ((rx_symbol_id == Configuration()->Frame().GetBeaconSymbolLast()) &&
            ((rx_frame_id / kFrameSync) > 0) &&
            ((rx_frame_id % kFrameSync) == 0)) {
          resync = true;
        }

        //If we have a beacon and we would like to resynch
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
            MLPD_INFO(
                "TxRxWorkerClientHw [%zu]: Re-syncing radio %zu, sync_index: "
                "%ld, rx sample offset: %ld tries %zu\n",
                tid_, radio_id, sync_index, rx_adjust_samples,
                resync_retry_cnt);
            resync_success++;
            resync = false;

            //Adjust the transmit time offset
            time0 += rx_adjust_samples;
            resync_retry_cnt = 0;
          } else {
            resync_retry_cnt++;
            if (resync_retry_cnt > kReSyncRetryCount) {
              MLPD_ERROR(
                  "TxRxWorkerClientHw [%zu]: Exceeded resync retry limit (%zu) "
                  "for client %zu reached after %zu resync successes at frame: "
                  "%zu.  Stopping!\n",
                  tid_, kReSyncRetryCount, radio_id, resync_success,
                  rx_frame_id);
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
// global_frame_id will be used and updated
// global_symbol_id will be used and updated
std::vector<Packet*> TxRxWorkerClientHw::DoRx(size_t interface_id,
                                              size_t& global_frame_id,
                                              size_t& global_symbol_id,
                                              long long& receive_time,
                                              ssize_t& sample_offset) {
  const size_t radio_id = interface_id + interface_offset_;
  const size_t ant_id = radio_id * channels_per_interface_;
  std::vector<Packet*> result_packets;

  size_t num_rx_samps = Configuration()->SampsPerSymbol();

  std::vector<RxPacket*> memory_tracking(channels_per_interface_);
  std::vector<size_t> ant_ids(channels_per_interface_);
  std::vector<void*> samp(channels_per_interface_);

  //Allocate memory for reception
  for (size_t ch = 0; ch < channels_per_interface_; ch++) {
    RxPacket& rx = GetRxPacket();
    memory_tracking.at(ch) = &rx;
    ant_ids.at(ch) = ant_id + ch;
    //Align current symbol, and read less for remaining (+2 is for I/Q)
    if (sample_offset < 0) {
      const size_t padding = (-2 * sample_offset);
      std::memset(rx.RawPacket()->data_, 0, padding);
      samp.at(ch) = &rx.RawPacket()->data_[padding];
    } else {
      samp.at(ch) = rx.RawPacket()->data_;
    }
    MLPD_TRACE("TxRxWorkerClientHw[%zu]: Using Packet at location %zu\n", tid_,
               reinterpret_cast<size_t>(&rx));
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
      radio_.RadioRx(radio_id, samp.data(), num_rx_samps, receive_time);
  if (rx_status < 0) {
    MLPD_ERROR(
        "TxRxWorkerClientHw[%zu]: Interface %zu | Radio %zu - Rx failure RX "
        "status = %d is less than 0\n",
        tid_, interface_id, interface_id + interface_offset_, rx_status);
  } else if (static_cast<size_t>(rx_status) != num_rx_samps) {
    std::cerr << "RX [" << tid_ << "]: BAD Receive(" << rx_status << "/"
              << num_rx_samps << ") at Time " << receive_time << std::endl;
  } else {
    //sample_offset > 0 means we ignore the rx'd data (don't update the symbol / frame tracking)
    if (sample_offset <= 0) {
      // Expected rx, Set the symbol / frame id's
      if (Configuration()->UeHwFramer()) {
        global_frame_id = static_cast<size_t>(receive_time >> 32);
        global_symbol_id = static_cast<size_t>((receive_time >> 16) & 0xFFFF);
      }

      if (kDebugPrintInTask) {
        std::printf(
            "TxRxWorkerClientHw [%zu]: rx'd frame %zu, symbol %zu, radio id "
            "%zu rxtime %llx\n",
            tid_, global_frame_id, global_symbol_id, radio_id, receive_time);
      }

      if (IsRxSymbol(global_symbol_id)) {
        for (size_t ant = 0; ant < ant_ids.size(); ant++) {
          auto* rx_packet = memory_tracking.at(ant);
          auto* raw_pkt = rx_packet->RawPacket();
          new (raw_pkt)
              Packet(global_frame_id, global_symbol_id, 0, ant_ids.at(ant));
          result_packets.push_back(raw_pkt);

          // Push kPacketRX event into the queue.
          EventData rx_message(EventType::kPacketRX, rx_tag_t(*rx_packet).tag_);
          NotifyComplete(rx_message);
          memory_tracking.at(ant) = nullptr;
        }
      }  // end is RxSymbol

      //Update the rx-symbol
      if (!Configuration()->UeHwFramer()) {
        global_symbol_id++;
        if (global_symbol_id == Configuration()->Frame().NumTotalSyms()) {
          global_symbol_id = 0;
          global_frame_id++;
        }
      }
    }  // sample offset <= 0
    sample_offset = 0;
  }

  //Free memory from most recent allocated to latest
  MLPD_TRACE("TxRxWorkerClientHw[%zu]: Memory allocation %zu\n", tid_,
             memory_tracking.size());
  for (ssize_t idx = (memory_tracking.size() - 1); idx > -1; idx--) {
    auto* memory_location = memory_tracking.at(idx);
    MLPD_TRACE("TxRxWorkerClientHw[%zu]: Checking location %zu\n", tid_,
               (intptr_t)memory_location);
    if (memory_location != nullptr) {
      MLPD_TRACE("TxRxWorkerClientHw[%zu]: Returning Packet at location %zu\n",
                 tid_, (intptr_t)memory_location);
      ReturnRxPacket(*memory_location);
    }
  }
  return result_packets;
}

//Tx data
size_t TxRxWorkerClientHw::DoTx(const long long time0) {
  const size_t samples_per_symbol = Configuration()->SampsPerSymbol();
  const size_t samples_per_frame =
      samples_per_symbol * Configuration()->Frame().NumTotalSyms();

  auto tx_events = GetPendingTxEvents();

  for (const EventData& current_event : tx_events) {
    RtAssert((current_event.event_type_ == EventType::kPacketTX) ||
                 (current_event.event_type_ == EventType::kPacketPilotTX),
             "Wrong Event Type in TX Queue!");

    //Assuming the 1 message per radio per frame
    const size_t frame_id = gen_tag_t(current_event.tags_[0u]).frame_id_;
    //ue_id = radio_id
    const size_t ue_id = gen_tag_t(current_event.tags_[0u]).ue_id_;
    const size_t ue_ant_start = ue_id * channels_per_interface_;
    const size_t tx_frame_id = (frame_id + TX_FRAME_DELTA);
    long long tx_time = 0;
    //Place zeros in all dimensions
    std::vector<void*> tx_data(channels_per_interface_);
    for (size_t ch = 0; ch < channels_per_interface_; ch++) {
      tx_data.at(ch) = frame_zeros_.at(ch).data();
    }

    MLPD_INFO("TxRxWorkerClientHw::DoTx[%zu]: Transmitted frame %zu, ue %zu\n",
              tid_, frame_id, ue_id);
    RtAssert(ue_id == tid_,
             "TxRxWorkerClientHw::DoTx - Ue id was not the expected values");

    // Transmit pilot(s)
    // For UHD devices, first pilot should not be with the END_BURST flag
    // 1: HAS_TIME, 2: HAS_TIME | END_BURST
    int flags_tx = 1;

    if (!Configuration()->UeHwFramer()) {
      for (size_t ch = 0; ch < channels_per_interface_; ch++) {
        const size_t ant_pilot = ue_ant_start + ch;
        const size_t pilot_symbol_id =
            Configuration()->Frame().GetPilotSymbol(ant_pilot);

        //See if we need to set end burst for the last channel
        // (see if the next symbol is an uplink symbol)
        if ((ch + 1) == channels_per_interface_) {
          if (Configuration()->Frame().NumULSyms() > 0) {
            const size_t first_ul_symbol =
                Configuration()->Frame().GetULSymbol(0);
            if ((pilot_symbol_id + 1) == (first_ul_symbol)) {
              flags_tx = 2;
            }
          }
        }

        //Add the pilot to the correct index
        tx_data.at(ch) = Configuration()->PilotCi16().data();

        tx_time = time0 + (tx_frame_id * samples_per_frame) +
                  (pilot_symbol_id * samples_per_symbol) -
                  Configuration()->ClTxAdvance().at(ue_id);

        const int tx_status = radio_.RadioTx(
            ue_id, tx_data.data(), samples_per_symbol, flags_tx, tx_time);

        if (tx_status < 0) {
          std::cout << "BAD Radio Tx: (PILOT)" << tx_status << "For Ue Radio "
                    << ue_id << "/" << samples_per_symbol << std::endl;
        } else if (tx_status != static_cast<int>(samples_per_symbol)) {
          std::cout << "BAD Write: (PILOT)" << tx_status << "For Ue Radio "
                    << ue_id << "/" << samples_per_symbol << std::endl;
        }

        if (kDebugPrintInTask) {
          std::printf(
              "TxRxWorkerClientHw [%zu]: Transmitted frame %zu, pilot symbol "
              "%zu, ue %zu\n",
              tid_, frame_id, pilot_symbol_id, ue_id);
        }
        //Replace the pilot with zeros for next channel pilot
        tx_data.at(ch) = frame_zeros_.at(ch).data();
      }
    }

    //Pilot transmit complete
    if (current_event.event_type_ == EventType::kPacketPilotTX) {
      auto complete_event =
          EventData(EventType::kPacketPilotTX, current_event.tags_[0]);
      NotifyComplete(complete_event);
    } else {
      // Transmit data for all symbols
      flags_tx = 1;
      for (size_t symbol_id = 0;
           symbol_id < Configuration()->Frame().NumULSyms(); symbol_id++) {
        const size_t tx_symbol_id =
            Configuration()->Frame().GetULSymbol(symbol_id);

        for (size_t ch = 0; ch < channels_per_interface_; ch++) {
          auto* pkt = GetUlTxPacket(frame_id, symbol_id, ue_ant_start + ch);
          tx_data.at(ch) = reinterpret_cast<void*>(pkt->data_);
        }
        if (Configuration()->UeHwFramer()) {
          tx_time = ((long long)tx_frame_id << 32) | (tx_symbol_id << 16);
        } else {
          tx_time = time0 + (tx_frame_id * samples_per_frame) +
                    (tx_symbol_id * samples_per_symbol) -
                    Configuration()->ClTxAdvance().at(ue_id);
        }

        if (tx_symbol_id == Configuration()->Frame().GetULSymbolLast()) {
          flags_tx = 2;  // HAS_TIME & END_BURST, fixme
        }
        const int tx_status = radio_.RadioTx(
            ue_id, tx_data.data(), samples_per_symbol, flags_tx, tx_time);
        if (tx_status < static_cast<int>(samples_per_symbol)) {
          std::cout << "BAD Write (UL): For Ue " << ue_id << " " << tx_status
                    << "/" << samples_per_symbol << std::endl;
        }

        if (kDebugPrintInTask) {
          std::printf(
              "TxRxWorkerClientHw::DoTx[%zu]: Transmitted frame %zu, data "
              "symbol %zu, radio %zu, tag %zu\n",
              tid_, frame_id, tx_symbol_id, ue_id,
              gen_tag_t(current_event.tags_[0]).tag_);
        }
      }
      //Frame transmit complete
      auto complete_event =
          EventData(EventType::kPacketTX, current_event.tags_[0]);
      NotifyComplete(complete_event);
    }
  }  // End all events
  return tx_events.size();
}

ssize_t TxRxWorkerClientHw::SyncBeacon(size_t radio_id, size_t sample_window) {
  ssize_t sync_index = -1;
  long long rx_time = 0;
  assert(sample_window <=
         (config->SampsPerSymbol() * config->Frame().NumTotalSyms()));

  //\todo add a retry exit.
  while ((Configuration()->Running() == true) && (sync_index < 0)) {
    const int rx_status =
        radio_.RadioRx(radio_id, rx_frame_.data(), sample_window, rx_time);

    if (rx_status != static_cast<int>(sample_window))
      std::cerr << "RadioTxRx [" << radio_id << "]: BAD SYNC Receive("
                << rx_status << "/" << sample_window << ") at Time " << rx_time
                << std::endl;
    else {
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
  assert(sample_window <=
         (config->SampsPerSymbol() * config->Frame().NumTotalSyms()));

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

/*
if (sync_index >= 0) {
  rx_offset = sync_index - Configuration()->BeaconLen() -
              Configuration()->OfdmTxZeroPrefix();
  MLPD_INFO(
      "RadioTxRx [%zu]: Beacon detected at Time %lld, sync_index: "
      "%ld, rx sample offset: %d\n",
      radio_id, rx_time, sync_index, rx_offset);
}
*/

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