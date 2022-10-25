/**
 * @file txrx_worker_hw.cc
 * @brief Implementation of PacketTxRx datapath functions for communicating
 * with real iris / faros hardware
 */

#include "txrx_worker_client_hw.h"

#include <cassert>
#include <complex>

#include "comms-lib.h"
#include "datatype_conversion.h"
#include "gettime.h"
#include "logger.h"
#include "message.h"
#include "utils.h"

static constexpr bool kDebugBeaconChannels = false;
static constexpr size_t kSyncDetectChannel = 0;
static constexpr bool kVerifyFirstSync = true;
static constexpr size_t kReSyncRetryCount = 100u;
static constexpr float kBeaconDetectWindow = 2.33f;
static constexpr size_t kBeaconsToStart = 2;
static constexpr bool kPrintClientBeaconSNR = true;
static constexpr ssize_t kMaxBeaconAdjust = 5;

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
      //kOffsetOfData to allocate space for the Packet * header.
      frame_storage_(
          config->NumUeChannels(),
          std::vector<std::complex<int16_t>>(
              Packet::kOffsetOfData +
                  (config->SampsPerSymbol() * config->Frame().NumTotalSyms()),
              std::complex<int16_t>(0, 0))),
      rx_frame_pkts_(config->NumUeChannels()),
      rx_pkts_ptrs_(config->NumUeChannels()) {
  for (size_t ch = 0; ch < config->NumUeChannels(); ch++) {
    auto* pkt_memory = reinterpret_cast<Packet*>(frame_storage_.at(ch).data());
    auto& scratch_rx_memory = rx_frame_pkts_.at(ch);
    scratch_rx_memory.Set(pkt_memory);
    rx_pkts_ptrs_.at(ch) = &scratch_rx_memory;

    AGORA_LOG_TRACE(
        "TxRxWorkerClientHw - rx pkt memory %ld:%ld data location %ld\n",
        reinterpret_cast<intptr_t>(pkt_memory),
        reinterpret_cast<intptr_t>(scratch_rx_memory.RawPacket()),
        reinterpret_cast<intptr_t>(scratch_rx_memory.RawPacket()->data_));
  }
  //throw std::runtime_error("Rx Prt locations");
  RtAssert(interface_count == 1,
           "Interface count must be set to 1 for use with this class");

  RtAssert(config->UeHwFramer() == false, "Must have ue hw framer disabled");
  InitRxStatus();
}

TxRxWorkerClientHw::~TxRxWorkerClientHw() = default;

//Main Thread Execution loop
void TxRxWorkerClientHw::DoTxRx() {
  PinToCoreWithOffset(ThreadType::kWorkerTXRX, core_offset_, tid_);

  const size_t max_cfo = 200;  // in ppb, For Iris
  // If JSON input if not default (0),
  // Else calculate based of ppb and frame length
  const size_t frame_sync_period =
      static_cast<int>(Configuration()->UeResyncPeriod()) > 0
          ? static_cast<unsigned long>(Configuration()->UeResyncPeriod())
          : static_cast<size_t>(
                std::floor(1e9 / (max_cfo * Configuration()->SampsPerFrame())));

  AGORA_LOG_INFO("TxRxWorkerClientHw[%zu] has %zu:%zu total radios %zu\n", tid_,
                 interface_offset_, (interface_offset_ + num_interfaces_) - 1,
                 num_interfaces_);

  running_ = true;

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

  size_t local_interface = 0;
  WaitDetectBeacon(local_interface);
  //Beacon has been detected...
  long long time0 = EstablishTime0(local_interface);
  ///\todo make sure we are "real time"
  AGORA_LOG_INFO("DoTxRx[%zu]: radio %zu established time0 %lld\n", tid_,
                 local_interface + interface_offset_, time0);

  size_t rx_frame_id = 0;
  size_t rx_symbol_id = 0;
  long long rx_time = 0;
  ssize_t rx_adjust_samples = 0;

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

        if (rx_symbol_id == Configuration()->Frame().GetBeaconSymbolLast()) {
          ssize_t adjustment;
          const bool success = ResyncOnBeacon(rx_frame_id, frame_sync_period,
                                              rx_pkts, adjustment);
          if (success) {
            rx_adjust_samples = adjustment;
            time0 = time0 + adjustment;
          }
        }
      } else if (rx_pkts.empty() == false) {
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
  auto& rx_info = rx_status_.at(interface_id);

  size_t num_rx_samps;
  //Sample offset alignment
  if (sample_offset <= 0) {
    //Don't read an entire symbol due to the offset ( + a negative number )
    num_rx_samps = Configuration()->SampsPerSymbol() + sample_offset;
  } else {
    //Otherwise throw out the offset (could just add this to the next symbol but our buffers are not large enough)
    num_rx_samps = sample_offset;
  }

  //Check for completion
  if (rx_info.SamplesAvailable() >= num_rx_samps) {
    sample_offset = 0;
    AGORA_LOG_INFO(
        "DoRx - Samples Per Symbol %zu, available %zu, offset %ld, exiting\n",
        Configuration()->SampsPerSymbol(), rx_info.SamplesAvailable(),
        sample_offset);
    if (rx_info.SamplesAvailable() > num_rx_samps) {
      //Reset Sample
      throw std::runtime_error("Need to implement this!!!");
    } else {
      ResetRxStatus(interface_id, true);
    }
  } else {
    // Else ensures num_rx_samps > rx_info.SamplesAvailable()
    num_rx_samps = num_rx_samps - rx_info.SamplesAvailable();

    AGORA_LOG_TRACE(
        "DoRx[%zu] - Calling RadioRx[%zu], available %zu, offset %ld, "
        "requesting samples %zu:%zu\n",
        tid_, radio_id, rx_info.SamplesAvailable(), sample_offset, num_rx_samps,
        Configuration()->SampsPerSymbol());

    auto rx_locations = rx_info.GetRxPtrs();

    Radio::RxFlags out_flags;
    long long current_rx_time;
    const int rx_status = radio_.RadioRx(radio_id, rx_locations, num_rx_samps,
                                         out_flags, current_rx_time);

    if (rx_status < 0) {
      AGORA_LOG_ERROR(
          "TxRxWorkerClientHw[%zu]: Interface %zu | Radio %zu - Rx failure RX "
          "status = %d is less than 0\n",
          tid_, interface_id, interface_id + interface_offset_, rx_status);
    } else if (rx_status > 0) {
      const size_t new_samples = static_cast<size_t>(rx_status);
      rx_info.Update(new_samples, current_rx_time);
      if (new_samples < num_rx_samps) {
        //Didn't receive everything we requested, try again next time (status saved in rx_info)
        AGORA_LOG_TRACE(
            "TxRxWorkerClientHw[%zu]: Interface %zu | Radio %zu - Rx failure "
            "RX status = %d is less than num samples %zu\n",
            tid_, interface_id, interface_id + interface_offset_, rx_status,
            num_rx_samps);
      } else if (new_samples == num_rx_samps) {
        //sample_offset > 0 means we ignore the rx'd data (don't update the symbol / frame tracking)
        receive_time = rx_info.StartTime();
        bool ignore = true;
        if (sample_offset <= 0) {
          // Expected rx, Set the symbol / frame id's
          if (Configuration()->UeHwFramer()) {
            global_frame_id = static_cast<size_t>(receive_time >> 32);
            global_symbol_id =
                static_cast<size_t>((receive_time >> 16) & 0xFFFF);
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
                tid_, global_frame_id, global_symbol_id, radio_id,
                receive_time);
          }

          ignore = (IsRxSymbol(global_symbol_id) == false);
          if (ignore == false) {
            auto packets = rx_info.GetRxPackets();
            for (size_t ch = 0; ch < channels_per_interface_; ch++) {
              auto* rx_packet = packets.at(ch);
              auto* raw_pkt = rx_packet->RawPacket();
              new (raw_pkt) Packet(global_frame_id, global_symbol_id, 0,
                                   first_ant_id + ch);
              result_packets.push_back(raw_pkt);

              AGORA_LOG_FRAME(
                  "TxRxWorkerClientHw [%zu]: Rx Downlink (Frame %zu, Symbol "
                  "%zu, Ant %zu) from Radio %zu at time %lld\n",
                  tid_, global_frame_id, global_symbol_id, first_ant_id + ch,
                  radio_id, receive_time);

              // Push kPacketRX event into the queue.
              const EventData rx_message(EventType::kPacketRX,
                                         rx_tag_t(*rx_packet).tag_);
              NotifyComplete(rx_message);
            }
          }  // end is RxSymbol
        }    // sample offset <= 0
        sample_offset = 0;
        ResetRxStatus(interface_id, ignore);
      } else {
        AGORA_LOG_ERROR(
            "TxRxWorkerClientHw[%zu]: Interface %zu | Radio %zu - Rx failure "
            "new samples %zu requested samples %zu samples rx'd exceed "
            "request\n",
            tid_, interface_id, interface_id + interface_offset_, new_samples,
            num_rx_samps);
      }
    }  // rx_status > 0
  }
  return result_packets;
}

//Tx data
size_t TxRxWorkerClientHw::DoTx(const long long time0) {
  const auto tx_events = GetPendingTxEvents();

  for (const EventData& current_event : tx_events) {
    RtAssert((current_event.event_type_ == EventType::kPacketTX) ||
                 (current_event.event_type_ == EventType::kPacketPilotTX),
             "Wrong Event Type in TX Queue!");

    //Assuming the 1 message per radio per frame
    const size_t frame_id = gen_tag_t(current_event.tags_[0u]).frame_id_;
    const size_t ue_ant = gen_tag_t(current_event.tags_[0u]).ue_id_;
    const size_t radio_id = ue_ant / channels_per_interface_;
    //[0...channels-1]
    const size_t ant_offset = ue_ant % channels_per_interface_;

    AGORA_LOG_FRAME(
        "TxRxWorkerClientHw::DoTx[%zu]: Request to Transmit (Frame %zu, "
        "User %zu, Ant %zu) time0 %lld\n",
        tid_, frame_id, radio_id, ue_ant, time0);

    RtAssert((radio_id >= interface_offset_) &&
                 (radio_id <= (num_interfaces_ + interface_offset_)),
             "Invalid Tx interface Id");
    RtAssert(radio_id == tid_,
             "TxRxWorkerClientHw::DoTx - Ue id was not the expected values");

    //For Tx we need all channels_per_interface_ antennas before we can transmit
    //we will assume that if you get the last antenna, you have already received all
    //other antennas (enforced in the passing utility)
    if ((ant_offset + 1) == channels_per_interface_) {
      // Transmit pilot(s)
      if (Configuration()->UeHwFramer() == false) {
        for (size_t ch = 0; ch < channels_per_interface_; ch++) {
          const size_t pilot_ant = (radio_id * channels_per_interface_) + ch;
          //Each pilot will be in a different tx slot (called for each pilot)
          TxPilot(pilot_ant, frame_id, time0);

          //Pilot transmit complete for pilot ue
          if (current_event.event_type_ == EventType::kPacketPilotTX) {
            const auto complete_event =
                EventData(EventType::kPacketPilotTX,
                          gen_tag_t::FrmSymUe(frame_id, 0, pilot_ant).tag_);
            NotifyComplete(complete_event);
          }
        }  //For each channel
      }

      if (current_event.event_type_ == EventType::kPacketTX) {
        // Transmit data for all symbols (each cannel transmits for each symbol)
        TxUplinkSymbols(radio_id, frame_id, time0);
        //Notify the tx is complete for all antennas on the interface
        for (size_t ch = 0; ch < channels_per_interface_; ch++) {
          const size_t tx_ant = (radio_id * channels_per_interface_) + ch;
          //Frame transmit complete
          const auto complete_event =
              EventData(EventType::kPacketTX,
                        gen_tag_t::FrmSymUe(frame_id, 0, tx_ant).tag_);
          NotifyComplete(complete_event);
        }
        AGORA_LOG_TRACE(
            "TxRxWorkerClientHw::DoTx[%zu]: Frame %zu Transmit Complete for Ue "
            "%zu\n",
            tid_, frame_id, radio_id);
      }
    }
  }  // End all events
  return tx_events.size();
}

///\todo for the multi radio case should let this return if not enough data is found
/// This function blocks untill all the discard_samples are received for a given local_interface
void TxRxWorkerClientHw::AdjustRx(size_t local_interface,
                                  size_t discard_samples) {
  const size_t radio_id = local_interface + interface_offset_;
  long long rx_time = 0;

  size_t request_samples = discard_samples;
  TxRxWorkerRx::RxStatusTracker rx_tracker(channels_per_interface_);
  rx_tracker.Reset(rx_pkts_ptrs_);

  while (Configuration()->Running() && (request_samples > 0)) {
    auto rx_locations = rx_tracker.GetRxPtrs();
    Radio::RxFlags out_flags;
    const int rx_status = radio_.RadioRx(radio_id, rx_locations,
                                         request_samples, out_flags, rx_time);

    if (rx_status < 0) {
      AGORA_LOG_ERROR("AdjustRx [%zu]: BAD SYNC Received (%d/%zu) %lld\n", tid_,
                      rx_status, request_samples, rx_time);
    } else {
      size_t new_samples = static_cast<size_t>(rx_status);
      rx_tracker.Update(new_samples, rx_time);
      if (new_samples <= request_samples) {
        request_samples -= new_samples;
      } else {
        AGORA_LOG_ERROR(
            "SycBeacon [%zu]: BAD SYNC Rx more samples then requested "
            "(%zu/%zu) %lld\n",
            tid_, new_samples, request_samples, rx_time);
      }
    }
  }  // request_samples > 0
}

///\todo for the multi radio case should let this return if not enough data is found
ssize_t TxRxWorkerClientHw::SyncBeacon(size_t local_interface,
                                       size_t sample_window) {
  const size_t radio_id = local_interface + interface_offset_;
  ssize_t sync_index = -1;
  long long rx_time = 0;
  assert(sample_window <= (Configuration()->SampsPerSymbol() *
                           Configuration()->Frame().NumTotalSyms()));

  size_t request_samples = sample_window;
  TxRxWorkerRx::RxStatusTracker rx_tracker(channels_per_interface_);
  rx_tracker.Reset(rx_pkts_ptrs_);

  while (Configuration()->Running() && (sync_index < 0)) {
    auto rx_locations = rx_tracker.GetRxPtrs();
    Radio::RxFlags out_flags;
    const int rx_status = radio_.RadioRx(radio_id, rx_locations,
                                         request_samples, out_flags, rx_time);

    if (rx_status < 0) {
      AGORA_LOG_ERROR("SyncBeacon [%zu]: BAD SYNC Received (%d/%zu) %lld\n",
                      tid_, rx_status, sample_window, rx_time);
    } else if (rx_status > 0) {
      const size_t new_samples = static_cast<size_t>(rx_status);
      const bool is_cont = rx_tracker.CheckContinuity(rx_time);
      if (is_cont == false) {
        AGORA_LOG_WARN(
            "SyncBeacon - Received new non-contiguous samples %zu, ignoring "
            "%zu, %zu \n",
            new_samples, rx_tracker.SamplesAvailable(), sample_window);
        //Samples do not align, throw out all old + new samples.
        rx_tracker.DiscardOld(new_samples, rx_time);
      } else {
        rx_tracker.Update(new_samples, rx_time);
        if (new_samples == request_samples) {
          AGORA_LOG_TRACE(
              "SyncBeacon - Samples %zu:%zu, Window %zu - Check Beacon %ld\n",
              new_samples, rx_tracker.SamplesAvailable(), sample_window,
              reinterpret_cast<intptr_t>(
                  rx_pkts_ptrs_.at(kSyncDetectChannel)->RawPacket()->data_));

          sync_index = FindSyncBeacon(
              reinterpret_cast<std::complex<int16_t>*>(
                  rx_pkts_ptrs_.at(kSyncDetectChannel)->RawPacket()->data_),
              sample_window);
          //Throw out samples until we detect the beacon
          request_samples = sample_window;
          rx_tracker.Reset(rx_pkts_ptrs_);
        } else if (new_samples < request_samples) {
          AGORA_LOG_TRACE("SyncBeacon - Samples %zu:%zu, Window %zu\n",
                          new_samples, rx_tracker.SamplesAvailable(),
                          sample_window);
          request_samples -= new_samples;
        } else {
          AGORA_LOG_ERROR(
              "SycBeacon [%zu]: BAD SYNC Rx more samples then requested "
              "(%zu/%zu) %lld\n",
              tid_, new_samples, request_samples, rx_time);
        }
      }  // is continuous
    }
  }  // end while sync_index < 0
  return sync_index;
}

ssize_t TxRxWorkerClientHw::FindSyncBeacon(
    const std::complex<int16_t>* check_data, size_t sample_window,
    float corr_scale) {
  ssize_t sync_index = -1;
  assert(sample_window <= (Configuration()->SampsPerSymbol() *
                           Configuration()->Frame().NumTotalSyms()));

  sync_index = CommsLib::FindBeaconAvx(check_data, Configuration()->GoldCf32(),
                                       sample_window, corr_scale);

  if (kPrintClientBeaconSNR && (sync_index >= 0) &&
      ((sync_index + Configuration()->BeaconLen()) < sample_window)) {
    ///\todo Remove this float conversion to speed up function
    float sig_power = 0;
    float noise_power = 0;
    for (size_t i = 0; i < Configuration()->BeaconLen(); i++) {
      const size_t power_idx = sync_index - i;
      const size_t noise_idx = sync_index + i + 1;
      std::complex<float> power_value;
      std::complex<float> noise_value;
      ConvertShortToFloat(
          reinterpret_cast<const short*>(&check_data[power_idx]),
          reinterpret_cast<float*>(&power_value), 2);

      ConvertShortToFloat(
          reinterpret_cast<const short*>(&check_data[noise_idx]),
          reinterpret_cast<float*>(&noise_value), 2);

      sig_power += std::pow(std::abs(power_value), 2);
      noise_power += std::pow(std::abs(noise_value), 2);
    }
    AGORA_LOG_INFO("TxRxWorkerClientHw: Sync Beacon - SNR %2.1f dB\n",
                   +10 * std::log10(sig_power / noise_power));
  }
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

  std::vector<void*> tx_data(channels_per_interface_);
  for (size_t ul_symbol_idx = 0;
       ul_symbol_idx < Configuration()->Frame().NumULSyms(); ul_symbol_idx++) {
    const size_t tx_symbol_id =
        Configuration()->Frame().GetULSymbol(ul_symbol_idx);

    for (size_t ch = 0; ch < channels_per_interface_; ch++) {
      const size_t tx_ant = (radio_id * channels_per_interface_) + ch;
      auto* pkt = GetUlTxPacket(frame_id, tx_symbol_id, tx_ant);
      tx_data.at(ch) = reinterpret_cast<void*>(pkt->data_);

      if (kDebugTxData) {
        auto* data_truth =
            &Configuration()
                 ->UlIqT()[ul_symbol_idx]
                          [tx_ant * Configuration()->SampsPerSymbol()];
        auto* data_pkt = reinterpret_cast<std::complex<int16_t>*>(pkt->data_);
        if (memcmp(data_truth, data_pkt, Configuration()->PacketLength()) ==
            0) {
          AGORA_LOG_INFO(
              "TxRxWorkerClientHw: (Frame %zu Symbol %zu Ant %zu) TX data "
              "matched UlIqT all %zu samples\n",
              frame_id, tx_symbol_id, tx_ant,
              Configuration()->SampsPerSymbol());
        } else {
          size_t samps_mismatch = 0;
          for (size_t i = 0; i < Configuration()->SampsPerSymbol(); i++) {
            if (data_truth[i] != data_pkt[i]) {
              samps_mismatch++;
            }
          }
          AGORA_LOG_INFO(
              "TxRxWorkerClientHw: (Frame %zu Symbol %zu Ant %zu) TX data "
              "mismatched UlIqT %zu of %zu samples\n",
              frame_id, tx_symbol_id, tx_ant, samps_mismatch,
              Configuration()->SampsPerSymbol());
        }
      }
    }

    if (Configuration()->UeHwFramer()) {
      tx_time = ((long long)tx_frame_id << 32) | (tx_symbol_id << 16);
    } else {
      tx_time = time0 + (tx_frame_id * samples_per_frame) +
                (tx_symbol_id * samples_per_symbol) -
                Configuration()->ClTxAdvance().at(radio_id);
    }

    Radio::TxFlags flags_tx = GetTxFlags(radio_id, tx_symbol_id);
    const int tx_status = radio_.RadioTx(radio_id, tx_data.data(),
                                         samples_per_symbol, flags_tx, tx_time);
    if (tx_status < static_cast<int>(samples_per_symbol)) {
      AGORA_LOG_ERROR(
          "TxRxWorkerClientHw[%zu]: RadioTx status failed with code: %d For Ue "
          "radio %zu for %zu bytes and flags %d\n",
          tid_, tx_status, radio_id, samples_per_symbol,
          static_cast<int>(flags_tx));
    }
    if (kDebugPrintInTask) {
      AGORA_LOG_INFO(
          "TxRxWorkerClientHw::DoTx[%zu]: Transmitted Symbol (Frame "
          "%zu:%zu, Symbol %zu, Ue %zu) at time %lld flags %d\n",
          tid_, frame_id, tx_frame_id, tx_symbol_id, radio_id, tx_time,
          static_cast<int>(flags_tx));
    }
  }
}

void TxRxWorkerClientHw::TxPilot(size_t pilot_ant, size_t frame_id,
                                 long long time0) {
  const size_t tx_frame_id = frame_id + TX_FRAME_DELTA;
  //[0..channel-1]
  const size_t pilot_channel = (pilot_ant % channels_per_interface_);
  const size_t pilot_radio = pilot_ant / channels_per_interface_;
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

  if (Configuration()->UeHwFramer()) {
    tx_time = ((long long)tx_frame_id << 32) | (pilot_symbol_id << 16);
  } else {
    tx_time = time0 + (tx_frame_id * samples_per_frame) +
              (pilot_symbol_id * samples_per_symbol) -
              Configuration()->ClTxAdvance().at(pilot_radio);
  }

  Radio::TxFlags flags_tx = GetTxFlags(pilot_radio, pilot_symbol_id);
  const int tx_status = radio_.RadioTx(pilot_radio, tx_data.data(),
                                       samples_per_symbol, flags_tx, tx_time);

  if (tx_status < static_cast<int>(samples_per_symbol)) {
    AGORA_LOG_ERROR(
        "TxRxWorkerClientHw[%zu]: RadioTx (Pilot) status failed with code: %d "
        "For Ue radio %zu for %zu bytes and flags %d\n",
        tid_, tx_status, pilot_radio, samples_per_symbol,
        static_cast<int>(flags_tx));
  }

  if (kDebugPrintInTask) {
    AGORA_LOG_INFO(
        "TxRxWorkerClientHw::DoTx[%zu]: Transmitted Pilot  (Frame "
        "%zu:%zu, Symbol %zu, Ue %zu, Ant %zu:%zu) at time %lld flags "
        "%d\n",
        tid_, frame_id, tx_frame_id, pilot_symbol_id, pilot_radio,
        pilot_channel, pilot_ant, tx_time, static_cast<int>(flags_tx));
  }
}

void TxRxWorkerClientHw::InitRxStatus() {
  rx_status_.resize(num_interfaces_,
                    TxRxWorkerRx::RxStatusTracker(channels_per_interface_));
  std::vector<RxPacket*> rx_packets(channels_per_interface_);
  for (auto& status : rx_status_) {
    for (auto& new_packet : rx_packets) {
      new_packet = &GetRxPacket();
      AGORA_LOG_TRACE(
          "InitRxStatus[%zu]: Using Packet at location %ld, data location "
          "%ld\n",
          tid_, reinterpret_cast<intptr_t>(new_packet),
          reinterpret_cast<intptr_t>(new_packet->RawPacket()->data_));
    }
    //Allocate memory for each interface / channel
    status.Reset(rx_packets);
  }
}

void TxRxWorkerClientHw::ResetRxStatus(size_t interface, bool reuse_memory) {
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

//Checks to see if the current symbol is followed by another tx symbol
//Need the radio id, to check against the reference radio
bool TxRxWorkerClientHw::IsTxSymbolNext(size_t radio_id,
                                        size_t current_symbol) {
  bool tx_symbol_next = false;

  if (current_symbol != Configuration()->Frame().NumTotalSyms()) {
    const auto next_symbol = current_symbol + 1;
    const auto next_symbol_type = Configuration()->GetSymbolType(next_symbol);
    if (next_symbol_type == SymbolType::kUL) {
      tx_symbol_next = true;
    } else if (next_symbol_type == SymbolType::kPilot) {
      //Does the current radio_id transmit on the next symbol pilot
      const size_t pilot_idx =
          Configuration()->Frame().GetPilotSymbolIdx(next_symbol);
      size_t tx_pilot_radio = pilot_idx / channels_per_interface_;
      if (radio_id == tx_pilot_radio) {
        tx_symbol_next = true;
      }
    }
  }
  return tx_symbol_next;
}

Radio::TxFlags TxRxWorkerClientHw::GetTxFlags(size_t radio_id,
                                              size_t tx_symbol_id) {
  Radio::TxFlags tx_flags;
  const auto tx_again = IsTxSymbolNext(radio_id, tx_symbol_id);
  if (tx_again) {
    tx_flags = Radio::TxFlags::kTxFlagNone;
  } else {
    tx_flags = Radio::TxFlags::kEndTransmit;
  }
  return tx_flags;
}

void TxRxWorkerClientHw::WaitDetectBeacon(size_t local_interface) {
  const size_t samples_per_symbol = Configuration()->SampsPerSymbol();
  const size_t samples_per_frame =
      samples_per_symbol * Configuration()->Frame().NumTotalSyms();

  const size_t beacon_detect_window = Roundup<64>(static_cast<size_t>(
      static_cast<float>(samples_per_symbol) * kBeaconDetectWindow));
  const size_t alignment_samples = samples_per_frame - beacon_detect_window;
  RtAssert(beacon_detect_window < samples_per_frame,
           "Frame must be greater than the beacon detect window");

  AGORA_LOG_INFO("TxRxWorkerClientHw [%zu]: Waiting for beacon on radio %zu\n",
                 tid_, local_interface + interface_offset_);

  //Turns out detecting more than 1 beacon is helpful for the start
  size_t beacons_detected = 0;
  while ((beacons_detected < kBeaconsToStart) && Configuration()->Running()) {
    const ssize_t sync_index =
        SyncBeacon(local_interface, beacon_detect_window);
    if (sync_index >= 0) {
      const size_t rx_adjust_samples = sync_index -
                                       Configuration()->BeaconLen() -
                                       Configuration()->OfdmTxZeroPrefix();
      AGORA_LOG_INFO(
          "TxRxWorkerClientHw [%zu]: Beacon detected for radio %zu, "
          "sync_index: %ld, rx sample offset: %ld, window %zu, samples in "
          "frame %zu, alignment removal %zu\n",
          tid_, local_interface, sync_index, rx_adjust_samples,
          beacon_detect_window, samples_per_frame, alignment_samples);

      AdjustRx(local_interface, alignment_samples + rx_adjust_samples);
      beacons_detected++;
    } else if (Configuration()->Running()) {
      AGORA_LOG_WARN(
          "TxRxWorkerClientHw [%zu]: Beacon could not be detected on interface "
          "%zu - sync_index: %ld\n",
          tid_, local_interface, sync_index);
      throw std::runtime_error("rx sample offset is less than 0");
    }
  }
}

long long TxRxWorkerClientHw::EstablishTime0(size_t local_interface) {
  const size_t samples_per_symbol = Configuration()->SampsPerSymbol();
  long long time0(0);
  long long rx_time(0);
  ssize_t adjust_samples = 0;
  //Set initial frame and symbol to max value so we start at 0
  size_t frame_id = SIZE_MAX;
  size_t symbol_id = Configuration()->Frame().NumTotalSyms() - 1;
  //Establish time0 from symbol = 0 (beacon), frame 0
  while (Configuration()->Running() && (time0 == 0)) {
    const auto rx_pkts =
        DoRx(local_interface, frame_id, symbol_id, rx_time, adjust_samples);
    if (rx_pkts.size() == channels_per_interface_) {
      time0 = rx_time;

      if (kVerifyFirstSync) {
        for (size_t ch = 0; ch < channels_per_interface_; ch++) {
          const ssize_t sync_index = FindSyncBeacon(
              reinterpret_cast<std::complex<int16_t>*>(rx_pkts.at(ch)->data_),
              samples_per_symbol, Configuration()->ClCorrScale().at(tid_));
          if (sync_index >= 0) {
            adjust_samples = sync_index - Configuration()->BeaconLen() -
                             Configuration()->OfdmTxZeroPrefix();
            AGORA_LOG_INFO(
                "TxRxWorkerClientHw [%zu]: Initial Sync - ant %zu, frame "
                "%zu, symbol %zu sync_index: %ld, rx sample offset: %ld time0 "
                "%lld\n",
                tid_, (local_interface + interface_offset_) + ch, frame_id,
                symbol_id, sync_index, adjust_samples, time0);
          } else {
            throw std::runtime_error(
                "No Beacon Detected at Frame 0 / Symbol 0");
          }
        }
      }  // end verify first sync
    }    // received Frame 0 Symbol 0
  }      // end - establish time0 for a given interface}
  return time0;
}

//Returns True if the beacon is detected
bool TxRxWorkerClientHw::DoResync(const std::vector<Packet*>& check_pkts,
                                  ssize_t& adjust_samples) {
  bool beacon_detected = false;
  const size_t samples_per_symbol = Configuration()->SampsPerSymbol();
  //This is adding a race condition on this data, it is ok for now but we should fix this
  const ssize_t sync_index = FindSyncBeacon(
      reinterpret_cast<std::complex<int16_t>*>(
          check_pkts.at(kSyncDetectChannel)->data_),
      samples_per_symbol, Configuration()->ClCorrScale().at(tid_));
  if (sync_index >= 0) {
    beacon_detected = true;
    const ssize_t adjust = sync_index - Configuration()->BeaconLen() -
                           Configuration()->OfdmTxZeroPrefix();
    if (std::abs(adjust) > kMaxBeaconAdjust) {
      adjust_samples = 0;
      AGORA_LOG_WARN(
          "TxRxWorkerClientHw [%zu]: Re-syncing ignored due to excess "
          "offset %ld - channel %zu, sync_index: %ld\n",
          tid_, adjust, kSyncDetectChannel, sync_index);
    } else {
      adjust_samples = adjust;
      AGORA_LOG_INFO(
          "TxRxWorkerClientHw [%zu]: Re-syncing channel %zu, "
          "sync_index: %ld, rx sample offset: %ld\n",
          tid_, kSyncDetectChannel, sync_index, adjust_samples);
      //Display all the other channels
      if (kDebugBeaconChannels) {
        for (size_t ch = 0; ch < channels_per_interface_; ch++) {
          if (ch != kSyncDetectChannel) {
            const ssize_t aux_channel_sync = FindSyncBeacon(
                reinterpret_cast<std::complex<int16_t>*>(
                    check_pkts.at(ch)->data_),
                samples_per_symbol, Configuration()->ClCorrScale().at(tid_));
            AGORA_LOG_INFO(
                "TxRxWorkerClientHw [%zu]: beacon status channel %zu, "
                "sync_index: %ld, rx sample offset: %ld\n",
                tid_, ch, aux_channel_sync,
                aux_channel_sync - (Configuration()->BeaconLen() +
                                    Configuration()->OfdmTxZeroPrefix()));
          }
        }
      }
    }
  }  // sync_index >= 0
  return beacon_detected;
}

bool TxRxWorkerClientHw::ResyncOnBeacon(size_t frame_id,
                                        size_t frame_sync_period,
                                        const std::vector<Packet*>& beacon_pkts,
                                        ssize_t& adjust_samples) {
  bool resynced = false;

  // resync every frame_sync_period frames
  if ((attempt_resync_ == false) && ((frame_id / frame_sync_period) > 0) &&
      ((frame_id % frame_sync_period) == 0)) {
    attempt_resync_ = true;
  }

  if (attempt_resync_) {
    resynced = DoResync(beacon_pkts, adjust_samples);
    if (resynced) {
      attempt_resync_ = false;
      resync_success_cnt_++;
      resync_retry_cnt_ = 0;
    } else {
      resync_retry_cnt_++;
      if (resync_retry_cnt_ > kReSyncRetryCount) {
        AGORA_LOG_ERROR(
            "TxRxWorkerClientHw [%zu]: Exceeded resync retry limit (%zu) "
            "reached after %zu resync successes at frame: %zu. Stopping!\n",
            tid_, kReSyncRetryCount, resync_success_cnt_, frame_id);
        Configuration()->Running(false);
      }
    }
  }
  return resynced;
}