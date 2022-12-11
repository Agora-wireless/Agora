/**
 * @file radio_set_bs.cc
 * @brief Implementation file for the RadioSetBs class.
 */
#include "radio_set_bs.h"

#include <thread>

#include "SoapySDR/Formats.h"
#include "SoapySDR/Logger.hpp"
#include "logger.h"

static constexpr bool kPrintCalibrationMats = false;
static constexpr size_t kSoapyMakeMaxAttempts = 3;
static constexpr size_t kHubMissingWaitMs = 100;

RadioSetBs::RadioSetBs(Config* cfg, Radio::RadioType radio_type)
    : RadioSet(cfg->SampsPerSymbol()),
      cfg_(cfg),
      num_radios_initialized_(0),
      num_radios_configured_(0) {
  SoapySDR::Kwargs args;
  SoapySDR::Kwargs sargs;
  // load channels
  auto channels = Utils::StrToChannels(cfg_->Channel());

  radio_num_ = cfg_->NumRadios();
  antenna_num_ = cfg_->BsAntNum();
  std::cout << "BS Radio num is " << radio_num_
            << ", Antenna num: " << antenna_num_ << std::endl;
  if (kUseUHD == false) {
    for (size_t i = 0; i < cfg_->NumCells(); i++) {
      SoapySDR::Device* hub_device = nullptr;
      if (cfg_->HubId().at(i).empty() == false) {
        args["driver"] = "remote";
        args["timeout"] = "100000";
        args["serial"] = cfg_->HubId().at(i);
        args["remote:type"] = "faros";
        args["remote:driver"] = "faros";
        args["remote:mtu"] = "1500";
        args["remote:ipver"] = "6";
        //remote: prot is also an option

        for (size_t tries = 0; tries < kSoapyMakeMaxAttempts; tries++) {
          try {
            hub_device = SoapySDR::Device::make(args);
            break;
          } catch (const std::runtime_error& e) {
            const auto* message = e.what();
            AGORA_LOG_WARN("RadioSetBs::Soapy error[%zu] -- %s\n", tries,
                           message);
            std::this_thread::sleep_for(
                std::chrono::milliseconds(kHubMissingWaitMs));
          }
        }
        if (hub_device == nullptr) {
          AGORA_LOG_ERROR(
              "SoapySDR failed to locate the hub device %s in %zu tries\n",
              cfg_->HubId().at(i).c_str(), kSoapyMakeMaxAttempts);
          throw std::runtime_error("SoapySDR failed to locate the hub device");
        } else {
          for (const auto& info : hub_device->getSettingInfo()) {
            AGORA_LOG_TRACE("Hub[%s(%zu)] setting: %s\n",
                            cfg_->HubId().at(i).c_str(), i, info.key.c_str());
            (void)info;
          }
          hub_device->writeSetting("FEC_SNOOPER_CLEAR", "");
        }
      }
      hubs_.push_back(hub_device);
    }
  }

  for (size_t i = 0; i < radio_num_; i++) {
    radios_.emplace_back(Radio::Create(radio_type));
  }

  std::vector<std::thread> init_bs_threads;

  for (size_t i = 0; i < radio_num_; i++) {
#ifdef THREADED_INIT
    init_bs_threads.emplace_back(&RadioSetBs::InitRadio, this, i);
#else
    InitRadio(i);
#endif
  }

  // Block until all radios are initialized
  size_t num_checks = 0;
  size_t num_radios_init = num_radios_initialized_.load();
  while (num_radios_init != radio_num_) {
    num_checks++;
    if (num_checks > 1e9) {
      std::printf(
          "RadioSetBs: Waiting for radio initialization, %zu of %zu ready\n",
          num_radios_init, radio_num_);
      num_checks = 0;
    }
    num_radios_init = num_radios_initialized_.load();
  }

  for (auto& join_thread : init_bs_threads) {
    join_thread.join();
  }

  /** apply DC Offset and IQ Imbalance Params here **/

  std::vector<std::thread> config_bs_threads;
  for (size_t i = 0; i < radio_num_; i++) {
#ifdef THREADED_INIT
    config_bs_threads.emplace_back(&RadioSetBs::ConfigureRadio, this, i);
#else
    ConfigureRadio(i);
#endif
  }

  num_checks = 0;
  // Block until all radios are configured
  size_t num_radios_config = num_radios_configured_.load();
  while (num_radios_config != radio_num_) {
    num_checks++;
    if (num_checks > 1e9) {
      AGORA_LOG_WARN(
          "RadioSetBs: Waiting for radio initialization, %zu of %zu ready\n",
          num_radios_config, radio_num_);
      num_checks = 0;
    }
    num_radios_config = num_radios_configured_.load();
  }

  for (auto& join_thread : config_bs_threads) {
    join_thread.join();
  }

  for (const auto& radio : radios_) {
    radio->PrintSettings();
  }

  // TODO: For multi-cell, this procedure needs modification
  if (kUseUHD == false) {
    for (size_t i = 0; i < cfg_->NumCells(); i++) {
      if (hubs_.at(i) == nullptr) {
        radios_.at(i)->ClearSyncDelay();
      } else {
        hubs_.at(i)->writeSetting("SYNC_DELAYS", "");
      }
    }
  }
  AGORA_LOG_INFO("RadioSetBs init complete!\n");
}

void RadioSetBs::InitRadio(size_t radio_id) {
  radios_.at(radio_id)->Init(cfg_, radio_id, cfg_->RadioId().at(radio_id),
                             Utils::StrToChannels(cfg_->Channel()),
                             cfg_->HwFramer());
  num_radios_initialized_.fetch_add(1);
}

RadioSetBs::~RadioSetBs() {
  for (auto* hub : hubs_) {
    SoapySDR::Device::unmake(hub);
  }
  hubs_.clear();
  AGORA_LOG_INFO("RadioStop destructed\n");
}

void RadioSetBs::ConfigureRadio(size_t radio_id) {
  std::vector<double> tx_gains;
  tx_gains.emplace_back(cfg_->TxGainA());
  tx_gains.emplace_back(cfg_->TxGainB());

  std::vector<double> rx_gains;
  rx_gains.emplace_back(cfg_->RxGainA());
  rx_gains.emplace_back(cfg_->RxGainB());

  radios_.at(radio_id)->Setup(tx_gains, rx_gains);
  num_radios_configured_.fetch_add(1);
}

bool RadioSetBs::RadioStart() {
  for (size_t i = 0; i < radio_num_; i++) {
    if (cfg_->HwFramer()) {
      const size_t cell_id = cfg_->CellId().at(i);
      const bool is_ref_radio = (i == cfg_->RefRadio(cell_id));
      radios_.at(i)->ConfigureTddModeBs(is_ref_radio);
    }
    radios_.at(i)->SetTimeAtTrigger(0);
  }
  RadioSet::RadioStart(Radio::kActivate);
  return true;
}

void RadioSetBs::Go() {
  // TODO: For multi-cell trigger process needs modification
  if (kUseUHD == false) {
    for (size_t i = 0; i < cfg_->NumCells(); i++) {
      if (hubs_.at(i) == nullptr) {
        radios_.at(i)->Trigger();
      } else {
        hubs_.at(i)->writeSetting("TRIGGER_GEN", "");
      }
    }
  }
}

void RadioSetBs::AdjustDelays(const std::vector<int>& ch0_offsets) {
  // adjust all trigger delay fwith respect to the max offset
  const size_t ref_offset =
      *std::max_element(ch0_offsets.begin(), ch0_offsets.end());
  for (size_t i = 0; i < ch0_offsets.size(); i++) {
    const int delta = ref_offset - ch0_offsets.at(i);
    AGORA_LOG_INFO("Sample adjusting delay of node %zu (offset %d) by %d\n", i,
                   ch0_offsets.at(i), delta);
    const int iter = delta < 0 ? -delta : delta;
    for (int j = 0; j < iter; j++) {
      if (delta < 0) {
        radios_.at(i)->AdjustDelay("-1");
      } else {
        radios_.at(i)->AdjustDelay("1");
      }
    }
  }
}

long long RadioSetBs::SyncArrayTime() {
  //1ms
  constexpr long long kTimeSyncMaxLimit = 1000000;
  //Use the trigger to sync the array
  AGORA_LOG_TRACE("SyncArrayTime: Setting trigger time\n");
  for (auto& radio : radios_) {
    radio->SetTimeAtTrigger();
  }

  AGORA_LOG_TRACE("SyncArrayTime: Triggering!\n");
  Go();

  ///Wait for enough time for the boards to update
  auto wait_time = std::chrono::milliseconds(500);
  AGORA_LOG_TRACE("SyncArrayTime: Waiting for %ld ms\n", wait_time.count());
  std::this_thread::sleep_for(wait_time);
  AGORA_LOG_TRACE("SyncArrayTime: Time Check!\n");

  //Get first time
  auto radio_time = radios_.at(0)->GetTimeNs();
  //Verify all times are within 1ms (typical .35ms - .85 between calls)
  for (size_t i = 1; i < radios_.size(); i++) {
    auto time_now = radios_.at(i)->GetTimeNs();
    auto time_diff_ns = time_now - radio_time;
    if (time_diff_ns > kTimeSyncMaxLimit) {
      AGORA_LOG_WARN(
          "SyncArrayTime: Radio time out of bounds during alignment.  Radio "
          "%zu, time %lld, reference %lld, difference %lld, tolerance %lld\n",
          i, time_now, radio_time, time_diff_ns, kTimeSyncMaxLimit);
    }
    //Update the check time with the current time
    radio_time = time_now;
  }
  return radio_time;
}
