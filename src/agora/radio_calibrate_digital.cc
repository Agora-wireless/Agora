/**
 * @file radio_calibrate_digital.cc
 * @brief Implementation file for the digital (baseband) calibration 
 * functions such as sample offset and reciprocity calibration
 */
#include "matplotlibcpp.h"
#include "radio_lib.h"
namespace plt = matplotlibcpp;

static constexpr size_t kMaxArraySampleOffset = 10;
static constexpr bool kReciprocalCalibPlot = false;
static constexpr bool kVerboseCalibration = false;
static constexpr size_t kRxTimeoutUs = 1000000;
static constexpr size_t kTxTimeoutUs = 1000000;

std::vector<std::complex<float>> RadioConfig::SnoopSamples(
    SoapySDR::Device* dev, size_t channel, size_t read_size) {
  std::vector<uint32_t> samps_int =
      dev->readRegisters("RX_SNOOPER", channel, read_size);
  std::vector<std::complex<float>> samps =
      Utils::Uint32tocfloat(samps_int, "IQ");
  return samps;
}

auto RadioConfig::TxArrayToRef(
    const std::vector<std::complex<int16_t>>& tx_vec) {
  size_t ref = cfg_->RefRadio(0);
  long long tx_time(0);
  long long rx_time(0);
  int read_len = tx_vec.size();
  std::vector<std::vector<std::complex<int16_t>>> dl_buff(cfg_->BfAntNum());
  std::vector<std::complex<int16_t>> dummybuff(read_len);
  std::vector<std::complex<int16_t>> zeros(read_len,
                                           std::complex<int16_t>(0, 0));

  std::vector<const void*> txbuff(2);
  std::vector<void*> rxbuff(2);
  if (cfg_->NumChannels() == 2) {
    rxbuff.at(1) = dummybuff.data();
  }

  for (size_t ant_i = 0; ant_i < cfg_->BfAntNum(); ant_i++) {
    // set up tx/rx buffers
    size_t radio_i = ant_i / cfg_->NumChannels();
    dl_buff.at(ant_i).resize(read_len);
    rxbuff.at(0) = dl_buff.at(ant_i).data();  // ref always txrx on channel 0
    txbuff[ant_i % cfg_->NumChannels()] = tx_vec.data();
    if (cfg_->NumChannels() == 2) {
      txbuff[1 - (ant_i % 2)] = zeros.data();
    }

    // Send a separate pilot from each antenna
    int tx_flags = SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST;
    int ret = ba_stn_.at(radio_i)->writeStream(this->tx_streams_.at(radio_i),
                                               txbuff.data(), read_len,
                                               tx_flags, tx_time, kTxTimeoutUs);
    if (ret < (int)read_len) {
      std::cout << "bad write\n";
    }

    int rx_flags_activate = SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST;
    ret = ba_stn_.at(ref)->activateStream(this->rx_streams_.at(ref),
                                          rx_flags_activate, rx_time, read_len);

    Go();  // trigger

    int rx_flags = 0;
    ret =
        ba_stn_.at(ref)->readStream(this->rx_streams_.at(ref), rxbuff.data(),
                                    read_len, rx_flags, rx_time, kRxTimeoutUs);
    if (ret < (int)read_len) {
      std::cout << "bad read (" << ret << ") at node " << ref
                << " from antenna " << ant_i << std::endl;
    }
  }
  return dl_buff;
}

auto RadioConfig::TxRefToArray(
    const std::vector<std::complex<int16_t>>& tx_vec) {
  size_t num_radios = cfg_->NumRadios() - 1;  // minus ref. node
  size_t ref = cfg_->RefRadio(0);
  long long tx_time(0);
  long long rx_time(0);

  // Transmitting from only one chain, create a null vector for chainB
  size_t read_len = tx_vec.size();

  std::vector<const void*> txbuff(2);
  txbuff.at(0) = tx_vec.data();
  if (cfg_->NumChannels() == 2) {
    std::vector<std::complex<int16_t>> zeros(read_len,
                                             std::complex<int16_t>(0, 0));
    txbuff.at(1) = zeros.data();
  }
  std::vector<void*> rxbuff(2);

  // Allocate buffers for uplink directions
  std::vector<std::vector<std::complex<int16_t>>> ul_buff(cfg_->BfAntNum());
  std::vector<std::complex<int16_t>> dummybuff(read_len);
  int tx_flags = SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST;
  int ret =
      ba_stn_.at(ref)->writeStream(this->tx_streams_.at(ref), txbuff.data(),
                                   read_len, tx_flags, tx_time, kTxTimeoutUs);
  if (ret < (int)read_len) {
    std::cout << "bad write\n";
  }

  for (size_t i = 0; i < num_radios; i++) {
    int rx_flags = SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST;
    ret = ba_stn_.at(i)->activateStream(this->rx_streams_.at(i), rx_flags,
                                        rx_time, read_len);
  }

  Go();  // Trigger

  int rx_flags = SOAPY_SDR_END_BURST;
  for (size_t radio_i = 0; radio_i < num_radios; radio_i++) {
    size_t ant_i = radio_i * cfg_->NumChannels();
    ul_buff.at(ant_i).resize(read_len);
    rxbuff.at(0) = ul_buff.at(ant_i).data();
    if (cfg_->NumChannels() == 2) {
      ul_buff.at(ant_i + 1).resize(read_len);
      rxbuff.at(1) = ul_buff.at(ant_i + 1).data();
    }
    ret = ba_stn_.at(radio_i)->readStream(this->rx_streams_.at(radio_i),
                                          rxbuff.data(), read_len, rx_flags,
                                          rx_time, kRxTimeoutUs);
    if (ret < (int)read_len) {
      std::cout << "Bad read (" << ret << ") at node " << radio_i
                << " from node " << ref << std::endl;
    }
  }
  return ul_buff;
}

bool RadioConfig::FindTimeOffset(
    const std::vector<std::vector<std::complex<int16_t>>>& rx_mat,
    std::vector<int>& offset) {
  bool bad_data = false;
  size_t seq_len = cfg_->PilotCf32().size();
  RtAssert(rx_mat.size() == offset.size(),
           "rx_mat size does not match offset vector size!\n");
  for (size_t i = 0; i < rx_mat.size(); i++) {
    auto samps = Utils::Cint16ToCfloat32(rx_mat.at(i));
    size_t peak = CommsLib::FindPilotSeq(samps, cfg_->PilotCf32(), seq_len);
    offset[i] = peak < seq_len ? 0 : peak - seq_len;
    if (offset.at(i) == 0) {
      std::cout << "Invalid uplink pilot offsets" << std::endl;
      bad_data = true;
      break;
    }
    if (i >= cfg_->NumChannels() &&
        std::abs((int)offset.at(i) - (int)offset.at(i - cfg_->NumChannels())) >
            static_cast<int>(kMaxArraySampleOffset))

    {  // make sure offsets are not too
       // different from each other
      std::cout << "Difference in uplink pilot offsets exceeds threshold ("
                << kMaxArraySampleOffset << ")." << std::endl;
      bad_data = true;
      break;
    }
  }
  return bad_data;
}

void RadioConfig::AdjustDelays(std::vector<int> offset) {
  // adjust all trigger delay for all radios
  // with respect to the first non-ref radio
  size_t ref_offset = *std::max_element(offset.begin(), offset.end());
  for (size_t i = 0; i < offset.size(); i++) {
    // int delta = cfg_->OfdmTxZeroPrefix() - offset[i];
    int delta = ref_offset - offset[i];
    std::cout << "sample_adjusting delay of node " << i << " (offset "
              << offset[i] << ") by " << delta << std::endl;
    int iter = delta < 0 ? -delta : delta;
    for (int j = 0; j < iter; j++) {
      if (delta < 0) {
        ba_stn_[i]->writeSetting("ADJUST_DELAYS", "-1");
      } else {
        ba_stn_[i]->writeSetting("ADJUST_DELAYS", "1");
      }
    }
  }
}

void RadioConfig::CalibrateSampleOffset() {
  size_t num_radios = cfg_->BfAntNum() / cfg_->NumChannels();

  size_t n = 0;
  const size_t max_retries = 10;
  std::cout << "Calibrating with uplink " << std::endl;
  // Transmit from Ref to Array and Adjust Delays Until Synced
  while (n < max_retries) {
    auto ul_buff = TxRefToArray(cfg_->PilotCi16());
    std::vector<int> ul_offset(cfg_->BfAntNum(), 0);
    bool bad_data = this->FindTimeOffset(ul_buff, ul_offset);
    if (bad_data) {
      n++;
      continue;
    }
    std::vector<int> ul_offset_a;
    std::vector<int> ul_offset_b;
    int max_ab_diff = 0;
    int min_ab_diff = INT_MAX;
    if (cfg_->NumChannels() > 1) {
      for (size_t i = 0; i < num_radios; i += cfg_->NumChannels()) {
        ul_offset_a.push_back(ul_offset.at(i));
        ul_offset_b.push_back(ul_offset.at(i + 1));
        int ul_offset_diff = std::abs(ul_offset_b.back() - ul_offset_a.back());
        if (ul_offset_diff > max_ab_diff) max_ab_diff = ul_offset_diff;
        if (ul_offset_diff < min_ab_diff) min_ab_diff = ul_offset_diff;
      }
      std::cout << "Rx max chan A & B offset diff: " << max_ab_diff
                << ", Rx min chan A & B offset diff: " << min_ab_diff
                << std::endl;
    } else {
      ul_offset_a = ul_offset;
    }
    int ul_max_offset =
        *std::max_element(ul_offset_a.begin(), ul_offset_a.end());
    int ul_min_offset =
        *std::min_element(ul_offset_a.begin(), ul_offset_a.end());
    std::cout << "Max ul_offset: " << ul_max_offset
              << ", Min ul_offset: " << ul_min_offset << std::endl;
    if (ul_max_offset - ul_min_offset > 0) {
      std::cout
          << "Uplink pilot offsets not synced. Adjusting trigger offset..."
          << std::endl;
      AdjustDelays(ul_offset_a);
    } else {
      // measure uplink SNR here
      std::vector<float> snr;
      std::cout << "*************************************************"
                << std::endl;
      std::cout << "Received SNR from the Reference Node At the Array"
                << std::endl;
      size_t pilot_start = ul_min_offset + cfg_->CpLen();
      size_t pilot_stop = ul_min_offset + cfg_->CpLen() + cfg_->OfdmCaNum();
      RtAssert(pilot_stop < cfg_->SampsPerSymbol(),
               "Pilot samples go beyond received symbol boundary."
               " Consider extending ofdm_tx_zero_postfix parameter!");
      for (size_t i = 0; i < ul_buff.size(); i++) {
        std::vector<std::complex<int16_t>> ofdm_samps(
            ul_buff.at(i).begin() + pilot_start,
            ul_buff.at(i).begin() + pilot_stop);
        auto ofdm_data = Utils::Cint16ToCfloat32(ofdm_samps);
        float snr_val = CommsLib::ComputeOfdmSnr(
            ofdm_data, cfg_->OfdmDataStart(), cfg_->OfdmDataStop());
        snr.push_back(snr_val);
        std::cout << snr_val << " ";
      }
      std::cout << std::endl;
      auto ul_max_snr_it = std::max_element(snr.begin(), snr.end());
      auto ul_min_snr_it = std::min_element(snr.begin(), snr.end());
      std::cout << "Min UL SNR at antenna " << ul_min_snr_it - snr.begin()
                << ": " << *ul_min_snr_it << std::endl;
      std::cout << "Max UL SNR at antenna " << ul_max_snr_it - snr.begin()
                << ": " << *ul_max_snr_it << std::endl;
      std::cout << "*************************************************"
                << std::endl;
      cfg_->OfdmRxZeroPrefixCalUl(ul_min_offset - (ul_min_offset % 4));
      break;
    }
    n++;
  }

  if (n >= max_retries) {
    std::cout << "Reached max retries for sample offset calibration (uplink)"
              << std::endl;
    exit(0);
  }

  // Transmit from Array to Ref and ensure they are all synced
  n = 0;
  std::cout << "Calibrating with downlink " << std::endl;
  while (n < max_retries) {
    auto dl_buff = TxArrayToRef(cfg_->PilotCi16());
    std::vector<int> dl_offset(cfg_->BfAntNum(), 0);
    bool bad_data = this->FindTimeOffset(dl_buff, dl_offset);
    if (bad_data) {
      n++;
      continue;
    }

    std::vector<int> dl_offset_a;
    std::vector<int> dl_offset_b;
    int max_ab_diff = 0;
    int min_ab_diff = INT_MAX;
    if (cfg_->NumChannels() > 1) {
      for (size_t i = 0; i < num_radios; i += cfg_->NumChannels()) {
        dl_offset_a.push_back(dl_offset.at(i));
        dl_offset_b.push_back(dl_offset.at(i + 1));
        int dl_offset_diff = std::abs(dl_offset_b.back() - dl_offset_a.back());
        if (dl_offset_diff > max_ab_diff) max_ab_diff = dl_offset_diff;
        if (dl_offset_diff < min_ab_diff) min_ab_diff = dl_offset_diff;
      }
      std::cout << "Tx max chan A & B offset diff: " << max_ab_diff
                << ", Tx min chan A & B offset diff: " << min_ab_diff
                << std::endl;
    } else {
      dl_offset_a = dl_offset;
    }

    int max_offset = *std::max_element(dl_offset_a.begin(), dl_offset_a.end());
    int min_offset = *std::min_element(dl_offset_a.begin(), dl_offset_a.end());
    std::cout << "Max dl_offset: " << max_offset
              << ", Min dl_offset: " << min_offset << std::endl;
    if (max_offset - min_offset > 4) {
      if (n + 1 < max_retries) {
        std::cout << "Downlink offsets mismatch: Try " << n + 1 << std::endl;
      } else {
        std::cout << "Downlink pilot offsets not synced!" << std::endl;
        exit(0);
      }
    } else {
      // measure downlink SNR here
      std::vector<float> snr;
      std::cout << "*************************************************"
                << std::endl;
      std::cout << "Received SNR from the Reference Node At the Array"
                << std::endl;
      size_t pilot_start = min_offset + cfg_->CpLen();
      size_t pilot_stop = min_offset + cfg_->CpLen() + cfg_->OfdmCaNum();
      RtAssert(pilot_stop < cfg_->SampsPerSymbol(),
               "Received pilot exceeds symbol boundary. Consider extending "
               "ofdm_tx_zero_postfix parameter!");
      for (size_t i = 0; i < dl_buff.size(); i++) {
        std::vector<std::complex<int16_t>> ofdm_samps(
            dl_buff.at(i).begin() + pilot_start,
            dl_buff.at(i).begin() + pilot_stop);
        auto ofdm_data = Utils::Cint16ToCfloat32(ofdm_samps);
        float snr_val = CommsLib::ComputeOfdmSnr(
            ofdm_data, cfg_->OfdmDataStart(), cfg_->OfdmDataStop());
        snr.push_back(snr_val);
        std::cout << snr_val << " ";
      }
      std::cout << std::endl;
      auto dl_max_snr_it = std::max_element(snr.begin(), snr.end());
      auto dl_min_snr_it = std::min_element(snr.begin(), snr.end());
      std::cout << "Min DL SNR at antenna " << dl_min_snr_it - snr.begin()
                << ": " << *dl_min_snr_it << std::endl;
      std::cout << "Max DL SNR at antenna " << dl_max_snr_it - snr.begin()
                << ": " << *dl_max_snr_it << std::endl;
      std::cout << "*************************************************"
                << std::endl;
      cfg_->OfdmRxZeroPrefixCalDl(min_offset - (min_offset % 4));
      break;
    }
    n++;
  }
  if (n >= max_retries) {
    std::cout << "Reached max retries for sample offset calibration (downlink)"
              << std::endl;
    exit(0);
  }
}

bool RadioConfig::InitialCalib() {
  // excludes zero padding
  const size_t seq_len = cfg_->PilotCf32().size();
  const size_t read_len = cfg_->PilotCi16().size();

  // Transmitting from only one chain, create a null vector for chainB
  std::vector<std::complex<int16_t>> dummy_ci16(read_len, 0);

  std::vector<void*> txbuff0(2);
  std::vector<void*> txbuff1(2);
  txbuff0[0] = cfg_->PilotCi16().data();
  if (cfg_->NumChannels() == 2) {
    std::vector<std::complex<int16_t>> zeros(read_len,
                                             std::complex<int16_t>(0, 0));
    txbuff0[1] = zeros.data();
    txbuff1[0] = zeros.data();
    txbuff1[1] = cfg_->PilotCi16().data();
  }

  std::vector<std::vector<std::complex<int16_t>>> buff;
  // int ant = cfg_->num_channels();
  size_t m = cfg_->BsAntNum();
  // TODO: Fix for multi-cell
  size_t r = cfg_->NumRadios();
  size_t ref = cfg_->RefRadio(0);
  // allocate for uplink and downlink directions
  buff.resize(2 * m);
  for (size_t i = 0; i < 2 * m; i++) {
    buff.at(i).resize(read_len);
  }

  std::vector<std::complex<int16_t>> dummybuff(read_len);
  // DrainBuffers();

  //for (size_t i = 0; i < r; i++) {
  //  for (size_t ch = 0; ch < cfg_->NumChannels(); ch++) {
  //    ba_stn_.at(i)->setGain(
  //        SOAPY_SDR_TX, ch, "PAD",
  //        ch != 0u ? cfg_->CalibTxGainB() : cfg_->CalibTxGainA());
  //  }
  //  ba_stn_.at(i)->writeSetting("TDD_CONFIG", "{\"tdd_enabled\":false}");
  //  ba_stn_.at(i)->writeSetting("TDD_MODE", "false");
  //  ba_stn_.at(i)->activateStream(this->tx_streams_.at(i));
  //}

  size_t good_csi_cnt = 0;
  size_t n = 0;
  const size_t max_retries = 3;
  // second condition is for when too many attemps fail
  while ((good_csi_cnt < calib_meas_num_) && (n < (6 * calib_meas_num_))) {
    long long tx_time(0);
    long long rx_time(0);
    bool good_csi = true;
    // Transmit from Beamforming Antennas to Ref Antenna (Down)
    for (size_t i = 0; i < r; i++) {
      if (good_csi == false) {
        break;
      }
      if (i == ref) {
        continue;
      }

      // Send a separate pilot from each antenna
      for (size_t ch = 0; ch < cfg_->NumChannels(); ch++) {
        int tx_flags = SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST;
        size_t retry = 0;
        bool bad_read = false;
        while (retry < max_retries) {
          bad_read = false;
          int ret = ba_stn_.at(i)->writeStream(
              this->tx_streams_.at(i), ch > 0 ? txbuff1.data() : txbuff0.data(),
              read_len, tx_flags, tx_time, kTxTimeoutUs);
          if (ret < (int)read_len) {
            std::cout << "bad write\n";
          }

          int rx_flags_activate = SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST;
          ret = ba_stn_.at(ref)->activateStream(
              this->rx_streams_.at(ref), rx_flags_activate, rx_time, read_len);

          Go();  // trigger

          int rx_flags = SOAPY_SDR_END_BURST;
          std::vector<void*> rxbuff0(2);
          rxbuff0.at(0) = buff.at(cfg_->NumChannels() * i + ch).data();
          if (cfg_->NumChannels() == 2) {
            rxbuff0.at(1) = dummybuff.data();
          }
          ret = ba_stn_.at(ref)->readStream(this->rx_streams_.at(ref),
                                            rxbuff0.data(), read_len, rx_flags,
                                            rx_time, kRxTimeoutUs);
          if (ret < (int)read_len) {
            std::cout << "bad read (" << ret << ") at node " << ref
                      << " from node " << i << std::endl;
            retry++;
          } else {
            break;
          }
        }
        if (bad_read) {
          good_csi = false;
          break;
        }
      }
    }
    // Transmit from Ref Antenna to Beamforming Antennas (Up)
    if (good_csi == true) {
      int tx_flags = SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST;
      size_t retry = 0;
      bool bad_read = false;
      while (retry < max_retries) {
        bad_read = false;
        int ret = ba_stn_.at(ref)->writeStream(this->tx_streams_.at(ref),
                                               txbuff0.data(), read_len,
                                               tx_flags, tx_time, kTxTimeoutUs);
        if (ret < (int)read_len) {
          std::cout << "bad write\n";
        }

        for (size_t i = 0; i < r; i++) {
          if (i != ref) {
            int rx_flags = SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST;
            ret = ba_stn_.at(i)->activateStream(this->rx_streams_.at(i),
                                                rx_flags, rx_time, read_len);
          }
        }

        Go();  // Trigger

        int rx_flags = SOAPY_SDR_END_BURST;
        for (size_t i = 0; i < r; i++) {
          if (good_csi == false) {
            break;
          }
          if (i == ref) {
            continue;
          }
          std::vector<void*> rxbuff(2);
          rxbuff.at(0) = buff.at(m + cfg_->NumChannels() * i).data();
          if (cfg_->NumChannels() == 2) {
            rxbuff.at(1) = buff.at(m + cfg_->NumChannels() * i + 1).data();
          }
          ret = ba_stn_.at(i)->readStream(this->rx_streams_.at(i),
                                          rxbuff.data(), read_len, rx_flags,
                                          rx_time, kRxTimeoutUs);
          if (ret < (int)read_len) {
            bad_read = true;
            std::cout << "Bad read (" << ret << ") at node " << i
                      << " from node " << ref << std::endl;
          }
        }
        if (bad_read) {
          retry++;
        } else {
          break;
        }
      }
      if (bad_read) {
        good_csi = false;
      }
    }

    std::vector<std::vector<std::complex<int16_t>>> noise_buff;
    if (good_csi == true) {
      noise_buff.resize(m);
      for (size_t i = 0; i < r; i++) {
        int rx_flags = SOAPY_SDR_END_BURST;
        int ret = ba_stn_.at(i)->activateStream(this->rx_streams_.at(i),
                                                rx_flags, rx_time, read_len);
        std::vector<void*> rxbuff(2);
        noise_buff.at(cfg_->NumChannels() * i).resize(read_len);
        rxbuff.at(0) = noise_buff.at(cfg_->NumChannels() * i).data();

        if (cfg_->NumChannels() == 2) {
          noise_buff.at(cfg_->NumChannels() * i + 1).resize(read_len);
          rxbuff.at(1) = noise_buff.at(cfg_->NumChannels() * i + 1).data();
        }
        ret = ba_stn_.at(i)->readStream(this->rx_streams_.at(i), rxbuff.data(),
                                        read_len, rx_flags, rx_time,
                                        kRxTimeoutUs);
        if (ret < (int)read_len) {
          good_csi = false;
          std::cout << "bad noise read (" << ret << ") at node " << i
                    << std::endl;
        }
        // ba_stn_.at(i)->deactivateStream(this->rx_streams_.at(i));
      }
    }

    std::vector<std::vector<std::complex<float>>> noise(m);
    std::vector<std::vector<std::complex<float>>> up(m);
    std::vector<std::vector<std::complex<float>>> dn(m);
    std::vector<size_t> start_up(m, 0);
    std::vector<size_t> start_dn(m, 0);
    std::vector<int> offset(r);
    if (good_csi == true) {
      for (size_t i = 0; i < m; i++) {
        noise[i].resize(read_len);
        std::transform(noise_buff.at(i).begin(), noise_buff.at(i).end(),
                       noise.at(i).begin(), [](std::complex<int16_t> ci) {
                         return std::complex<float>(ci.real() / 32768.0,
                                                    ci.imag() / 32768.0);
                       });
      }

      for (size_t i = 0; i < m; i++) {
        if (i / cfg_->NumChannels() == ref) {
          continue;
        }

        up[i].resize(read_len);
        dn[i].resize(read_len);
        std::transform(buff.at(m + i).begin(), buff.at(m + i).end(),
                       up.at(i).begin(), [](std::complex<int16_t> ci) {
                         return std::complex<float>(ci.real() / 32768.0,
                                                    ci.imag() / 32768.0);
                       });
        std::transform(buff.at(i).begin(), buff.at(i).end(), dn.at(i).begin(),
                       [](std::complex<int16_t> ci) {
                         return std::complex<float>(ci.real() / 32768.0,
                                                    ci.imag() / 32768.0);
                       });
      }

      std::stringstream ss0;
      ss0 << "SNR_dn" << n << " = [";
      std::stringstream ss1;
      ss1 << "SNR_up" << n << " = [";
      for (size_t i = 0; i < m; i++) {
        if (good_csi == false) {
          break;
        }
        if (i / cfg_->NumChannels() == ref) {
          continue;
        }
        if (i % cfg_->NumChannels() == 0) {
          size_t peak_up =
              CommsLib::FindPilotSeq(up.at(i), cfg_->PilotCf32(), seq_len);
          size_t peak_dn =
              CommsLib::FindPilotSeq(dn.at(i), cfg_->PilotCf32(), seq_len);
          start_up[i] =
              peak_up < seq_len ? 0 : peak_up - seq_len + cfg_->CpLen();
          start_dn[i] =
              peak_dn < seq_len ? 0 : peak_dn - seq_len + cfg_->CpLen();
          if (kVerboseCalibration) {
            std::cout << "receive starting position from/to node "
                      << i / cfg_->NumChannels() << ": " << peak_up << "/"
                      << peak_dn << std::endl;
          }
          if ((start_up.at(i) == 0) || (start_dn.at(i) == 0)) {
            good_csi = false;
            std::cout << "Potential bad pilots: At node " << i
                      << " uplink/downlink peak found at index " << peak_up
                      << "/" << peak_dn << std::endl;
            if (kVerboseCalibration) {
              std::cout << "dn(" << i << ")=";
              for (auto& s : dn.at(i)) {
                std::cout << s.real() << "+1j*" << s.imag() << " ";
              }
              std::cout << std::endl;
              std::cout << "up(" << i << ")=";
              for (auto& s : up.at(i)) {
                std::cout << s.real() << "+1j*" << s.imag() << " ";
              }
              std::cout << std::endl;
            }
            break;
          }
        } else {
          start_up.at(i) = start_up.at(i - 1);
          start_dn.at(i) = start_dn.at(i - 1);
        }

        float sig_up = 0;
        float noise_up = 0;
        if (cfg_->OfdmCaNum() + start_up.at(i) >= up.at(i).size()) {
          std::cout << "up(" << i << ")=";
          //for (size_t s = 0; s < up.at(i).size(); s++) {
          //  std::cout << up.at(i).at(s).real() << "+1j*"
          //            << up.at(i).at(s).imag() << " ";
          //}
          std::cout << std::endl;
          std::cout << "Uplink pilot offset (" << start_up.at(i)
                    << ") too large!" << std::endl;
          good_csi = false;
          break;
        }
        for (size_t q = 0; q < cfg_->OfdmCaNum(); q++) {
          sig_up += std::pow(std::abs(up.at(i).at(q + start_up.at(i))), 2);
          noise_up += std::pow(std::abs(noise.at(i).at(q + start_up.at(i))), 2);
        }
        ss1 << 10 * std::log10(sig_up / noise_up) << " ";

        float sig_dn = 0;
        float noise_dn = 0;
        if (cfg_->OfdmCaNum() + start_dn.at(i) >= dn.at(i).size()) {
          std::cout << "Downlink pilot offset (" << start_dn.at(i)
                    << ") too large!" << std::endl;
          std::cout << "dn(" << i << ")=";
          //for (size_t s = 0; s < dn.at(i).size(); s++) {
          //  std::cout << dn.at(i).at(s).real() << "+1j*"
          //            << dn.at(i).at(s).imag() << " ";
          //}
          std::cout << std::endl;
          good_csi = false;
          break;
        }
        for (size_t q = 0; q < cfg_->OfdmCaNum(); q++) {
          sig_dn += std::pow(std::abs(dn.at(i).at(q + start_dn.at(i))), 2);
          noise_dn += std::pow(
              std::abs(noise.at(cfg_->RefAnt(0)).at(q + start_dn.at(i))), 2);
        }
        ss0 << 10 * std::log10(sig_dn / noise_dn) << " ";
        if (kReciprocalCalibPlot) {
          std::vector<double> up_i(read_len);
          std::transform(up.at(i).begin(), up.at(i).end(), up_i.begin(),
                         [](std::complex<double> cd) { return cd.real(); });

          std::vector<double> dn_i(read_len);
          std::transform(dn.at(i).begin(), dn.at(i).end(), dn_i.begin(),
                         [](std::complex<double> cd) { return cd.real(); });

          plt::figure_size(1200, 780);
          plt::plot(up_i);
          // plt::xlim(0, read_len);
          plt::ylim(-1, 1);
          plt::title("ant " + std::to_string(cfg_->RefAnt(0)) +
                     " (ref) to ant " + std::to_string(i));
          plt::legend();
          plt::save("up_" + std::to_string(i) + ".png");

          plt::figure_size(1200, 780);
          plt::plot(dn_i);
          // plt::xlim(0, read_len);
          plt::ylim(-1, 1);
          plt::title("ant " + std::to_string(i) + " to ant (ref)" +
                     std::to_string(cfg_->RefAnt(0)));
          plt::legend();
          plt::save("dn_" + std::to_string(i) + ".png");
        }
        if (i % cfg_->NumChannels() == 0) {
          if ((i > 0) &&
              ((std::abs((int)start_up.at(i) -
                         (int)start_up.at(i - cfg_->NumChannels())) >
                static_cast<int>(kMaxArraySampleOffset)) ||
               (std::abs((int)start_dn.at(i) -
                         (int)start_dn.at(i - cfg_->NumChannels())) >
                static_cast<int>(
                    kMaxArraySampleOffset)))) {  // make sure offsets are not too
                                                 // different from each other
            std::cout << "Uplink and Downlink pilot offsets (" << start_up.at(i)
                      << "/" << start_up.at(i - cfg_->NumChannels()) << ", "
                      << start_dn.at(i) << "/"
                      << start_dn.at(i - cfg_->NumChannels())
                      << ") are far apart!" << std::endl;
            good_csi = false;
            break;
          }
          offset.at(i / cfg_->NumChannels()) = start_up.at(i);
        }
      }
      ss0 << "];\n";
      ss1 << "];\n";
      if (kVerboseCalibration) {
        std::cout << ss1.str();
        std::cout << ss0.str();
      }
    }

    n++;  // increment number of measurement attemps
    if (good_csi == false) {
      std::cout << "Attempt " << n << " failed. Retrying..." << std::endl;
      continue;
    } else {
      std::cout << "Attempt " << n << " Succeeded. Processing " << good_csi_cnt
                << "th calibration result..." << std::endl;

      for (size_t i = 0; i < m; i++) {
        size_t id = i;
        if (cfg_->ExternalRefNode(0) && i / cfg_->NumChannels() == ref) {
          continue;
        }
        if (cfg_->ExternalRefNode(0) && (i / cfg_->NumChannels() > ref)) {
          id = i - cfg_->NumChannels();
        }
        if (kVerboseCalibration) {  // print time-domain data
          std::cout << "up_t" << id << " = [";
          for (size_t j = 0; j < read_len; j++) {
            std::cout << buff.at(m + i).at(j).real() << "+1j*"
                      << buff.at(m + i).at(j).imag() << " ";
          }
          std::cout << "];" << std::endl;
          std::cout << "dn_t" << id << " = [";
          for (size_t j = 0; j < read_len; j++) {
            std::cout << buff.at(i).at(j).real() << "+1j*"
                      << buff.at(i).at(j).imag() << " ";
          }
          std::cout << "];" << std::endl;
        }
        // computing reciprocity calibration matrix
        auto first_up = up.at(i).begin() + start_up.at(i);
        auto last_up = up.at(i).begin() + start_up.at(i) + cfg_->OfdmCaNum();
        std::vector<std::complex<float>> up_ofdm(first_up, last_up);
        assert(up_ofdm.size() == cfg_->OfdmCaNum());

        auto first_dn = dn.at(i).begin() + start_dn.at(i);
        auto last_dn = dn.at(i).begin() + start_dn.at(i) + cfg_->OfdmCaNum();
        std::vector<std::complex<float>> dn_ofdm(first_dn, last_dn);
        assert(dn_ofdm.size() == cfg_->OfdmCaNum());

        //dn_ofdm / up_ofdm is transformed to the fft output at this point
        CommsLib::FFT(dn_ofdm, cfg_->OfdmCaNum());
        CommsLib::FFT(up_ofdm, cfg_->OfdmCaNum());
        if (cfg_->ExternalRefNode(0) == false &&
            i / cfg_->NumChannels() == ref) {
          for (size_t j = 0; j < cfg_->OfdmCaNum(); j++) {
            dn_ofdm[j] = std::complex<float>(1, 0);
            up_ofdm[j] = std::complex<float>(1, 0);
          }
        }
        arma::cx_fvec dn_vec(
            reinterpret_cast<arma::cx_float*>(&dn_ofdm[cfg_->OfdmDataStart()]),
            cfg_->OfdmDataNum(), false);
        arma::cx_fvec calib_dl_vec(
            reinterpret_cast<arma::cx_float*>(
                &init_calib_dl_[good_csi_cnt][id * cfg_->OfdmDataNum()]),
            cfg_->OfdmDataNum(), false);
        calib_dl_vec = dn_vec;

        arma::cx_fvec up_vec(
            reinterpret_cast<arma::cx_float*>(&up_ofdm[cfg_->OfdmDataStart()]),
            cfg_->OfdmDataNum(), false);
        arma::cx_fvec calib_ul_vec(
            reinterpret_cast<arma::cx_float*>(
                &init_calib_ul_[good_csi_cnt][id * cfg_->OfdmDataNum()]),
            cfg_->OfdmDataNum(), false);
        calib_ul_vec = up_vec;
        // Utils::print_vec(dn_vec / up_vec,
        //     "n" + std::to_string(good_csi_cnt) + "_ant" + std::to_string(i));
      }
      good_csi_cnt++;
    }
  }
  //for (size_t i = 0; i < r; i++) {
  //  ba_stn_.at(i)->deactivateStream(this->tx_streams_.at(i));
  //  ba_stn_.at(i)->deactivateStream(this->rx_streams_.at(i));
  //  for (size_t ch = 0; ch < cfg_->NumChannels(); ch++) {
  //    ba_stn_.at(i)->setGain(SOAPY_SDR_TX, ch, "PAD",
  //                           ch != 0u ? cfg_->TxGainB() : cfg_->TxGainA());
  //  }
  //}
  return good_csi_cnt == calib_meas_num_;
}
