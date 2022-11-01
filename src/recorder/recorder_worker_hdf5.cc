/**
 * @file recorder_worker_hdf5.cc
 * @brief Recorder worker to write hdf5 output file with all rx symbols
 */

#include "recorder_worker_hdf5.h"

#include <ctime>
#include <fstream>
#include <nlohmann/json.hpp>

#include "comms-lib.h"
#include "logger.h"
#include "utils.h"
#include "version_config.h"

namespace Agora_recorder {

static constexpr bool kDebugPrint = false;
static constexpr size_t kFrameInc = 2000;
static constexpr ssize_t kFixedDimensions = -1;
static const std::string kHdf5Group = "Data";

static constexpr size_t kBeaconDatasetIndex = 0;
static constexpr size_t kDownlinkDatasetIndex = 1;
static constexpr size_t kPilotDatasetIndex = 0;
static constexpr size_t kUplinkDatasetIndex = 1;

RecorderWorkerHDF5::RecorderWorkerHDF5(const Config* in_cfg,
                                       size_t antenna_offset,
                                       size_t num_antennas,
                                       size_t record_interval,
                                       Direction rx_direction)
    : RecorderWorker(in_cfg, antenna_offset, num_antennas, record_interval,
                     rx_direction),
      cfg_(in_cfg),
      antenna_offset_(antenna_offset),
      num_antennas_(num_antennas),
      interval_(record_interval),
      rx_direction_(rx_direction),
      max_frame_number_(0),
      data_chunk_dims_{
          {1, 1, 1, 1, static_cast<hsize_t>(2 * cfg_->SampsPerSymbol())}} {}

RecorderWorkerHDF5::~RecorderWorkerHDF5() = default;

void RecorderWorkerHDF5::Init() {
  AGORA_LOG_INFO("RecorderWorkerHDF5::Creating output HD5F file: %s\n",
                 cfg_->TraceFilename().c_str());
  hdf5_ = std::make_unique<Hdf5Lib>(cfg_->TraceFilename(), kHdf5Group);

  hdf5_->WriteAttribute("SOFTWARE", "AGORA");
  hdf5_->WriteAttribute("SOFTWARE_VERSION", GetAgoraProjectVersion());

  auto time = std::time(nullptr);
  auto local_time = *std::localtime(&time);

  std::ostringstream oss;
  oss << std::put_time(&local_time, "%d-%m-%Y %H-%M-%S");
  hdf5_->WriteAttribute("TIME", oss.str());

  // Write the entire config
  const std::string conf_filename = cfg_->ConfigFilename();
  std::ifstream conf(conf_filename);
  nlohmann::json data = nlohmann::json::parse(conf, nullptr, true, true);
  conf.close();
  hdf5_->WriteAttribute("CONFIG", data.dump());

  hdf5_->WriteAttribute("MAX_FRAME", cfg_->FramesToTest());
  // Write Atrributes
  // ******* COMMON ******** //
  // TX/RX Frequencyfile
  hdf5_->WriteAttribute("FREQ", cfg_->Freq());

  // BW
  hdf5_->WriteAttribute("RATE", cfg_->Rate());

  // Number of samples for prefix (padding)
  hdf5_->WriteAttribute("TX_ZERO_PREFIX_LEN", cfg_->OfdmTxZeroPrefix());

  // Number of samples for postfix (padding)
  hdf5_->WriteAttribute("TX_ZERO_POSTFIX_LEN", cfg_->OfdmTxZeroPostfix());

  // Number of samples on each symbol including prefix and postfix
  hdf5_->WriteAttribute("SLOT_SAMP_LEN", cfg_->SampsPerSymbol());

  // Length of cyclic prefix
  hdf5_->WriteAttribute("CP_LEN", cfg_->CpLen());

  hdf5_->WriteAttribute("OFDM_DATA_NUM", cfg_->OfdmDataNum());
  hdf5_->WriteAttribute("OFDM_DATA_START", cfg_->OfdmDataStart());
  hdf5_->WriteAttribute("OFDM_DATA_STOP", cfg_->OfdmDataStop());
  hdf5_->WriteAttribute("OFDM_PILOT_SPACING", cfg_->OfdmPilotSpacing());
  hdf5_->WriteAttribute("OFDM_CA_NUM", cfg_->OfdmCaNum());

  // ******* Base Station ******** //
  // Hub ID
  hdf5_->WriteAttribute("BS_HUB_ID", cfg_->HubId());

  // BS SDR IDs
  hdf5_->WriteAttribute("BS_SDR_ID", cfg_->RadioId());

  // Number of Base Station Cells
  hdf5_->WriteAttribute("BS_NUM_CELLS", cfg_->NumCells());

  // How many RF channels per Iris board are enabled ("single" or "dual")
  hdf5_->WriteAttribute("BS_CH_PER_RADIO", cfg_->NumChannels());

  // Frame schedule (vec of strings for now, this should change to matrix when we go to multi-cell)
  hdf5_->WriteAttribute("BS_FRAME_SCHED", cfg_->Frame().FrameIdentifier());

  // RX Gain RF channel A
  hdf5_->WriteAttribute("BS_RX_GAIN_A", cfg_->RxGainA());

  // TX Gain RF channel A
  hdf5_->WriteAttribute("BS_TX_GAIN_A", cfg_->TxGainA());

  // RX Gain RF channel B
  hdf5_->WriteAttribute("BS_RX_GAIN_B", cfg_->RxGainB());

  // TX Gain RF channel B
  hdf5_->WriteAttribute("BS_TX_GAIN_B", cfg_->TxGainB());

  // Beamsweep (true or false)
  hdf5_->WriteAttribute("BS_BEAMSWEEP", cfg_->Beamsweep() ? 1 : 0);

  // Beacon Antenna
  hdf5_->WriteAttribute("BS_BEACON_ANT", cfg_->BeaconAnt());

  hdf5_->WriteAttribute("ANT_OFFSET", antenna_offset_);
  hdf5_->WriteAttribute("ANT_NUM", num_antennas_);
  hdf5_->WriteAttribute("ANT_TOTAL", cfg_->BsAntNum());

  // Number of symbols in a frame
  hdf5_->WriteAttribute("BS_FRAME_LEN", cfg_->Frame().NumTotalSyms());

  // Number of uplink symbols per frame
  hdf5_->WriteAttribute("UL_SLOTS", cfg_->Frame().NumULSyms());
  hdf5_->WriteAttribute("DL_SLOTS", cfg_->Frame().NumDLSyms());
  hdf5_->WriteAttribute("PILOT_SLOTS", cfg_->Frame().NumPilotSyms());
  //PILOT_NUM
  hdf5_->WriteAttribute("DL_CAL_SLOTS", cfg_->Frame().NumDLCalSyms());
  hdf5_->WriteAttribute("UL_CAL_SLOTS", cfg_->Frame().NumULCalSyms());
  hdf5_->WriteAttribute("UL_PILOT_SLOTS", cfg_->Frame().ClientUlPilotSymbols());
  hdf5_->WriteAttribute("DL_PILOT_SLOTS", cfg_->Frame().ClientDlPilotSymbols());

  // Reciprocal Calibration Mode
  hdf5_->WriteAttribute("RECIPROCAL_CALIB",
                        cfg_->Frame().IsRecCalEnabled() ? 1 : 0);

  // Number of Client Antennas
  hdf5_->WriteAttribute("CL_NUM", cfg_->UeAntNum());

  // Data modulation
  hdf5_->WriteAttribute("UL_MCS", cfg_->MCSParams(Direction::kUplink).dump());
  hdf5_->WriteAttribute("DL_MCS", cfg_->MCSParams(Direction::kDownlink).dump());

  // Client antenna polarization
  //Should loop over all values
  hdf5_->WriteAttribute("CL_CH_PER_RADIO", cfg_->NumUeChannels());

  // RX Gain RF channel A
  hdf5_->WriteAttribute("CL_RX_GAIN_A", cfg_->ClientRxGainA());

  // TX Gain RF channel A
  hdf5_->WriteAttribute("CL_TX_GAIN_A", cfg_->ClientTxGainA());

  // RX Gain RF channel B
  hdf5_->WriteAttribute("CL_RX_GAIN_B", cfg_->ClientRxGainB());

  // TX Gain RF channel B
  hdf5_->WriteAttribute("CL_TX_GAIN_B", cfg_->ClientTxGainB());

  // Set of client SDR IDs (vec of strings)
  hdf5_->WriteAttribute("CL_SDR_ID", cfg_->UeRadioId());
  // ********************* //

  // *****Temp compatibility values
  hdf5_->WriteAttribute("DATA_SUBCARRIER_NUM", cfg_->OfdmDataNum());
  const size_t fft_size = cfg_->OfdmCaNum();
  const size_t sym_data_sc_num = cfg_->OfdmDataNum();
  hdf5_->WriteAttribute("FFT_SIZE", fft_size);
  hdf5_->WriteAttribute("PILOT_NUM", cfg_->Frame().NumPilotSyms());
  hdf5_->WriteAttribute("PREFIX_LEN", cfg_->OfdmTxZeroPrefix());
  hdf5_->WriteAttribute("POSTFIX_LEN", cfg_->OfdmTxZeroPostfix());
  hdf5_->WriteAttribute("PILOT_SEQ_TYPE", std::string("zadoff-chu"));

  // Data subcarriers
  const size_t ul_pilot_sc_spacing = 1;
  //with sc_pilot_spacing set to 1, then the offset can be anything but 0
  const auto data_idx =
      CommsLib::GetDataSc(fft_size, sym_data_sc_num, 1, ul_pilot_sc_spacing);
  if (data_idx.empty() == false) {
    hdf5_->WriteAttribute("OFDM_DATA_SC", data_idx);
  }

  const size_t ul_pilot_sc_offset = 0;
  // Pilot subcarriers (indexes)
  const auto pilot_sc_idx = CommsLib::GetPilotScIdx(
      fft_size, sym_data_sc_num, ul_pilot_sc_offset, ul_pilot_sc_spacing);
  if (pilot_sc_idx.empty() == false) {
    hdf5_->WriteAttribute("OFDM_PILOT_SC", pilot_sc_idx);
  }

  const auto pilot_sc = CommsLib::GetPilotScValue(
      fft_size, sym_data_sc_num, ul_pilot_sc_offset, ul_pilot_sc_spacing);
  if (pilot_sc.empty() == false) {
    hdf5_->WriteAttribute("OFDM_PILOT_SC_VALS", pilot_sc);
  }

  // Freq. Domain Pilot symbols (2 for complex)
  std::vector<double> split_vec_pilot_f(2 * fft_size, 0.0);
  const auto ofdm_data_start = cfg_->OfdmDataStart();
  for (size_t i = 0; i < cfg_->OfdmDataNum(); i++) {
    split_vec_pilot_f[(2 * (ofdm_data_start + i)) + 0] =
        cfg_->CommonPilot().at(i).real();
    split_vec_pilot_f[(2 * (ofdm_data_start + i)) + 1] =
        cfg_->CommonPilot().at(i).imag();
  }
  hdf5_->WriteAttribute("OFDM_PILOT_F", split_vec_pilot_f);

  // Number of frames for UL data recorded in bit source files
  hdf5_->WriteAttribute("CL_MODULATION", cfg_->Modulation(Direction::kUplink));
  hdf5_->WriteAttribute("UL_DATA_FRAME_NUM", 1);

  // Names of Files including uplink tx frequency-domain data
  if (cfg_->UlTxFreqDataFiles().empty() == false) {
    hdf5_->WriteAttribute("TX_FD_DATA_FILENAMES", cfg_->UlTxFreqDataFiles());
  }
  // *****Temp compatibility values

  if (rx_direction_ == Direction::kDownlink) {
    if (cfg_->Frame().NumBeaconSyms() > 0) {
      datasets_.emplace_back(
          std::make_pair<std::string, std::array<hsize_t, kDsDimsNum>>(
              std::string("BeaconData"),
              {kFrameInc, cfg_->NumCells(), cfg_->Frame().NumBeaconSyms(),
               num_antennas_, data_chunk_dims_.back()}));
      auto& current_dataset = datasets_.back();
      hdf5_->CreateDataset(current_dataset.first, data_chunk_dims_,
                           current_dataset.second);
    }

    if (cfg_->Frame().NumDLSyms() > 0) {
      datasets_.emplace_back(
          std::make_pair<std::string, std::array<hsize_t, kDsDimsNum>>(
              std::string("DownlinkData"),
              {kFrameInc, cfg_->NumCells(), cfg_->Frame().NumDLSyms(),
               num_antennas_, data_chunk_dims_.back()}));

      auto& current_dataset = datasets_.back();
      hdf5_->CreateDataset(current_dataset.first, data_chunk_dims_,
                           current_dataset.second);

      //Adding the ground truths as a DataSet
      {  //TxData
        const std::string dataset_name("TxData");
        //Including the Pilots here.....
        //*2 for complex
        const hsize_t tx_data_size = 2 * cfg_->OfdmDataNum();
        const size_t num_dl_syms = cfg_->Frame().NumDLSyms();
        const std::array<hsize_t, kDsDimsNum> tx_data_dims = {1, 1, 1, 1,
                                                              tx_data_size};
        const std::array<hsize_t, kDsDimsNum> total_dims = {
            1, 1, num_dl_syms, num_antennas_, tx_data_dims.back()};

        hdf5_->CreateDataset(dataset_name, tx_data_dims, total_dims,
                             kFixedDimensions, H5::PredType::INTEL_F32);

        for (size_t ant = 0; ant < num_antennas_; ant++) {
          for (size_t sym = 0; sym < num_dl_syms; sym++) {
            const std::array<hsize_t, kDsDimsNum> start = {0, 0, sym, ant, 0};
            AGORA_LOG_TRACE(
                "Attempting to write TxData for Symbol %zu, Antenna %zu "
                "total syms %zu\n",
                sym, ant, num_dl_syms);
            //DlIqF is indexed by complex float
            hdf5_->WriteDataset(
                dataset_name, start, tx_data_dims,
                reinterpret_cast<const float*>(
                    &const_cast<Config*>(cfg_)
                         ->DlIqF()[sym][ant * (tx_data_size / 2)]));
          }
        }
        hdf5_->FinalizeDataset(dataset_name);
      }

      {  //TXPilot
        const std::string dataset_name("TxPilot");
        //2 for complex float
        const hsize_t tx_pilot_sym_size = 2 * cfg_->OfdmDataNum();
        const size_t num_dl_pilot_syms = cfg_->Frame().ClientDlPilotSymbols();
        const std::array<hsize_t, kDsDimsNum> tx_pilot_dims = {
            1, 1, 1, 1, tx_pilot_sym_size};
        const std::array<hsize_t, kDsDimsNum> total_dims = {
            1, 1, num_dl_pilot_syms, num_antennas_, tx_pilot_dims.back()};

        hdf5_->CreateDataset(dataset_name, tx_pilot_dims, total_dims,
                             kFixedDimensions, H5::PredType::INTEL_F32);

        for (size_t ant = 0; ant < num_antennas_; ant++) {
          for (size_t sym = 0; sym < num_dl_pilot_syms; sym++) {
            const std::array<hsize_t, kDsDimsNum> start = {0, 0, sym, ant, 0};
            AGORA_LOG_TRACE(
                "Attempting to write TxPilot for Antenna %zu Symbol %zu\n", ant,
                sym);
            //Since UeSpecificPilot == OfdmDataNum size then repeat for each symbol
            hdf5_->WriteDataset(
                dataset_name, start, tx_pilot_dims,
                reinterpret_cast<const float*>(
                    &const_cast<Config*>(cfg_)->UeSpecificPilot()[ant][0u]));
          }
        }
        hdf5_->FinalizeDataset(dataset_name);
      }
    }  // end cfg_->Frame().NumDLSyms() > 0
  } else {
    if (cfg_->Frame().NumPilotSyms() > 0) {
      datasets_.emplace_back(
          std::make_pair<std::string, std::array<hsize_t, kDsDimsNum>>(
              std::string("Pilot_Samples"),
              {kFrameInc, cfg_->NumCells(), cfg_->Frame().NumPilotSyms(),
               num_antennas_, data_chunk_dims_.back()}));
      auto& current_dataset = datasets_.back();
      hdf5_->CreateDataset(current_dataset.first, data_chunk_dims_,
                           current_dataset.second);
    }

    if (cfg_->Frame().NumULSyms() > 0) {
      datasets_.emplace_back(
          std::make_pair<std::string, std::array<hsize_t, kDsDimsNum>>(
              std::string("UplinkData"),
              {kFrameInc, cfg_->NumCells(), cfg_->Frame().NumULSyms(),
               num_antennas_, data_chunk_dims_.back()}));
      auto& current_dataset = datasets_.back();
      hdf5_->CreateDataset(current_dataset.first, data_chunk_dims_,
                           current_dataset.second);

      //Adding the ground truths as a DataSet
      {
        const std::string dataset_name("TxUplink");
        //*2 for complex
        const hsize_t tx_data_size = 2 * cfg_->OfdmDataNum();
        const size_t num_ul_syms = cfg_->Frame().NumULSyms();
        const std::array<hsize_t, kDsDimsNum> tx_data_dims = {1, 1, 1, 1,
                                                              tx_data_size};
        const std::array<hsize_t, kDsDimsNum> total_dims = {
            1, 1, num_ul_syms, cfg_->UeAntNum(), tx_data_dims.back()};

        hdf5_->CreateDataset(dataset_name, tx_data_dims, total_dims,
                             kFixedDimensions, H5::PredType::INTEL_F32);

        for (size_t ant = 0; ant < cfg_->UeAntNum(); ant++) {
          for (size_t sym = 0; sym < num_ul_syms; sym++) {
            const std::array<hsize_t, kDsDimsNum> start = {0, 0, sym, ant, 0};
            AGORA_LOG_TRACE(
                "Attempting to write TxUplink for Symbol %zu, Antenna %zu "
                "total syms %zu\n",
                sym, ant, num_ul_syms);
            //UlIqF is indexed by complex float
            hdf5_->WriteDataset(
                dataset_name, start, tx_data_dims,
                reinterpret_cast<const float*>(
                    &const_cast<Config*>(cfg_)
                         ->UlIqF()[sym][ant * (tx_data_size / 2)]));
          }
        }
        hdf5_->FinalizeDataset(dataset_name);
      }
    }
  }
}

void RecorderWorkerHDF5::Finalize() { hdf5_.reset(); }

void RecorderWorkerHDF5::WriteDatasetValue(const Packet* pkt,
                                           size_t symbol_index,
                                           size_t dataset_index) {
  const size_t frame_id = pkt->frame_id_;
  const size_t ant_id = pkt->ant_id_;
  const uint32_t antenna_index = ant_id - antenna_offset_;
  const std::array<hsize_t, kDsDimsNum> start = {
      frame_id, pkt->cell_id_, symbol_index, antenna_index, 0};

  auto& dataset = datasets_.at(dataset_index);
  AGORA_LOG_TRACE("Writting to dataset %s - [%lld, %lld, %lld, %lld, %lld]\n",
                  dataset.first.c_str(), start.at(0), start.at(1), start.at(2),
                  start.at(3), start.at(4));
  ///If the frame id is > than the current 0 indexed dimension then we need to extend
  if (frame_id >= dataset.second.at(0)) {
    dataset.second.at(0) = dataset.second.at(0) + kFrameInc;
    RtAssert(dataset.second.at(0) > frame_id,
             "Frame ID must be less than extended dimension");
    hdf5_->ExtendDataset(dataset.first, dataset.second);
  }
  hdf5_->WriteDataset(dataset.first, start, data_chunk_dims_, pkt->data_);
}

int RecorderWorkerHDF5::Record(const Packet* pkt) {
  const size_t end_antenna = (antenna_offset_ + num_antennas_) - 1;

  if ((pkt->ant_id_ < antenna_offset_) || (pkt->ant_id_ > end_antenna)) {
    AGORA_LOG_ERROR(
        "Antenna id is not within range of this recorder %d, %zu:%zu",
        pkt->ant_id_, antenna_offset_, end_antenna);
  }
  assert((pkt->ant_id_ >= antenna_offset_) && (pkt->ant_id_ <= end_antenna));
  int ret = 0;

  const size_t frame_id = pkt->frame_id_;
  const size_t symbol_id = pkt->symbol_id_;

  if (kDebugPrint) {
    const size_t ant_id = pkt->ant_id_;
    std::printf(
        "RecorderWorkerHDF5::record [frame %zu, symbol %zu, cell %d, "
        "ant %zu] samples: %d %d %d %d %d %d %d %d ....\n",
        frame_id, symbol_id, pkt->cell_id_, ant_id, pkt->data_[0u],
        pkt->data_[1u], pkt->data_[2u], pkt->data_[3u], pkt->data_[4u],
        pkt->data_[5u], pkt->data_[6u], pkt->data_[7u]);
  }

  if (frame_id > cfg_->FramesToTest()) {
    AGORA_LOG_ERROR("Ignoring rx data due to frame id %zu : %zu max\n",
                    frame_id, cfg_->FramesToTest());
  } else if ((frame_id % interval_) == 0) {
    if (kDebugPrint) {
      const size_t ant_id = pkt->ant_id_;
      AGORA_LOG_INFO(
          "RecorderWorkerHDF5::record [frame %zu, symbol %zu, cell %d, "
          "ant %zu] samples: %d %d %d %d %d %d %d %d ....\n",
          frame_id, symbol_id, pkt->cell_id_, ant_id, pkt->data_[0u],
          pkt->data_[1u], pkt->data_[2u], pkt->data_[3u], pkt->data_[4u],
          pkt->data_[5u], pkt->data_[6u], pkt->data_[7u]);
    }

    auto rx_symbol_type = cfg_->GetSymbolType(symbol_id);
    switch (rx_symbol_type) {
      case SymbolType::kBeacon: {
        const size_t beacon_symbol_id =
            cfg_->Frame().GetBeaconSymbolIdx(pkt->symbol_id_);
        WriteDatasetValue(pkt, beacon_symbol_id, kBeaconDatasetIndex);
        break;
      }
      case SymbolType::kDL: {
        const size_t dl_symbol_id =
            cfg_->Frame().GetDLSymbolIdx(pkt->symbol_id_);
        WriteDatasetValue(pkt, dl_symbol_id, kDownlinkDatasetIndex);
        break;
      }
      case SymbolType::kPilot: {
        const size_t pilot_id =
            cfg_->Frame().GetPilotSymbolIdx(pkt->symbol_id_);
        WriteDatasetValue(pkt, pilot_id, kPilotDatasetIndex);
        break;
      }
      case SymbolType::kUL: {
        const size_t ul_symbol_id =
            cfg_->Frame().GetULSymbolIdx(pkt->symbol_id_);
        WriteDatasetValue(pkt, ul_symbol_id, kUplinkDatasetIndex);
        break;
      }
      default: {
      }
    }
  } /* End else */
  return ret;
}
};  // End namespace Agora_recorder
