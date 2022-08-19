/**
 * @file recorder_worker_multifile.cc
 * @brief Recorder worker to write to multiple bin files per rx symbol
 */

#include "recorder_worker_hdf5.h"

#include <ctime>
#include <fstream>
#include <nlohmann/json.hpp>

#include "logger.h"
#include "utils.h"
#include "version_config.h"

namespace Agora_recorder {

static constexpr bool kDebugPrint = false;
static constexpr size_t kFrameInc = 2000;
static constexpr size_t kDsDimSymbol = 2;
static constexpr ssize_t kFixedDimensions = -1;
static const std::string kHdf5Group = "Data";

RecorderWorkerHDF5::RecorderWorkerHDF5(const Config* in_cfg,
                                       size_t antenna_offset,
                                       size_t num_antennas,
                                       size_t record_interval)
    : RecorderWorker(in_cfg, antenna_offset, num_antennas, record_interval),
      cfg_(in_cfg),
      antenna_offset_(antenna_offset),
      num_antennas_(num_antennas),
      interval_(record_interval),
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
  hdf5_->WriteAttribute("BS_NUM_CELLS", cfg_->CellId().size());

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
              "Attempting to write TxData for Symbol %zu, Antenna %zu total "
              "syms %zu\n",
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
      const std::array<hsize_t, kDsDimsNum> tx_pilot_dims = {1, 1, 1, 1,
                                                             tx_pilot_sym_size};
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
}

void RecorderWorkerHDF5::Finalize() { hdf5_.reset(); }

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
  const size_t ant_id = pkt->ant_id_;
  const size_t symbol_id = pkt->symbol_id_;

  if (kDebugPrint) {
    std::printf(
        "RecorderWorkerHDF5::record [frame %zu, symbol %zu, cell %d, "
        "ant %zu] samples: %d %d %d %d %d %d %d %d ....\n",
        frame_id, symbol_id, pkt->cell_id_, ant_id, pkt->data_[0u],
        pkt->data_[1u], pkt->data_[2u], pkt->data_[3u], pkt->data_[4u],
        pkt->data_[5u], pkt->data_[6u], pkt->data_[7u]);
  }

  if (frame_id > cfg_->FramesToTest()) {
    AGORA_LOG_ERROR("Closing file due to frame id %zu : %zu max\n", frame_id,
                    cfg_->FramesToTest());
  } else if ((frame_id % interval_) == 0) {
    if (kDebugPrint) {
      std::printf(
          "RecorderWorkerHDF5::record [frame %zu, symbol %zu, cell %d, "
          "ant %zu] samples: %d %d %d %d %d %d %d %d ....\n",
          frame_id, symbol_id, pkt->cell_id_, ant_id, pkt->data_[0u],
          pkt->data_[1u], pkt->data_[2u], pkt->data_[3u], pkt->data_[4u],
          pkt->data_[5u], pkt->data_[6u], pkt->data_[7u]);
    }

    const uint32_t antenna_index = ant_id - antenna_offset_;
    std::array<hsize_t, kDsDimsNum> start = {frame_id, pkt->cell_id_, 0,
                                             antenna_index, 0};

    //Bad way to check for beacon.  But here it is
    if (symbol_id == 0) {
      auto& dataset = datasets_.at(0);
      ///If the frame id is > than the current 0 indexed dimension then we need to extend
      if (frame_id >= dataset.second.at(0)) {
        dataset.second.at(0) = dataset.second.at(0) + kFrameInc;
        RtAssert(dataset.second.at(0) > frame_id,
                 "Frame ID must be less than extended dimension");
        hdf5_->ExtendDataset(dataset.first, dataset.second);
      }
      start[kDsDimSymbol] = symbol_id;
      hdf5_->WriteDataset(dataset.first, start, data_chunk_dims_, pkt->data_);
    } else if (cfg_->IsDownlink(frame_id, symbol_id)) {
      const size_t dl_symbol_id = cfg_->Frame().GetDLSymbolIdx(pkt->symbol_id_);
      auto& dataset = datasets_.at(1);
      ///If the frame id is > than the current 0 indexed dimension then we need to extend
      if (frame_id >= dataset.second.at(0)) {
        dataset.second.at(0) = dataset.second.at(0) + kFrameInc;
        RtAssert(dataset.second.at(0) > frame_id,
                 "Frame ID must be less than extended dimension");
        hdf5_->ExtendDataset(dataset.first, dataset.second);
      }
      start[kDsDimSymbol] = dl_symbol_id;
      hdf5_->WriteDataset(dataset.first, start, data_chunk_dims_, pkt->data_);
    }
  } /* End else */
  return ret;
}
};  // End namespace Agora_recorder
