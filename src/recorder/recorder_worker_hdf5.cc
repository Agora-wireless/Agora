/**
 * @file recorder_worker_multifile.cc
 * @brief Recorder worker to write to multiple bin files per rx symbol
 */

#include "recorder_worker_hdf5.h"

#include <ctime>

#include "logger.h"
#include "utils.h"
#include "version_config.h"

namespace Agora_recorder {

static constexpr bool kDebugPrint = false;
static const std::string hdf5_filename = "TestOutputWithTx.h5";
static constexpr size_t kMaxFrameInc = 2000;
static constexpr size_t kDsDimSymbol = 2;

RecorderWorkerHDF5::RecorderWorkerHDF5(const Config* in_cfg,
                                       size_t antenna_offset,
                                       size_t num_antennas)
    : RecorderWorker(in_cfg, antenna_offset, num_antennas),
      cfg_(in_cfg),
      antenna_offset_(antenna_offset),
      num_antennas_(num_antennas),
      max_frame_number_(0) {}

RecorderWorkerHDF5::~RecorderWorkerHDF5() { Gc(); }

void RecorderWorkerHDF5::Gc() {
  AGORA_LOG_INFO("RecorderWorkerHDF5::Garbage collect\n");
}

void RecorderWorkerHDF5::Init() {
  AGORA_LOG_INFO("RecorderWorkerHDF5::Creating output HD5F file: %s\n",
                 hdf5_filename.c_str());
  hdf5_ = std::make_unique<Hdf5Lib>(hdf5_filename, "Data");

  hdf5_->write_attribute("SOFTWARE", "AGORA");
  hdf5_->write_attribute("SOFTWARE_VERSION", GetAgoraProjectVersion());

  auto time = std::time(nullptr);
  auto local_time = *std::localtime(&time);

  std::ostringstream oss;
  oss << std::put_time(&local_time, "%d-%m-%Y %H-%M-%S");
  hdf5_->write_attribute("TIME", oss.str());

  // Write Atrributes
  // ******* COMMON ******** //
  // TX/RX Frequencyfile
  hdf5_->write_attribute("FREQ", cfg_->Freq());

  // BW
  hdf5_->write_attribute("RATE", cfg_->Rate());

  // Number of samples for prefix (padding)
  hdf5_->write_attribute("TX_ZERO_PREFIX_LEN", cfg_->OfdmTxZeroPrefix());

  // Number of samples for postfix (padding)
  hdf5_->write_attribute("TX_ZERO_POSTFIX_LEN", cfg_->OfdmTxZeroPostfix());

  // Number of samples on each symbol including prefix and postfix
  hdf5_->write_attribute("SLOT_SAMP_LEN", cfg_->SampsPerSymbol());

  // Size of FFT
  //hdf5_->write_attribute("FFT_SIZE", cfg_->OfdmCaNum());

  // Length of cyclic prefix
  hdf5_->write_attribute("CP_LEN", cfg_->CpLen());

  hdf5_->write_attribute("OFDM_DATA_NUM", cfg_->OfdmDataNum());
  hdf5_->write_attribute("OFDM_DATA_START", cfg_->OfdmDataStart());
  hdf5_->write_attribute("OFDM_DATA_STOP", cfg_->OfdmDataStop());
  hdf5_->write_attribute("OFDM_PILOT_SPACING", cfg_->OfdmPilotSpacing());
  hdf5_->write_attribute("OFDM_CA_NUM", cfg_->OfdmCaNum());

  // Beacon sequence type (string)
  //hdf5_->write_attribute("BEACON_SEQ_TYPE", cfg_->beacon_seq());

  // Pilot sequence type (string)
  //hdf5_->write_attribute("PILOT_SEQ_TYPE", cfg_->pilot_seq());

  // ******* Base Station ******** //
  // Hub ID
  hdf5_->write_attribute("BS_HUB_ID", cfg_->HubId());

  // BS SDR IDs
  hdf5_->write_attribute("BS_SDR_ID", cfg_->RadioId());

  // Number of Base Station Cells
  hdf5_->write_attribute("BS_NUM_CELLS", cfg_->CellId().size());

  // How many RF channels per Iris board are enabled ("single" or "dual")
  hdf5_->write_attribute("BS_CH_PER_RADIO", cfg_->NumChannels());

  // Frame schedule (vec of strings for now, this should change to matrix when we go to multi-cell)
  hdf5_->write_attribute("BS_FRAME_SCHED", cfg_->Frame().FrameIdentifier());

  // RX Gain RF channel A
  hdf5_->write_attribute("BS_RX_GAIN_A", cfg_->RxGainA());

  // TX Gain RF channel A
  hdf5_->write_attribute("BS_TX_GAIN_A", cfg_->TxGainA());

  // RX Gain RF channel B
  hdf5_->write_attribute("BS_RX_GAIN_B", cfg_->RxGainB());

  // TX Gain RF channel B
  hdf5_->write_attribute("BS_TX_GAIN_B", cfg_->TxGainB());

  // Beamsweep (true or false)
  hdf5_->write_attribute("BS_BEAMSWEEP", cfg_->Beamsweep() ? 1 : 0);

  // Beacon Antenna
  hdf5_->write_attribute("BS_BEACON_ANT", cfg_->BeaconAnt());

  //If the antennas are non consective this will be an issue.
  //hdf5_->write_attribute("ANT_OFFSET", antenna_offset_);
  //hdf5_->write_attribute("ANT_NUM", num_antennas_);
  //hdf5_->write_attribute("ANT_TOTAL", cfg_->getTotNumAntennas());

  // Number of symbols in a frame
  hdf5_->write_attribute("BS_FRAME_LEN", cfg_->Frame().NumTotalSyms());

  // Number of uplink symbols per frame
  hdf5_->write_attribute("UL_SLOTS", cfg_->Frame().NumULSyms());
  hdf5_->write_attribute("DL_SLOTS", cfg_->Frame().NumDLSyms());
  hdf5_->write_attribute("PILOT_SLOTS", cfg_->Frame().NumPilotSyms());
  //PILOT_NUM
  hdf5_->write_attribute("DL_CAL_SLOTS", cfg_->Frame().NumDLCalSyms());
  hdf5_->write_attribute("UL_CAL_SLOTS", cfg_->Frame().NumULCalSyms());
  hdf5_->write_attribute("UL_PILOT_SLOTS",
                         cfg_->Frame().ClientUlPilotSymbols());
  hdf5_->write_attribute("DL_PILOT_SLOTS",
                         cfg_->Frame().ClientDlPilotSymbols());

  // Reciprocal Calibration Mode
  hdf5_->write_attribute("RECIPROCAL_CALIB",
                         cfg_->Frame().IsRecCalEnabled() ? 1 : 0);

  // ******* Clients ******** //
  // Freq. Domain Pilot symbols
  //std::vector<double> split_vec_pilot_f(2 * cfg_->pilot_sym_f().at(0).size());
  //for (size_t i = 0; i < cfg_->pilot_sym_f().at(0).size(); i++) {
  //  split_vec_pilot_f[2 * i + 0] = cfg_->pilot_sym_f().at(0).at(i);
  //  split_vec_pilot_f[2 * i + 1] = cfg_->pilot_sym_f().at(1).at(i);
  //}
  //hdf5_->write_attribute("OFDM_PILOT_F", split_vec_pilot_f);

  // Time Domain Pilot symbols
  //std::vector<double> split_vec_pilot(2 * cfg_->pilot_sym_t().at(0).size());
  //for (size_t i = 0; i < cfg_->pilot_sym_t().at(0).size(); i++) {
  //  split_vec_pilot[2 * i + 0] = cfg_->pilot_sym_t().at(0).at(i);
  //  split_vec_pilot[2 * i + 1] = cfg_->pilot_sym_t().at(1).at(i);
  //}
  //hdf5_->write_attribute("OFDM_PILOT", split_vec_pilot);

  // Data subcarriers
  //if (cfg_->data_ind().size() > 0)
  //  hdf5_->write_attribute("OFDM_DATA_SC", cfg_->data_ind());

  // Pilot subcarriers (indexes)
  //if (cfg_->pilot_sc_ind().size() > 0)
  //  hdf5_->write_attribute("OFDM_PILOT_SC", cfg_->pilot_sc_ind());
  //if (cfg_->pilot_sc().size() > 0)
  //  hdf5_->write_attribute("OFDM_PILOT_SC_VALS", cfg_->pilot_sc());

  // Number of Client Antennas
  hdf5_->write_attribute("CL_NUM", cfg_->UeAntNum());

  // Data modulation
  //hdf5_->write_attribute("CL_MODULATION", cfg_->cl_data_mod());

  // Client antenna polarization
  //Should loop over all values
  hdf5_->write_attribute("CL_CH_PER_RADIO", cfg_->NumUeChannels());

  // RX Gain RF channel A
  hdf5_->write_attribute("CL_RX_GAIN_A", cfg_->ClientRxGainA());

  // TX Gain RF channel A
  hdf5_->write_attribute("CL_TX_GAIN_A", cfg_->ClientTxGainA());

  // RX Gain RF channel B
  hdf5_->write_attribute("CL_RX_GAIN_B", cfg_->ClientRxGainB());

  // TX Gain RF channel B
  hdf5_->write_attribute("CL_TX_GAIN_B", cfg_->ClientTxGainB());

  // Set of client SDR IDs (vec of strings)
  hdf5_->write_attribute("CL_SDR_ID", cfg_->UeRadioId());
  // ********************* //

  // dataset dimension
  const hsize_t IQ = 2 * cfg_->SampsPerSymbol();
  // recording chunk size, TODO: optimize size
  const std::array<hsize_t, kDsDimsNum> cdims = {1, 1, 1, 1, IQ};

  if (cfg_->Frame().NumBeaconSyms() > 0) {
    // Beacon
    datasets_.push_back("BeaconData");
    const std::array<hsize_t, kDsDimsNum> dims_beacon_data = {
        kMaxFrameInc, cfg_->NumCells(), cfg_->Frame().NumBeaconSyms(),
        num_antennas_, IQ};
    hdf5_->createDataset(datasets_.back(), dims_beacon_data, cdims);
  }

  if (cfg_->Frame().NumDLSyms() > 0) {
    // DL
    datasets_.push_back("DownlinkData");
    std::array<hsize_t, kDsDimsNum> dims_dl_data = {
        kMaxFrameInc, cfg_->NumCells(), cfg_->Frame().NumDLSyms(),
        num_antennas_, IQ};
    hdf5_->createDataset(datasets_.back(), dims_dl_data, cdims);
  }

  //*2 for complex
  const hsize_t tx_data_size = 2 * cfg_->OfdmCaNum();
  const std::array<hsize_t, kDsDimsNum> tx_data_dims = {1, 1, 1, 1,
                                                        tx_data_size};
  //Adding the ground truths as a DataSet
  {
    std::string dataset_name = "TxData";
    datasets_.push_back(dataset_name);
    const std::array<hsize_t, kDsDimsNum> total_dims = {
        1, 1, cfg_->Frame().ClientDlPilotSymbols(), num_antennas_,
        tx_data_size};

    hdf5_->createDataset(datasets_.back(), total_dims, tx_data_dims,
                         H5::PredType::INTEL_F32);
  }

  const hsize_t tx_pilot_size = 2 * cfg_->OfdmDataNum();
  const std::array<hsize_t, kDsDimsNum> tx_pilot_dims = {1, 1, 1, 1,
                                                         tx_pilot_size};
  {  //TXPilot
    std::string dataset_name = "TxPilot";
    datasets_.push_back(dataset_name);
    const std::array<hsize_t, kDsDimsNum> total_dims = {1, 1, 1, num_antennas_,
                                                        tx_pilot_size};

    hdf5_->createDataset(datasets_.back(), total_dims, tx_pilot_dims,
                         H5::PredType::INTEL_F32);
  }

  hdf5_->setTargetPrimaryDimSize(kMaxFrameInc);
  hdf5_->setMaxPrimaryDimSize(cfg_->FramesToTest());
  hdf5_->openDataset();

  {
    const std::string dataset_name = "TxData";
    for (size_t ant = 0; ant < num_antennas_; ant++) {
      for (size_t sym = 0; sym < cfg_->Frame().ClientDlPilotSymbols(); sym++) {
        const std::array<hsize_t, kDsDimsNum> hdfoffset = {0, 0, sym, ant, 0};
        AGORA_LOG_INFO(
            "Attempting to write TxData for Symbol %zu, Antenna %zu total syms "
            "%zu\n",
            sym, ant, cfg_->Frame().ClientDlPilotSymbols());
        hdf5_->writeDataset(
            dataset_name, hdfoffset, tx_data_dims,
            reinterpret_cast<float*>(const_cast<Config*>(cfg_)->DlIqF()[sym] +
                                     (ant * cfg_->OfdmCaNum())));
      }
    }
  }

  {
    const std::string dataset_name = "TxPilot";
    for (size_t ant = 0; ant < num_antennas_; ant++) {
      const std::array<hsize_t, kDsDimsNum> hdfoffset = {0, 0, 0, ant, 0};
      hdf5_->writeDataset(
          dataset_name, hdfoffset, tx_pilot_dims,
          reinterpret_cast<float*>(
              const_cast<Config*>(cfg_)->UeSpecificPilot()[ant]));
    }
  }
}
void RecorderWorkerHDF5::Finalize() {
  hdf5_->closeDataset();
  hdf5_->closeFile();
  hdf5_.reset();
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

  const hsize_t IQ = 2 * cfg_->SampsPerSymbol();
  if (frame_id > cfg_->FramesToTest()) {
    hdf5_->closeDataset();
    AGORA_LOG_ERROR("Closing file due to frame id %zu : %zu max\n", frame_id,
                    cfg_->FramesToTest());
  } else {
    // Update the max frame number.
    // Note that the 'frame_id' might be out of order.
    max_frame_number_ = hdf5_->getTargetPrimaryDimSize();
    if (frame_id >= max_frame_number_) {
      // Open the hdf5 file if we haven't.
      hdf5_->closeDataset();
      hdf5_->openDataset();
      hdf5_->setTargetPrimaryDimSize(max_frame_number_ + kMaxFrameInc);
    }

    const uint32_t antenna_index = ant_id - antenna_offset_;
    std::array<hsize_t, kDsDimsNum> hdfoffset = {frame_id, pkt->cell_id_, 0,
                                                 antenna_index, 0};
    const std::array<hsize_t, kDsDimsNum> count = {1, 1, 1, 1, IQ};
    //Bad way to check for beacon.  But here it is
    if (symbol_id == 0) {
      hdf5_->extendDataset(std::string("BeaconData"), frame_id);
      hdfoffset[kDsDimSymbol] = 0;
      hdf5_->writeDataset(std::string("BeaconData"), hdfoffset, count,
                          pkt->data_);
    } else if (cfg_->IsDownlink(frame_id, symbol_id)) {
      const size_t dl_symbol_id = cfg_->Frame().GetDLSymbolIdx(pkt->symbol_id_);
      hdf5_->extendDataset(std::string("DownlinkData"), frame_id);
      hdfoffset[kDsDimSymbol] = dl_symbol_id;
      hdf5_->writeDataset(std::string("DownlinkData"), hdfoffset, count,
                          pkt->data_);
    }
  } /* End else */
  return ret;
}
};  // End namespace Agora_recorder
