/*
 Copyright (c) 2018-2020, Rice University
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

----------------------------------------------------------------------
 Class to handle writting data to an hdf5 file
---------------------------------------------------------------------
*/

#include "recorder_worker.h"

#include "logger.h"
#include "utils.h"

namespace Agora_recorder {
// pilot dataset size increment
const int RecorderWorker::kConfigPilotExtentStep = 400;
// data dataset size increment
const int RecorderWorker::kConfigDataExtentStep = 400;

#if (DEBUG_PRINT)
const int kDsSim = 5;
#endif

RecorderWorker::RecorderWorker(Config* in_cfg, size_t antenna_offset,
                               size_t num_antennas)
    : cfg_(in_cfg) {
  file_ = nullptr;
  pilot_dataset_ = nullptr;
  noise_dataset_ = nullptr;
  data_dataset_ = nullptr;
  antenna_offset_ = antenna_offset;
  num_antennas_ = num_antennas;
}

RecorderWorker::~RecorderWorker() { gc(); }

void RecorderWorker::gc() {
  MLPD_TRACE("Worker Garbage collect\n");

  if (this->pilot_dataset_ != nullptr) {
    MLPD_TRACE("Pilot Dataset exists during garbage collection\n");
    this->pilot_dataset_->close();
    delete this->pilot_dataset_;
    this->pilot_dataset_ = nullptr;
  }

  if (this->noise_dataset_ != nullptr) {
    MLPD_TRACE("Noise Dataset exists during garbage collection\n");
    this->noise_dataset_->close();
    delete this->noise_dataset_;
    this->noise_dataset_ = nullptr;
  }

  if (this->data_dataset_ != nullptr) {
    MLPD_TRACE("Data dataset exists during garbage collection\n");
    this->data_dataset_->close();
    delete this->data_dataset_;
    this->data_dataset_ = nullptr;
  }

  if (this->file_ != nullptr) {
    MLPD_TRACE("File exists exists during garbage collection\n");
    this->file_->close();
    delete this->file_;
    this->file_ = nullptr;
  }
}

void RecorderWorker::init() {
  unsigned int end_antenna = (this->antenna_offset_ + this->num_antennas_) - 1;

  this->hdf5_name_ = this->cfg_->trace_file();
  size_t found_index = this->hdf5_name_.find_last_of('.');
  std::string append = "_" + std::to_string(this->antenna_offset_) + "_" +
                       std::to_string(end_antenna);
  this->hdf5_name_.insert(found_index, append);

  if (this->initHDF5() < 0) {
    throw std::runtime_error("Could not init the output file");
  }
  this->openHDF5();
}

void RecorderWorker::finalize() {
  this->closeHDF5();
  this->finishHDF5();
}

void RecorderWorker::finishHDF5() {
  MLPD_TRACE("Finish HD5F file\n");
  if (this->file_ != nullptr) {
    MLPD_TRACE("Deleting the file ptr for: %s\n", hdf5_name_.c_str());
    delete this->file_;
    this->file_ = nullptr;
  }
}

static void write_attribute(H5::Group& g, const char name[], double val) {
  hsize_t dims[] = {1};
  H5::DataSpace attr_ds = H5::DataSpace(1, dims);
  H5::Attribute att =
      g.createAttribute(name, H5::PredType::NATIVE_DOUBLE, attr_ds);
  att.write(H5::PredType::NATIVE_DOUBLE, &val);
}

static void write_attribute(H5::Group& g, const char name[],
                            const std::vector<double>& val) {
  size_t size = val.size();
  hsize_t dims[] = {size};
  H5::DataSpace attr_ds = H5::DataSpace(1, dims);
  H5::Attribute att =
      g.createAttribute(name, H5::PredType::NATIVE_DOUBLE, attr_ds);
  att.write(H5::PredType::NATIVE_DOUBLE, &val[0]);
}

static void write_attribute(H5::Group& g, const char name[],
                            const std::vector<std::complex<float>>& val) {
  size_t size = val.size();
  hsize_t dims[] = {2 * size};
  H5::DataSpace attr_ds = H5::DataSpace(1, dims);
  H5::Attribute att =
      g.createAttribute(name, H5::PredType::NATIVE_DOUBLE, attr_ds);
  double val_pair[2 * size];
  for (size_t j = 0; j < size; j++) {
    val_pair[2 * j + 0] = std::real(val[j]);
    val_pair[2 * j + 1] = std::imag(val[j]);
  }
  att.write(H5::PredType::NATIVE_DOUBLE, &val_pair[0]);
}

static void write_attribute(H5::Group& g, const char name[], size_t val) {
  hsize_t dims[] = {1};
  H5::DataSpace attr_ds = H5::DataSpace(1, dims);
  H5::Attribute att = g.createAttribute(name, H5::PredType::STD_U32BE, attr_ds);
  uint32_t val_uint = val;
  att.write(H5::PredType::NATIVE_UINT, &val_uint);
}

static void write_attribute(H5::Group& g, const char name[], int val) {
  hsize_t dims[] = {1};
  H5::DataSpace attr_ds = H5::DataSpace(1, dims);
  H5::Attribute att = g.createAttribute(name, H5::PredType::STD_I32BE, attr_ds);
  att.write(H5::PredType::NATIVE_INT, &val);
}

static void write_attribute(H5::Group& g, const char name[],
                            const std::vector<size_t>& val) {
  size_t size = val.size();
  hsize_t dims[] = {size};
  H5::DataSpace attr_ds = H5::DataSpace(1, dims);
  H5::Attribute att = g.createAttribute(name, H5::PredType::STD_U32BE, attr_ds);
  std::vector<uint32_t> val_uint;
  for (unsigned long i : val)
    val_uint.push_back((uint32_t)i);
  att.write(H5::PredType::NATIVE_UINT, &val_uint[0]);
}

static void write_attribute(H5::Group& g, const char name[],
                            const std::string& val) {
  hsize_t dims[] = {1};
  H5::DataSpace attr_ds = H5::DataSpace(1, dims);
  H5::StrType strdatatype(H5::PredType::C_S1,
                          H5T_VARIABLE);  // of variable length characters
  H5::Attribute att = g.createAttribute(name, strdatatype, attr_ds);
  att.write(strdatatype, val);
}

static void write_attribute(H5::Group& g, const char name[],
                            const std::vector<std::string>& val) {
  if (val.empty()) return;
  size_t size = val.size();
  H5::StrType strdatatype(H5::PredType::C_S1,
                          H5T_VARIABLE);  // of variable length characters
  hsize_t dims[] = {size};
  H5::DataSpace attr_ds = H5::DataSpace(1, dims);
  H5::Attribute att = g.createAttribute(name, strdatatype, attr_ds);
  const char* cStrArray[size];

  for (size_t i = 0; i < size; ++i) cStrArray[i] = val[i].c_str();
  att.write(strdatatype, cStrArray);
}

enum {
  kDsFrameNumber,
  kDsNumCells,
  kDsSymsPerFrame,
  kDsNumAntennas,
  kDsPkgDataLen,
  kDsDim
};
typedef hsize_t DataspaceIndex[kDsDim];

herr_t RecorderWorker::initHDF5() {
  MLPD_INFO("Creating output HD5F file: %s\n", this->hdf5_name_.c_str());

  // dataset dimension
  hsize_t IQ = 2 * this->cfg_->SampsPerSymbol();
  DataspaceIndex cdims = {1, 1, 1, 1,
                          IQ};  // pilot chunk size, TODO: optimize size
  this->frame_number_pilot_ = MAX_FRAME_INC;
  // pilots
  DataspaceIndex dims_pilot = {
      this->frame_number_pilot_, this->cfg_->NumCells(),
      this->cfg_->pilot_syms_per_frame(), this->num_antennas_, IQ};
  DataspaceIndex max_dims_pilot = {H5S_UNLIMITED, this->cfg_->NumCells(),
                                   this->cfg_->pilot_syms_per_frame(),
                                   this->num_antennas_, IQ};
  // noise
  this->frame_number_noise_ = MAX_FRAME_INC;
  DataspaceIndex dims_noise = {
      this->frame_number_noise_, this->cfg_->NumCells(),
      this->cfg_->noise_syms_per_frame(), this->num_antennas_, IQ};
  DataspaceIndex max_dims_noise = {H5S_UNLIMITED, this->cfg_->NumCells(),
                                   this->cfg_->noise_syms_per_frame(),
                                   this->num_antennas_, IQ};
  // data
  this->frame_number_data_ = MAX_FRAME_INC;
  DataspaceIndex dims_data = {this->frame_number_data_, this->cfg_->NumCells(),
                              this->cfg_->ul_syms_per_frame(),
                              this->num_antennas_, IQ};
  DataspaceIndex max_dims_data = {H5S_UNLIMITED, this->cfg_->NumCells(),
                                  this->cfg_->ul_syms_per_frame(),
                                  this->num_antennas_, IQ};

  try {
    H5::Exception::dontPrint();

    this->file_ = new H5::H5File(this->hdf5_name_, H5F_ACC_TRUNC);
    auto mainGroup = this->file_->createGroup("/Data");
    this->pilot_prop_.setChunk(kDsDim, cdims);

    H5::DataSpace pilot_dataspace(kDsDim, dims_pilot, max_dims_pilot);
    this->file_->createDataSet("/Data/Pilot_Samples", H5::PredType::STD_I16BE,
                               pilot_dataspace, this->pilot_prop_);

    // ******* COMMON ******** //
    // TX/RX Frequencyfile
    write_attribute(mainGroup, "FREQ", this->cfg_->Freq());

    // BW
    write_attribute(mainGroup, "RATE", this->cfg_->Rate());

    // Number of samples on each symbol (excluding prefix/postfix)
    write_attribute(mainGroup, "SYMBOL_LEN_NO_PAD",
                    this->cfg_->subframe_size());

    // Number of samples for prefix (padding)
    write_attribute(mainGroup, "PREFIX_LEN", this->cfg_->prefix());

    // Number of samples for postfix (padding)
    write_attribute(mainGroup, "POSTFIX_LEN", this->cfg_->postfix());

    // Number of samples on each symbol including prefix and postfix
    write_attribute(mainGroup, "SYMBOL_LEN", this->cfg_->SampsPerSymbol());

    // Size of FFT
    write_attribute(mainGroup, "FFT_SIZE", this->cfg_->fft_size());

    // Number of data subcarriers in ofdm symbols
    write_attribute(mainGroup, "DATA_SUBCARRIER_NUM",
                    this->cfg_->symbol_data_subcarrier_num());

    // Length of cyclic prefix
    write_attribute(mainGroup, "CP_LEN", this->cfg_->cp_size());

    // Beacon sequence type (string)
    write_attribute(mainGroup, "BEACON_SEQ_TYPE", this->cfg_->BeaconLen());

    // Pilot sequence type (string)
    write_attribute(mainGroup, "PILOT_SEQ_TYPE", this->cfg_->pilot_seq());

    // ******* Base Station ******** //
    // Hub IDs (vec of strings)
    write_attribute(mainGroup, "BS_HUB_ID", this->cfg_->HubIds());

    // BS SDR IDs
    // *** first, how many boards in each cell? ***
    std::vector<std::string> bs_sdr_num_per_cell(
        this->cfg_->bs_sdr_ids().size());
    for (size_t i = 0; i < bs_sdr_num_per_cell.size(); ++i) {
      bs_sdr_num_per_cell[i] =
          std::to_string(this->cfg_->bs_sdr_ids().at(i).size());
    }
    write_attribute(mainGroup, "BS_SDR_NUM_PER_CELL", bs_sdr_num_per_cell);

    // *** second, reshape matrix into vector ***
    std::vector<std::string> bs_sdr_id;
    for (auto&& v : this->cfg_->bs_sdr_ids()) {
      bs_sdr_id.insert(bs_sdr_id.end(), v.begin(), v.end());
    }
    write_attribute(mainGroup, "BS_SDR_ID", bs_sdr_id);

    // Number of Base Station Cells
    write_attribute(mainGroup, "BS_NUM_CELLS", this->cfg_->num_cells());

    // How many RF channels per Iris board are enabled ("single" or "dual")
    write_attribute(mainGroup, "BS_CH_PER_RADIO",
                    this->cfg_->bs_channel().length());

    // Frame schedule (vec of strings for now, this should change to matrix when
    // we go to multi-cell)
    write_attribute(mainGroup, "BS_FRAME_SCHED", this->cfg_->frames());

    // RX Gain RF channel A
    write_attribute(mainGroup, "BS_RX_GAIN_A", this->cfg_->rx_gain().at(0));

    // TX Gain RF channel A
    write_attribute(mainGroup, "BS_TX_GAIN_A", this->cfg_->tx_gain().at(0));

    // RX Gain RF channel B
    write_attribute(mainGroup, "BS_RX_GAIN_B", this->cfg_->rx_gain().at(1));

    // TX Gain RF channel B
    write_attribute(mainGroup, "BS_TX_GAIN_B", this->cfg_->tx_gain().at(1));

    // Beamsweep (true or false)
    write_attribute(mainGroup, "BS_BEAMSWEEP",
                    this->cfg_->beam_sweep() ? 1 : 0);

    // Beacon Antenna
    write_attribute(mainGroup, "BS_BEACON_ANT", this->cfg_->beacon_ant());

    // Number of antennas on Base Station (per cell)
    std::vector<std::string> bs_ant_num_per_cell(
        this->cfg_->bs_sdr_ids().size());
    for (size_t i = 0; i < bs_ant_num_per_cell.size(); ++i) {
      bs_ant_num_per_cell[i] =
          std::to_string(this->cfg_->bs_sdr_ids().at(i).size() *
                         this->cfg_->bs_channel().length());
    }
    write_attribute(mainGroup, "BS_ANT_NUM_PER_CELL", bs_ant_num_per_cell);

    // If the antennas are non consective this will be an issue.
    write_attribute(mainGroup, "ANT_OFFSET", this->antenna_offset_);
    write_attribute(mainGroup, "ANT_NUM", this->num_antennas_);
    write_attribute(mainGroup, "ANT_TOTAL", this->cfg_->getTotNumAntennas());

    // Number of symbols in a frame
    write_attribute(mainGroup, "BS_FRAME_LEN", this->cfg_->symbols_per_frame());

    // Number of uplink symbols per frame
    write_attribute(mainGroup, "UL_SYMS", this->cfg_->ul_syms_per_frame());

    // Reciprocal Calibration Mode
    write_attribute(mainGroup, "RECIPROCAL_CALIB",
                    this->cfg_->reciprocal_calib() ? 1 : 0);

    // ******* Clients ******** //
    // Freq. Domain Pilot symbols
    std::vector<double> split_vec_pilot_f(
        2 * this->cfg_->pilot_sym_f().at(0).size());
    for (size_t i = 0; i < this->cfg_->pilot_sym_f().at(0).size(); i++) {
      split_vec_pilot_f[2 * i + 0] = this->cfg_->pilot_sym_f().at(0).at(i);
      split_vec_pilot_f[2 * i + 1] = this->cfg_->pilot_sym_f().at(1).at(i);
    }
    write_attribute(mainGroup, "OFDM_PILOT_F", split_vec_pilot_f);

    // Time Domain Pilot symbols
    std::vector<double> split_vec_pilot(2 *
                                        this->cfg_->pilot_sym().at(0).size());
    for (size_t i = 0; i < this->cfg_->pilot_sym().at(0).size(); i++) {
      split_vec_pilot[2 * i + 0] = this->cfg_->pilot_sym().at(0).at(i);
      split_vec_pilot[2 * i + 1] = this->cfg_->pilot_sym().at(1).at(i);
    }
    write_attribute(mainGroup, "OFDM_PILOT", split_vec_pilot);

    // Number of Pilots
    write_attribute(mainGroup, "PILOT_NUM", this->cfg_->pilot_syms_per_frame());

    // Number of Client Antennas
    write_attribute(mainGroup, "CL_NUM", this->cfg_->num_cl_antennas());

    // Data modulation
    write_attribute(mainGroup, "CL_MODULATION", this->cfg_->data_mod());

    if (this->cfg_->client_present() == true) {
      // Client antenna polarization
      write_attribute(mainGroup, "CL_CH_PER_RADIO", this->cfg_->cl_sdr_ch());

      // Client AGC enable flag
      write_attribute(mainGroup, "CL_AGC_EN", this->cfg_->cl_agc_en() ? 1 : 0);

      // RX Gain RF channel A
      write_attribute(mainGroup, "CL_RX_GAIN_A",
                      this->cfg_->cl_rxgain_vec().at(0));

      // TX Gain RF channel A
      write_attribute(mainGroup, "CL_TX_GAIN_A",
                      this->cfg_->cl_txgain_vec().at(0));

      // RX Gain RF channel B
      write_attribute(mainGroup, "CL_RX_GAIN_B",
                      this->cfg_->cl_rxgain_vec().at(1));

      // TX Gain RF channel B
      write_attribute(mainGroup, "CL_TX_GAIN_B",
                      this->cfg_->cl_txgain_vec().at(1));

      // Number of frames for UL data recorded in bit source files
      write_attribute(mainGroup, "UL_DATA_FRAME_NUM",
                      this->cfg_->ul_data_frame_num());

      // Names of Files including uplink tx frequency-domain data
      if (this->cfg_->tx_fd_data_files().size() > 0) {
        write_attribute(mainGroup, "TX_FD_DATA_FILENAMES",
                        this->cfg_->tx_fd_data_files());
      }

      // Client frame schedule (vec of strings)
      write_attribute(mainGroup, "CL_FRAME_SCHED", this->cfg_->cl_frames());

      // Set of client SDR IDs (vec of strings)
      write_attribute(mainGroup, "CL_SDR_ID", this->cfg_->cl_sdr_ids());
    }

    if (this->cfg_->ul_data_sym_present()) {
      // Data subcarriers
      if (this->cfg_->data_ind().size() > 0)
        write_attribute(mainGroup, "OFDM_DATA_SC", this->cfg_->data_ind());

      // Pilot subcarriers (indexes)
      if (this->cfg_->pilot_sc_ind().size() > 0)
        write_attribute(mainGroup, "OFDM_PILOT_SC", this->cfg_->pilot_sc_ind());
      if (this->cfg_->pilot_sc().size() > 0)
        write_attribute(mainGroup, "OFDM_PILOT_SC_VALS",
                        this->cfg_->pilot_sc());

      // Freq. Domain Data Symbols
      for (size_t i = 0; i < this->cfg_->txdata_freq_dom().size(); i++) {
        std::string var = std::string("OFDM_DATA_CL") + std::to_string(i);
        write_attribute(mainGroup, var.c_str(),
                        this->cfg_->txdata_freq_dom().at(i));
      }

      // Time Domain Data Symbols
      for (size_t i = 0; i < this->cfg_->txdata_time_dom().size(); i++) {
        std::string var = std::string("OFDM_DATA_TIME_CL") + std::to_string(i);
        write_attribute(mainGroup, var.c_str(),
                        this->cfg_->txdata_time_dom().at(i));
      }
    }
    // ********************* //

    this->pilot_prop_.close();
    if (this->cfg_->noise_syms_per_frame() > 0) {
      H5::DataSpace noise_dataspace(kDsDim, dims_noise, max_dims_noise);
      this->noise_prop_.setChunk(kDsDim, cdims);
      this->file_->createDataSet("/Data/Noise_Samples", H5::PredType::STD_I16BE,
                                 noise_dataspace, this->noise_prop_);
      this->noise_prop_.close();
    }

    if (this->cfg_->ul_syms_per_frame() > 0) {
      H5::DataSpace data_dataspace(kDsDim, dims_data, max_dims_data);
      this->data_prop_.setChunk(kDsDim, cdims);
      this->file_->createDataSet("/Data/UplinkData", H5::PredType::STD_I16BE,
                                 data_dataspace, this->data_prop_);
      this->data_prop_.close();
    }
    this->file_->close();
  }
  // catch failure caused by the H5File operations
  catch (H5::FileIException& error) {
    error.printErrorStack();
    return -1;
  }

  // catch failure caused by the DataSet operations
  catch (H5::DataSetIException& error) {
    error.printErrorStack();
    return -1;
  }

  // catch failure caused by the DataSpace operations
  catch (H5::DataSpaceIException& error) {
    error.printErrorStack();
    return -1;
  }
  this->max_frame_number_ = MAX_FRAME_INC;
  return 0;  // successfully terminated
}

void RecorderWorker::openHDF5() {
  MLPD_TRACE("Open HDF5 file: %s\n", this->hdf5_name_.c_str());
  this->file_->openFile(this->hdf5_name_, H5F_ACC_RDWR);
  assert(this->pilot_dataset_ == nullptr);
  // Get Dataset for pilot and check the shape of it
  this->pilot_dataset_ =
      new H5::DataSet(this->file_->openDataSet("/Data/Pilot_Samples"));

  // Get the dataset's dataspace and creation property list.
  H5::DataSpace pilot_filespace(this->pilot_dataset_->getSpace());
  this->pilot_prop_.copy(this->pilot_dataset_->getCreatePlist());

#if DEBUG_PRINT
  hsize_t IQ = 2 * this->cfg_->samps_per_symbol();
  int cndims_pilot = 0;
  int ndims = pilot_filespace.getSimpleExtentNdims();
  DataspaceIndex dims_pilot = {
      this->frame_number_pilot_, this->cfg_->num_cells(),
      this->cfg_->pilot_syms_per_frame(), this->num_antennas(), IQ};
  if (H5D_CHUNKED == this->pilot_prop_.getLayout())
    cndims_pilot = this->pilot_prop_.getChunk(ndims, dims_pilot);
  using std::cout;
  cout << "dim pilot chunk = " << cndims_pilot << std::endl;
  cout << "New Pilot Dataset Dimension: [";
  for (auto i = 0; i < kDsSim - 1; ++i) cout << dims_pilot[i] << ",";
  cout << dims_pilot[kDsSim - 1] << "]" << std::endl;
#endif
  pilot_filespace.close();
  // Get Dataset for DATA (If Enabled) and check the shape of it
  if (this->cfg_->ul_syms_per_frame() > 0) {
    this->data_dataset_ =
        new H5::DataSet(this->file_->openDataSet("/Data/UplinkData"));

    H5::DataSpace data_filespace(this->data_dataset_->getSpace());
    this->data_prop_.copy(this->data_dataset_->getCreatePlist());

#if DEBUG_PRINT
    int ndims = data_filespace.getSimpleExtentNdims();
    // status_n = data_filespace.getSimpleExtentDims(dims_data);
    int cndims_data = 0;
    DataspaceIndex cdims_data = {1, 1, 1, 1,
                                 IQ};  // data chunk size, TODO: optimize size
    if (H5D_CHUNKED == this->data_prop_.getLayout())
      cndims_data = this->data_prop_.getChunk(ndims, cdims_data);
    cout << "dim data chunk = " << cndims_data << std::endl;
    DataspaceIndex dims_data = {
        this->frame_number_data_, this->cfg_->num_cells(),
        this->cfg_->ul_syms_per_frame(), this->num_antennas(), IQ};
    cout << "New Data Dataset Dimension " << ndims << ",";
    for (auto i = 0; i < kDsSim - 1; ++i) cout << dims_data[i] << ",";
    cout << dims_data[kDsSim - 1] << std::endl;
#endif
    data_filespace.close();
  }

  // Get Dataset for NOISE (If Enabled) and check the shape of it
  if (this->cfg_->noise_syms_per_frame() > 0) {
    this->noise_dataset_ =
        new H5::DataSet(this->file_->openDataSet("/Data/Noise_Samples"));
    H5::DataSpace noise_filespace(this->noise_dataset_->getSpace());
    this->noise_prop_.copy(this->noise_dataset_->getCreatePlist());

#if DEBUG_PRINT
    int ndims = noise_filespace.getSimpleExtentNdims();
    int cndims_noise = 0;
    DataspaceIndex cdims_noise = {1, 1, 1, 1,
                                  IQ};  // data chunk size, TODO: optimize size
    if (H5D_CHUNKED == this->noise_prop_.getLayout())
      cndims_noise = this->noise_prop_.getChunk(ndims, cdims_noise);
    cout << "dim noise chunk = " << cndims_noise << std::endl;
    DataspaceIndex dims_noise = {
        this->frame_number_noise_, this->cfg_->num_cells(),
        this->cfg_->noise_syms_per_frame(), this->antennas_.size(), IQ};
    cout << "New Noise Dataset Dimension " << ndims << ",";
    for (auto i = 0; i < kDsSim - 1; ++i) cout << dims_noise[i] << ",";
    cout << dims_noise[kDsSim - 1] << std::endl;
#endif
    noise_filespace.close();
  }
}

void RecorderWorker::closeHDF5() {
  MLPD_TRACE("Close HD5F file: %s\n", this->hdf5_name_.c_str());

  if (this->file_ == nullptr) {
    MLPD_WARN("File does not exist while calling close: %s\n",
              this->hdf5_name_.c_str());
  } else {
    unsigned frame_number = this->max_frame_number_;
    hsize_t IQ = 2 * this->cfg_->samps_per_symbol();

    assert(this->pilot_dataset_ != nullptr);
    // Resize Pilot Dataset
    this->frame_number_pilot_ = frame_number;
    DataspaceIndex dims_pilot = {
        this->frame_number_pilot_, this->cfg_->num_cells(),
        this->cfg_->pilot_syms_per_frame(), this->num_antennas_, IQ};
    this->pilot_dataset_->extend(dims_pilot);
    this->pilot_prop_.close();
    this->pilot_dataset_->close();
    delete this->pilot_dataset_;
    this->pilot_dataset_ = nullptr;

    // Resize Data Dataset (If Needed)
    if (this->cfg_->ul_syms_per_frame() > 0) {
      assert(this->data_dataset_ != nullptr);
      this->frame_number_data_ = frame_number;
      DataspaceIndex dims_data = {
          this->frame_number_data_, this->cfg_->num_cells(),
          this->cfg_->ul_syms_per_frame(), this->num_antennas_, IQ};
      this->data_dataset_->extend(dims_data);
      this->data_prop_.close();
      this->data_dataset_->close();
      delete this->data_dataset_;
      this->data_dataset_ = nullptr;
    }

    // Resize Noise Dataset (If Needed)
    if (this->cfg_->noise_syms_per_frame() > 0) {
      assert(this->noise_dataset_ != nullptr);
      this->frame_number_noise_ = frame_number;
      DataspaceIndex dims_noise = {
          this->frame_number_noise_, this->cfg_->num_cells(),
          this->cfg_->noise_syms_per_frame(), this->num_antennas_, IQ};
      this->noise_dataset_->extend(dims_noise);
      this->noise_prop_.close();
      this->noise_dataset_->close();
      delete this->noise_dataset_;
      this->noise_dataset_ = nullptr;
    }

    this->file_->close();
    MLPD_INFO("Saving HD5F: %d frames saved on CPU %d\n", frame_number,
              sched_getcpu());
  }
}

herr_t RecorderWorker::record(int tid, Packet* pkg) {
  (void)tid;
  /* TODO: remove TEMP check */
  size_t end_antenna = (this->antenna_offset_ + this->num_antennas_) - 1;

  if ((pkg->ant_id < this->antenna_offset_) || (pkg->ant_id > end_antenna)) {
    MLPD_ERROR("Antenna id is not within range of this recorder %d, %zu:%zu",
               pkg->ant_id, this->antenna_offset_, end_antenna);
  }
  assert((pkg->ant_id >= this->antenna_offset_) &&
         (pkg->ant_id <= end_antenna));

  herr_t ret = 0;

  // Generates a ton of messages
  // MLPD_TRACE( "Tid: %d -- frame_id %u, antenna: %u\n", tid, pkg->frame_id,
  // pkg->ant_id);

#if DEBUG_PRINT
  printf(
      "record            frame %d, symbol %d, cell %d, ant %d samples: %d "
      "%d %d %d %d %d %d %d ....\n",
      pkg->frame_id, pkg->symbol_id, pkg->cell_id, pkg->ant_id, pkg->data[1],
      pkg->data[2], pkg->data[3], pkg->data[4], pkg->data[5], pkg->data[6],
      pkg->data[7], pkg->data[8]);
#endif
  hsize_t IQ = 2 * this->cfg_->samps_per_symbol();
  if ((this->cfg_->max_frame()) != 0 &&
      (pkg->frame_id > this->cfg_->max_frame())) {
    closeHDF5();
    MLPD_TRACE("Closing file due to frame id %d : %zu max\n", pkg->frame_id,
               this->cfg_->max_frame());
  } else {
    try {
      H5::Exception::dontPrint();
      // Update the max frame number.
      // Note that the 'frame_id' might be out of order.
      if (pkg->frame_id >= this->max_frame_number_) {
        // Open the hdf5 file if we haven't.
        closeHDF5();
        openHDF5();
        this->max_frame_number_ = this->max_frame_number_ + MAX_FRAME_INC;
      }

      uint32_t antenna_index = pkg->ant_id - this->antenna_offset_;
      DataspaceIndex hdfoffset = {pkg->frame_id, pkg->cell_id, 0, antenna_index,
                                  0};
      if ((this->cfg_->reciprocal_calib() == true) ||
          (this->cfg_->isPilot(pkg->frame_id, pkg->symbol_id) == true)) {
        assert(this->pilot_dataset_ != nullptr);
        // Are we going to extend the dataset?
        if (pkg->frame_id >= this->frame_number_pilot_) {
          this->frame_number_pilot_ += kConfigPilotExtentStep;
          if (this->cfg_->max_frame() != 0) {
            this->frame_number_pilot_ = std::min(this->frame_number_pilot_,
                                                 this->cfg_->max_frame() + 1);
          }
          DataspaceIndex dims_pilot = {
              this->frame_number_pilot_, this->cfg_->num_cells(),
              this->cfg_->pilot_syms_per_frame(), this->num_antennas_, IQ};
          this->pilot_dataset_->extend(dims_pilot);
#if DEBUG_PRINT
          std::cout << "FrameId " << pkg->frame_id << ", (Pilot) Extent to "
                    << this->frame_number_pilot_ << " Frames" << std::endl;
#endif
        }
        hdfoffset[kDsSymsPerFrame] =
            this->cfg_->getClientId(pkg->frame_id, pkg->symbol_id);

        // Select a hyperslab in extended portion of the dataset
        H5::DataSpace pilot_filespace(this->pilot_dataset_->getSpace());
        DataspaceIndex count = {1, 1, 1, 1, IQ};
        pilot_filespace.selectHyperslab(H5S_SELECT_SET, count, hdfoffset);
        // define memory space
        H5::DataSpace pilot_memspace(kDsDim, count, nullptr);
        this->pilot_dataset_->write(pkg->data, H5::PredType::NATIVE_INT16,
                                    pilot_memspace, pilot_filespace);
        pilot_filespace.close();
      } else if (this->cfg_->isData(pkg->frame_id, pkg->symbol_id) == true) {
        assert(this->data_dataset_ != nullptr);
        // Are we going to extend the dataset?
        if (pkg->frame_id >= this->frame_number_data_) {
          this->frame_number_data_ += kConfigDataExtentStep;
          if (this->cfg_->max_frame() != 0)
            this->frame_number_data_ =
                std::min(this->frame_number_data_, this->cfg_->max_frame() + 1);
          DataspaceIndex dims_data = {
              this->frame_number_data_, this->cfg_->num_cells(),
              this->cfg_->ul_syms_per_frame(), this->num_antennas_, IQ};
          this->data_dataset_->extend(dims_data);
#if DEBUG_PRINT
          std::cout << "FrameId " << pkg->frame_id << ", (Data) Extent to "
                    << this->frame_number_data_ << " Frames" << std::endl;
#endif
        }
        hdfoffset[kDsSymsPerFrame] =
            this->cfg_->getUlSFIndex(pkg->frame_id, pkg->symbol_id);
        // Select a hyperslab in extended portion of the dataset
        H5::DataSpace data_filespace(this->data_dataset_->getSpace());
        DataspaceIndex count = {1, 1, 1, 1, IQ};
        data_filespace.selectHyperslab(H5S_SELECT_SET, count, hdfoffset);
        // define memory space
        H5::DataSpace data_memspace(kDsDim, count, nullptr);
        this->data_dataset_->write(pkg->data, H5::PredType::NATIVE_INT16,
                                   data_memspace, data_filespace);
        data_filespace.close();

      } else if (this->cfg_->isNoise(pkg->frame_id, pkg->symbol_id) == true) {
        assert(this->noise_dataset_ != nullptr);
        // Are we going to extend the dataset?
        if (pkg->frame_id >= this->frame_number_noise_) {
          this->frame_number_noise_ += kConfigDataExtentStep;
          if (this->cfg_->max_frame() != 0)
            this->frame_number_noise_ = std::min(this->frame_number_noise_,
                                                 this->cfg_->max_frame() + 1);
          DataspaceIndex dims_noise = {
              this->frame_number_noise_, this->cfg_->num_cells(),
              this->cfg_->noise_syms_per_frame(), this->num_antennas_, IQ};
          this->noise_dataset_->extend(dims_noise);
#if DEBUG_PRINT
          std::cout << "FrameId " << pkg->frame_id << ", (Noise) Extent to "
                    << this->frame_number_noise_ << " Frames" << std::endl;
#endif
        }
        hdfoffset[kDsSymsPerFrame] =
            this->cfg_->getNoiseSFIndex(pkg->frame_id, pkg->symbol_id);
        // Select a hyperslab in extended portion of the dataset
        H5::DataSpace noise_filespace(this->noise_dataset_->getSpace());
        DataspaceIndex count = {1, 1, 1, 1, IQ};
        noise_filespace.selectHyperslab(H5S_SELECT_SET, count, hdfoffset);
        // define memory space
        H5::DataSpace noise_memspace(kDsDim, count, nullptr);
        this->noise_dataset_->write(pkg->data, H5::PredType::NATIVE_INT16,
                                    noise_memspace, noise_filespace);
        noise_filespace.close();
      }
    }
    // catch failure caused by the H5File operations
    catch (H5::FileIException& error) {
      error.printErrorStack();
      ret = -1;
      throw;
    }
    // catch failure caused by the DataSet operations
    catch (H5::DataSetIException& error) {
      error.printErrorStack();

      MLPD_WARN(
          "DataSet: Failed to record pilots from frame: %d , UE %d "
          "antenna %d IQ %llu\n",
          pkg->frame_id, this->cfg_->getClientId(pkg->frame_id, pkg->symbol_id),
          pkg->ant_id, IQ);

      DataspaceIndex dims_pilot = {
          this->frame_number_pilot_, this->cfg_->num_cells(),
          this->cfg_->pilot_syms_per_frame(), this->num_antennas_, IQ};
      int ndims = this->data_dataset_->getSpace().getSimpleExtentNdims();

      std::stringstream ss;
      ss.str(std::string());
      ss << "Dataset Dimension is: " << ndims;
      for (auto i = 0; i < (kDsDim - 1); ++i) {
        ss << dims_pilot[i] << ",";
      }
      ss << dims_pilot[kDsDim - 1];
      MLPD_TRACE("%s", ss.str().c_str());
      ret = -1;
      throw;
    }
    // catch failure caused by the DataSpace operations
    catch (H5::DataSpaceIException& error) {
      error.printErrorStack();
      ret = -1;
      throw;
    }
  } /* End else */
  return ret;
}
};  // End namespace Agora_recorder
