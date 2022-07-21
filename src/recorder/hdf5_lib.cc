/*
 Copyright (c) 2018-2022, Rice University
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

----------------------------------------------------------------------
 Implementation of Hdf5Lib API Class
---------------------------------------------------------------------
*/

#include "hdf5_lib.h"

#include <cassert>

#include "logger.h"
#include "utils.h"

namespace Agora_recorder {
static constexpr bool kPrintDataSetInfo = true;
static constexpr int kDsExtendStep = 400;
static constexpr size_t kDExtendDimIdx = 0;

Hdf5Lib::Hdf5Lib(H5std_string hdf5_name, H5std_string group_name)
    : hdf5_name_(hdf5_name), group_name_(group_name) {
  AGORA_LOG_INFO("Creating output HD5F file: %s\n", hdf5_name_.c_str());
  file_ = std::make_unique<H5::H5File>(hdf5_name_, H5F_ACC_TRUNC);
  file_->close();
  group_ = file_->createGroup("/" + group_name_);
  target_prim_dim_size_ = 1;
  max_prim_dim_size_ = 0;
}

Hdf5Lib::~Hdf5Lib() {
  for (size_t i = 0; i < dataset_str_.size(); i++) {
    removeDataset(dataset_str_.at(i));
  }
  closeFile();
}

void Hdf5Lib::closeFile() {
  if (file_ != nullptr) {
    AGORA_LOG_INFO("File exists exists during garbage collection\n");
    file_->close();
    file_.reset();
  }
}

///Dataset -> Data Type + Dataspace
int Hdf5Lib::createDataset(H5std_string dataset_name,
                           std::array<hsize_t, kDsDimsNum> tot_dims,
                           std::array<hsize_t, kDsDimsNum> chunk_dims,
                           H5::PredType type) {
  std::string ds_name("/" + group_name_ + "/" + dataset_name);
  std::array<hsize_t, kDsDimsNum> max_ds_dims = tot_dims;
  //max_ds_dims.at(kDExtendDimIdx) = H5S_UNLIMITED;
  H5::DataSpace ds_dataspace(kDsDimsNum, tot_dims.data(), max_ds_dims.data());
  H5::DSetCreatPropList ds_prop;
  try {
    H5::Exception::dontPrint();
    ds_prop.setChunk(kDsDimsNum, chunk_dims.data());
    //ds_prop.setFillValue(type, &fill_val);
    auto new_dataset =
        file_->createDataSet(ds_name, type, ds_dataspace, ds_prop);
    ds_prop.close();
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
  dataset_str_.push_back(dataset_name);
  prop_list_.push_back(ds_prop);
  dataspace_.push_back(ds_dataspace);
  dims_.push_back(tot_dims);
  const size_t map_size = ds_name_id_.size();
  ds_name_id_[dataset_name] = map_size;
  return 0;
}

void Hdf5Lib::openDataset() {
  AGORA_LOG_INFO("Open HDF5 file Dataset: %s\n", hdf5_name_.c_str());
  ///Probably not necessary to open the file here
  file_->openFile(hdf5_name_, H5F_ACC_RDWR);
  for (size_t i = 0; i < dataset_str_.size(); i++) {
    const std::string ds_name("/" + group_name_ + "/" + dataset_str_.at(i));
    try {
      datasets_.emplace_back(
          std::make_unique<H5::DataSet>(file_->openDataSet(ds_name)));
      H5::DataSpace filespace(datasets_.at(i)->getSpace());
      prop_list_.at(i).copy(datasets_.at(i)->getCreatePlist());
      if (kPrintDataSetInfo) {
        const int ndims = filespace.getSimpleExtentNdims();
        int cndims = 0;
        if (H5D_CHUNKED == prop_list_.at(i).getLayout()) {
          cndims = prop_list_.at(i).getChunk(ndims, dims_.at(i).data());
        }
        std::cout << "dim " + dataset_str_.at(i) + " chunk = " << cndims
                  << std::endl
                  << "New " + dataset_str_.at(i) + " Dataset Dimension: [";
        for (size_t j = 0; j < kDsDimsNum - 1; ++j) {
          std::cout << dims_.at(i).at(j) << ",";
        }
        std::cout << dims_.at(i).at(kDsDimsNum - 1) << "]" << std::endl;
      }
      filespace.close();
    }
    // catch failure caused by the H5File operations
    catch (H5::FileIException& error) {
      error.printErrorStack();
      throw;
    }
  }
}

void Hdf5Lib::removeDataset(std::string dataset_name) {
  const std::string ds_name("/" + group_name_ + "/" + dataset_name);
  const size_t ds_id = ds_name_id_[dataset_name];
  if (datasets_.at(ds_id) != nullptr) {
    AGORA_LOG_INFO("%s - Removing Dataset\n", dataset_str_.at(ds_id).c_str());
    datasets_.at(ds_id)->close();
    datasets_.at(ds_id).reset();
  }
}

void Hdf5Lib::closeDataset() {
  AGORA_LOG_INFO("Close HD5F file dataset: %s\n", hdf5_name_.c_str());

  if (file_ != nullptr) {
    for (size_t i = 0; i < dataset_str_.size(); i++) {
      //assert(datasets_.at(i) != nullptr);
      try {
        H5::Exception::dontPrint();
        extendDataset(dataset_str_.at(i), target_prim_dim_size_);
        prop_list_.at(i).close();
        datasets_.at(i)->close();
      }
      // catch failure caused by the H5File operations
      catch (H5::FileIException& error) {
        error.printErrorStack();
        throw;
      }
      datasets_.at(i).reset();
    }
    file_->close();
    file_.reset();
    AGORA_LOG_INFO("Saving HD5F: %llu frames saved on CPU %d\n",
                   target_prim_dim_size_, sched_getcpu());
  }
}

bool Hdf5Lib::extendDataset(std::string dataset_name, size_t prim_dim_size) {
  const std::string ds_name("/" + group_name_ + "/" + dataset_name);
  const size_t ds_id = ds_name_id_[dataset_name];
  auto& current_size = dims_.at(ds_id).at(kDExtendDimIdx);
  if (kPrintDataSetInfo) {
    std::cout << dataset_name << " current dimension size: " << current_size;
  }

  if (current_size <= prim_dim_size) {
    /// Extend to nearest kDsExtendStep multiple
    //hsize_t new_dim_size =
    //    prim_dim_size + (kDsExtendStep - (prim_dim_size % kDsExtendStep));
    hsize_t new_dim_size = current_size + kDsExtendStep;

    if (max_prim_dim_size_ != 0) {
      new_dim_size = std::min(new_dim_size, max_prim_dim_size_);
    }

    if (new_dim_size > current_size) {
      current_size = new_dim_size;
      datasets_.at(ds_id)->extend(dims_.at(ds_id).data());
      if (kPrintDataSetInfo) {
        std::cout << " requested: " << prim_dim_size
                  << " extended: " << dims_.at(ds_id).at(kDExtendDimIdx)
                  << std::endl;
      }
    }
    return true;
  }
  return false;
}

herr_t Hdf5Lib::writeDataset(std::string dataset_name,
                             std::array<hsize_t, kDsDimsNum> target_id,
                             std::array<hsize_t, kDsDimsNum> wrt_dim,
                             const short* wrt_data) {
  const std::string ds_name("/" + group_name_ + "/" + dataset_name);
  const size_t ds_id = ds_name_id_[dataset_name];
  herr_t ret = 0;
  // Select a hyperslab in extended portion of the dataset
  try {
    H5::Exception::dontPrint();
    H5::DataSpace filespace(datasets_.at(ds_id)->getSpace());
    filespace.selectHyperslab(H5S_SELECT_SET, wrt_dim.data(), target_id.data());
    // define memory space
    H5::DataSpace memspace(kDsDimsNum, wrt_dim.data(), NULL);
    datasets_.at(ds_id)->write(wrt_data, H5::PredType::NATIVE_INT16, memspace,
                               filespace);
    filespace.close();
  }
  // catch failure caused by the DataSet operations
  catch (H5::DataSetIException& error) {
    error.printErrorStack();

    AGORA_LOG_WARN(
        "DataSet: Failed to write to dataset at primary dim index: %llu\n",
        target_id.at(kDExtendDimIdx));

    const int ndims = datasets_.at(ds_id)->getSpace().getSimpleExtentNdims();

    std::stringstream ss;
    ss.str(std::string());
    ss << "\nDataset Dimension is: " << ndims;
    for (size_t i = 0; i < (kDsDimsNum - 1); ++i) {
      ss << dims_.at(ds_id)[i] << ", ";
    }
    ss << dims_.at(ds_id)[kDsDimsNum - 1];
    ss << "\nRequested Write Dimension is: " << ndims;
    for (size_t i = 0; i < (kDsDimsNum - 1); ++i) {
      ss << target_id[i] << ", ";
    }
    ss << target_id.at(kDsDimsNum - 1);
    AGORA_LOG_TRACE("%s\n", ss.str().c_str());
    ret = -1;
    throw;
  }
  // catch failure caused by the DataSpace operations
  catch (H5::DataSpaceIException& error) {
    error.printErrorStack();
    ret = -1;
    throw;
  }
  return ret;
}

herr_t Hdf5Lib::writeDataset(std::string dataset_name,
                             std::array<hsize_t, kDsDimsNum> target_id,
                             std::array<hsize_t, kDsDimsNum> wrt_dim,
                             const float* wrt_data) {
  const std::string ds_name("/" + group_name_ + "/" + dataset_name);
  const size_t ds_id = ds_name_id_[dataset_name];
  herr_t ret = 0;
  // Select a hyperslab in extended portion of the dataset
  try {
    H5::Exception::dontPrint();
    H5::DataSpace filespace(datasets_.at(ds_id)->getSpace());
    filespace.selectHyperslab(H5S_SELECT_SET, wrt_dim.data(), target_id.data());
    // define memory space
    H5::DataSpace memspace(kDsDimsNum, wrt_dim.data(), NULL);
    datasets_.at(ds_id)->write(wrt_data, H5::PredType::INTEL_F32, memspace,
                               filespace);
    filespace.close();
  }
  // catch failure caused by the DataSet operations
  catch (H5::DataSetIException& error) {
    error.printErrorStack();

    AGORA_LOG_WARN(
        "DataSet: Failed to write to dataset at primary dim index: %llu\n",
        target_id.at(kDExtendDimIdx));

    int ndims = datasets_.at(ds_id)->getSpace().getSimpleExtentNdims();

    std::stringstream ss;
    ss.str(std::string());
    ss << "\nDataset Dimension is: " << ndims;
    for (size_t i = 0; i < (kDsDimsNum - 1); ++i) {
      ss << dims_.at(ds_id)[i] << ", ";
    }
    ss << dims_.at(ds_id)[kDsDimsNum - 1];
    ss << "\nRequested Write Dimension is: " << ndims;
    for (size_t i = 0; i < (kDsDimsNum - 1); ++i) {
      ss << target_id[i] << ", ";
    }
    ss << target_id[kDsDimsNum - 1];
    AGORA_LOG_TRACE("%s\n", ss.str().c_str());
    ret = -1;
    throw;
  }
  // catch failure caused by the DataSpace operations
  catch (H5::DataSpaceIException& error) {
    error.printErrorStack();
    ret = -1;
    throw;
  }
  return ret;
}

std::vector<short> Hdf5Lib::readDataset(
    std::string dataset_name, std::array<hsize_t, kDsDimsNum> target_id,
    std::array<hsize_t, kDsDimsNum> read_dim) {
  std::vector<short> read_data;
  const std::string ds_name("/" + group_name_ + "/" + dataset_name);
  const size_t ds_id = ds_name_id_[dataset_name];
  // Select a hyperslab in extended portion of the dataset
  try {
    H5::Exception::dontPrint();
    H5::DataSpace filespace(datasets_.at(ds_id)->getSpace());
    filespace.selectHyperslab(H5S_SELECT_SET, read_dim.data(),
                              target_id.data());
    // define memory space
    H5::DataSpace memspace(kDsDimsNum, read_dim.data(), NULL);
    read_data.resize(read_dim.at(kDsDimsNum - 1), 0);
    datasets_.at(ds_id)->read(read_data.data(), H5::PredType::NATIVE_INT16,
                              memspace, filespace);
    filespace.close();
  }
  // catch failure caused by the DataSet operations
  catch (H5::DataSetIException& error) {
    error.printErrorStack();

    AGORA_LOG_WARN(
        "DataSet: Failed to write to dataset at primary dim index: %llu",
        target_id.at(kDExtendDimIdx));

    const int ndims = datasets_.at(ds_id)->getSpace().getSimpleExtentNdims();

    std::stringstream ss;
    ss.str(std::string());
    ss << "Dataset Dimension is: " << ndims;
    for (size_t i = 0; i < (kDsDimsNum - 1); ++i) {
      ss << dims_.at(ds_id)[i] << ",";
    }
    ss << dims_.at(ds_id)[kDsDimsNum - 1];
    ss << "Requested Write Dimension is: " << ndims;
    for (size_t i = 0; i < (kDsDimsNum - 1); ++i) {
      ss << target_id[i] << ",";
    }
    ss << target_id[kDsDimsNum - 1];
    AGORA_LOG_TRACE("%s", ss.str().c_str());
    throw;
  }
  // catch failure caused by the DataSpace operations
  catch (H5::DataSpaceIException& error) {
    error.printErrorStack();
    throw;
  }
  return read_data;
}

void Hdf5Lib::write_attribute(const char name[], double val) {
  hsize_t dims[] = {1};
  H5::DataSpace attr_ds = H5::DataSpace(1, dims);
  H5::Attribute att =
      group_.createAttribute(name, H5::PredType::NATIVE_DOUBLE, attr_ds);
  att.write(H5::PredType::NATIVE_DOUBLE, &val);
}

void Hdf5Lib::write_attribute(const char name[],
                              const std::vector<double>& val) {
  size_t size = val.size();
  hsize_t dims[] = {size};
  H5::DataSpace attr_ds = H5::DataSpace(1, dims);
  H5::Attribute att =
      group_.createAttribute(name, H5::PredType::NATIVE_DOUBLE, attr_ds);
  att.write(H5::PredType::NATIVE_DOUBLE, &val[0]);
}

void Hdf5Lib::write_attribute(const char name[],
                              const std::vector<std::complex<int16_t>>& val) {
  size_t size = val.size();
  hsize_t dims[] = {2 * size};
  H5::DataSpace attr_ds = H5::DataSpace(1, dims);
  H5::Attribute att =
      group_.createAttribute(name, H5::PredType::STD_I16BE, attr_ds);
  short val_pair[2 * size];
  for (size_t j = 0; j < size; j++) {
    val_pair[2 * j + 0] = std::real(val[j]);
    val_pair[2 * j + 1] = std::imag(val[j]);
  }
  att.write(H5::PredType::STD_I16BE, &val_pair[0]);
}

void Hdf5Lib::write_attribute(const char name[],
                              const std::vector<std::complex<float>>& val) {
  size_t size = val.size();
  hsize_t dims[] = {2 * size};
  H5::DataSpace attr_ds = H5::DataSpace(1, dims);
  H5::Attribute att =
      group_.createAttribute(name, H5::PredType::NATIVE_DOUBLE, attr_ds);
  double val_pair[2 * size];
  for (size_t j = 0; j < size; j++) {
    val_pair[2 * j + 0] = std::real(val[j]);
    val_pair[2 * j + 1] = std::imag(val[j]);
  }
  att.write(H5::PredType::NATIVE_DOUBLE, &val_pair[0]);
}

void Hdf5Lib::write_attribute(const char name[], size_t val) {
  hsize_t dims[] = {1};
  H5::DataSpace attr_ds = H5::DataSpace(1, dims);
  H5::Attribute att =
      group_.createAttribute(name, H5::PredType::STD_U32BE, attr_ds);
  uint32_t val_uint = val;
  att.write(H5::PredType::NATIVE_UINT, &val_uint);
}

void Hdf5Lib::write_attribute(const char name[], int val) {
  hsize_t dims[] = {1};
  H5::DataSpace attr_ds = H5::DataSpace(1, dims);
  H5::Attribute att =
      group_.createAttribute(name, H5::PredType::STD_I32BE, attr_ds);
  att.write(H5::PredType::NATIVE_INT, &val);
}

void Hdf5Lib::write_attribute(const char name[],
                              const std::vector<size_t>& val) {
  size_t size = val.size();
  hsize_t dims[] = {size};
  H5::DataSpace attr_ds = H5::DataSpace(1, dims);
  H5::Attribute att =
      group_.createAttribute(name, H5::PredType::STD_U32BE, attr_ds);
  std::vector<uint32_t> val_uint;
  for (size_t i = 0; i < val.size(); i++)
    val_uint.push_back((uint32_t)val.at(i));
  att.write(H5::PredType::NATIVE_UINT, &val_uint[0]);
}

void Hdf5Lib::write_attribute(const char name[], const std::string& val) {
  hsize_t dims[] = {1};
  H5::DataSpace attr_ds = H5::DataSpace(1, dims);
  H5::StrType strdatatype(H5::PredType::C_S1,
                          H5T_VARIABLE);  // of variable length characters
  H5::Attribute att = group_.createAttribute(name, strdatatype, attr_ds);
  att.write(strdatatype, val);
}

void Hdf5Lib::write_attribute(const char name[],
                              const std::vector<std::string>& val) {
  if (val.empty()) return;
  size_t size = val.size();
  H5::StrType strdatatype(H5::PredType::C_S1,
                          H5T_VARIABLE);  // of variable length characters
  hsize_t dims[] = {size};
  H5::DataSpace attr_ds = H5::DataSpace(1, dims);
  H5::Attribute att = group_.createAttribute(name, strdatatype, attr_ds);
  const char* cStrArray[size];

  for (size_t i = 0; i < size; ++i) {
    cStrArray[i] = val[i].c_str();
  }
  att.write(strdatatype, cStrArray);
}
};  // namespace Agora_recorder
