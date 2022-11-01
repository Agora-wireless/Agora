/*
 Copyright (c) 2018-2022, Rice University
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

----------------------------------------------------------------------
 Implementation of Hdf5Lib API Class
---------------------------------------------------------------------
*/
#include "hdf5_lib.h"

#include <cassert>
#include <utility>

#include "logger.h"
#include "utils.h"

namespace Agora_recorder {
static constexpr bool kPrintDataSetInfo = true;
static constexpr size_t kDExtendDimIdx = 0;

Hdf5Lib::Hdf5Lib(H5std_string hdf5_name, H5std_string group_name)
    : hdf5_name_(std::move(hdf5_name)), group_name_(std::move(group_name)) {
  AGORA_LOG_TRACE("Hdf5Lib::Creating output HD5F file: %s\n",
                  hdf5_name_.c_str());
  file_ = std::make_unique<H5::H5File>(hdf5_name_, H5F_ACC_TRUNC);
  group_ = std::make_unique<H5::Group>(file_->createGroup("/" + group_name_));
  ///Disable for debugging
  H5::Exception::dontPrint();
}

Hdf5Lib::~Hdf5Lib() {
  for (auto& value : ds_name_id_) {
    AGORA_LOG_INFO("Hdf5Lib::%s - Finalizing Dataset\n", value.first.c_str());
    if (datasets_.at(value.second) != nullptr) {
      datasets_.at(value.second)->close();
    } else {
      AGORA_LOG_WARN("Hdf5Lib::%s - Dataset already closed\n",
                     value.first.c_str());
    }
    datasets_.at(value.second).reset();
  }
  datasets_.clear();
  ds_name_id_.clear();
  group_->close();
  file_->close();
}

///Dataset -> Data Type + Dataspace
///extend_dimension -1 for no extension
void Hdf5Lib::CreateDataset(const std::string& dataset_name,
                            const std::array<hsize_t, kDsDimsNum>& chunk_dims,
                            const std::array<hsize_t, kDsDimsNum>& init_dims,
                            const ssize_t extend_dimension,
                            const H5::PredType& type) {
  const std::string create_ds_name("/" + group_name_ + "/" + dataset_name);
  std::array<hsize_t, kDsDimsNum> max_ds_dims = init_dims;
  if ((extend_dimension >= 0) &&
      (static_cast<size_t>(extend_dimension) <= kDsDimsNum)) {
    max_ds_dims.at(extend_dimension) = H5S_UNLIMITED;
  }

  try {
    //Rank | Create Dimensions | Max Dimensions
    H5::DataSpace ds_dataspace(kDsDimsNum, init_dims.data(),
                               max_ds_dims.data());
    H5::DSetCreatPropList ds_prop;
    ds_prop.setChunk(kDsDimsNum, chunk_dims.data());
    //ds_prop.setFillValue(type, &fill_val);
    datasets_.emplace_back(std::make_unique<H5::DataSet>(
        file_->createDataSet(create_ds_name, type, ds_dataspace, ds_prop)));
    ds_prop.close();
    //Save name -> dataset idx (quick lookup table)
    ds_name_id_[dataset_name] = datasets_.size() - 1;
  }
  // catch failure caused by the H5File operations
  catch (H5::FileIException& error) {
    AGORA_LOG_ERROR("CreateDataset %s, FileIException - %s\n",
                    create_ds_name.c_str(), error.getCDetailMsg());
    H5::FileIException::printErrorStack();
    throw error;
  }

  // catch failure caused by the DataSet operations
  catch (H5::DataSetIException& error) {
    AGORA_LOG_ERROR("CreateDataset %s, DataSetIException - %s\n",
                    create_ds_name.c_str(), error.getCDetailMsg());
    H5::DataSetIException::printErrorStack();
    throw error;
  }

  // catch failure caused by the DataSpace operations
  catch (H5::DataSpaceIException& error) {
    AGORA_LOG_ERROR("CreateDataset %s, DataSpaceIException - %s\n",
                    create_ds_name.c_str(), error.getCDetailMsg());
    H5::DataSpaceIException::printErrorStack();
    throw error;
  }
  AGORA_LOG_TRACE("CreateDataset: %s\n", dataset_name.c_str());
}

void Hdf5Lib::FinalizeDataset(const std::string& dataset_name) {
  const size_t ds_id = ds_name_id_.at(dataset_name);
  ds_name_id_.erase(dataset_name);
  datasets_.at(ds_id)->close();
  datasets_.at(ds_id).reset();
  AGORA_LOG_INFO("%s(%zu) - FinalizeDataset\n", dataset_name.c_str(), ds_id);
}

void Hdf5Lib::ExtendDataset(
    const std::string& dataset_name,
    const std::array<hsize_t, kDsDimsNum>& extended_dims) {
  const std::string ds_name("/" + group_name_ + "/" + dataset_name);
  const size_t ds_id = ds_name_id_.at(dataset_name);
  auto& current_dataset = datasets_.at(ds_id);
  try {
    // H5F_SCOPE_LOCAL | H5F_SCOPE_GLOBAL
    current_dataset->flush(H5F_SCOPE_LOCAL);
    current_dataset->extend(extended_dims.data());
    AGORA_LOG_INFO("ExtendDataset: Extending %s dataset to frame %llu\n",
                   dataset_name.c_str(), extended_dims.at(kDExtendDimIdx));

  } catch (H5::DataSetIException& error) {
    H5::DataSetIException::printErrorStack();
    AGORA_LOG_WARN(
        "ExtendDataset: DataSetIException - Failed to extend the dataset\n");
    throw error;
  } catch (H5::DataSpaceIException& error) {
    H5::DataSpaceIException::printErrorStack();
    AGORA_LOG_WARN(
        "ExtendDataset: DataSpaceIException - Failed to extend the dataset\n");
    throw error;
  }
}

herr_t Hdf5Lib::WriteDataset(const std::string& dataset_name,
                             const std::array<hsize_t, kDsDimsNum>& start,
                             const std::array<hsize_t, kDsDimsNum>& count,
                             const short* wrt_data) {
  const std::string ds_name("/" + group_name_ + "/" + dataset_name);
  const size_t ds_id = ds_name_id_.at(dataset_name);
  herr_t ret = 0;
  AGORA_LOG_TRACE("WriteDataset: %s\n", dataset_name.c_str());

  auto& current_dataset = datasets_.at(ds_id);
  // Select a hyperslab in extended portion of the dataset
  try {
    H5::DataSpace working_space = current_dataset->getSpace();
    ///Select the hyperslab
    working_space.selectHyperslab(H5S_SELECT_SET, count.data(), start.data());

    // define memory space (wrt / chunk)
    H5::DataSpace mem_space(kDsDimsNum, count.data(), nullptr);
    // Write the data to the hyperslab
    current_dataset->write(wrt_data, H5::PredType::NATIVE_INT16, mem_space,
                           working_space);
    working_space.selectNone();
    mem_space.close();
  }
  // catch failure caused by the DataSet operations
  catch (H5::DataSetIException& error) {
    H5::DataSetIException::printErrorStack();

    AGORA_LOG_WARN(
        "DataSet: Failed to write to %s dataset at primary dim index: %llu\n",
        dataset_name.c_str(), start.at(kDExtendDimIdx));

    const int ndims = datasets_.at(ds_id)->getSpace().getSimpleExtentNdims();

    std::stringstream ss;
    ss.str(std::string());
    ss << "\nRequested Write Dimension is: " << ndims;
    for (size_t i = 0; i < (kDsDimsNum - 1); ++i) {
      ss << start.at(i) << ", ";
    }
    ss << start.at(kDsDimsNum - 1);
    AGORA_LOG_WARN("%s\n", ss.str().c_str());
    ret = -1;
    throw error;
  }
  // catch failure caused by the DataSpace operations
  catch (H5::DataSpaceIException& error) {
    H5::DataSpaceIException::printErrorStack();
    ret = -1;
    throw error;
  }
  return ret;
}

herr_t Hdf5Lib::WriteDataset(const std::string& dataset_name,
                             const std::array<hsize_t, kDsDimsNum>& start,
                             const std::array<hsize_t, kDsDimsNum>& count,
                             const float* wrt_data) {
  const std::string ds_name("/" + group_name_ + "/" + dataset_name);
  const size_t ds_id = ds_name_id_.at(dataset_name);
  herr_t ret = 0;
  AGORA_LOG_TRACE("WriteDataset: %s\n", dataset_name.c_str());

  auto& current_dataset = datasets_.at(ds_id);
  // Select a hyperslab in extended portion of the dataset
  try {
    H5::DataSpace working_space = current_dataset->getSpace();
    ///Select the hyperslab
    working_space.selectHyperslab(H5S_SELECT_SET, count.data(), start.data());

    // define memory space (wrt / chunk)
    H5::DataSpace mem_space(kDsDimsNum, count.data(), nullptr);
    // Write the data to the hyperslab
    current_dataset->write(wrt_data, H5::PredType::INTEL_F32, mem_space,
                           working_space);
    mem_space.close();
  }
  // catch failure caused by the DataSet operations
  catch (H5::DataSetIException& error) {
    H5::DataSetIException::printErrorStack();

    AGORA_LOG_WARN(
        "DataSet: Failed to write to dataset at primary dim index: %llu\n",
        start.at(kDExtendDimIdx));

    const int ndims = datasets_.at(ds_id)->getSpace().getSimpleExtentNdims();

    std::stringstream ss;
    ss.str(std::string());
    ss << "\nRequested Write Dimension is: " << ndims;
    for (size_t i = 0; i < (kDsDimsNum - 1); ++i) {
      ss << start.at(i) << ", ";
    }
    ss << start.at(kDsDimsNum - 1);
    AGORA_LOG_TRACE("%s\n", ss.str().c_str());
    ret = -1;
    throw;
  }
  // catch failure caused by the DataSpace operations
  catch (H5::DataSpaceIException& error) {
    H5::DataSpaceIException::printErrorStack();
    ret = -1;
    throw;
  }
  return ret;
}

void Hdf5Lib::WriteAttribute(const char name[], double val) {
  hsize_t dims[] = {1};
  H5::DataSpace attr_ds = H5::DataSpace(1, dims);
  H5::Attribute att =
      group_->createAttribute(name, H5::PredType::NATIVE_DOUBLE, attr_ds);
  att.write(H5::PredType::NATIVE_DOUBLE, &val);
}

void Hdf5Lib::WriteAttribute(const char name[],
                             const std::vector<double>& val) {
  size_t size = val.size();
  hsize_t dims[] = {size};
  H5::DataSpace attr_ds = H5::DataSpace(1, dims);
  H5::Attribute att =
      group_->createAttribute(name, H5::PredType::NATIVE_DOUBLE, attr_ds);
  att.write(H5::PredType::NATIVE_DOUBLE, &val[0]);
}

void Hdf5Lib::WriteAttribute(const char name[],
                             const std::vector<std::complex<int16_t>>& val) {
  size_t size = val.size();
  hsize_t dims[] = {2 * size};
  H5::DataSpace attr_ds = H5::DataSpace(1, dims);
  H5::Attribute att =
      group_->createAttribute(name, H5::PredType::STD_I16BE, attr_ds);
  short val_pair[2 * size];
  for (size_t j = 0; j < size; j++) {
    val_pair[2 * j + 0] = std::real(val[j]);
    val_pair[2 * j + 1] = std::imag(val[j]);
  }
  att.write(H5::PredType::STD_I16BE, &val_pair[0]);
}

void Hdf5Lib::WriteAttribute(const char name[],
                             const std::vector<std::complex<float>>& val) {
  size_t size = val.size();
  hsize_t dims[] = {2 * size};
  H5::DataSpace attr_ds = H5::DataSpace(1, dims);
  H5::Attribute att =
      group_->createAttribute(name, H5::PredType::NATIVE_DOUBLE, attr_ds);
  double val_pair[2 * size];
  for (size_t j = 0; j < size; j++) {
    val_pair[2 * j + 0] = std::real(val[j]);
    val_pair[2 * j + 1] = std::imag(val[j]);
  }
  att.write(H5::PredType::NATIVE_DOUBLE, &val_pair[0]);
}

void Hdf5Lib::WriteAttribute(const char name[], size_t val) {
  hsize_t dims[] = {1};
  H5::DataSpace attr_ds = H5::DataSpace(1, dims);
  H5::Attribute att =
      group_->createAttribute(name, H5::PredType::STD_U32BE, attr_ds);
  uint32_t val_uint = val;
  att.write(H5::PredType::NATIVE_UINT, &val_uint);
}

void Hdf5Lib::WriteAttribute(const char name[], int val) {
  hsize_t dims[] = {1};
  H5::DataSpace attr_ds = H5::DataSpace(1, dims);
  H5::Attribute att =
      group_->createAttribute(name, H5::PredType::STD_I32BE, attr_ds);
  att.write(H5::PredType::NATIVE_INT, &val);
}

void Hdf5Lib::WriteAttribute(const char name[],
                             const std::vector<size_t>& val) {
  size_t size = val.size();
  hsize_t dims[] = {size};
  H5::DataSpace attr_ds = H5::DataSpace(1, dims);
  H5::Attribute att =
      group_->createAttribute(name, H5::PredType::STD_U32BE, attr_ds);
  std::vector<uint32_t> val_uint;
  val_uint.reserve(val.size());
  for (unsigned long i : val) {
    val_uint.push_back(static_cast<uint32_t>(i));
  }
  att.write(H5::PredType::NATIVE_UINT, &val_uint[0]);
}

void Hdf5Lib::WriteAttribute(const char name[], const std::string& val) {
  hsize_t dims[] = {1};
  H5::DataSpace attr_ds = H5::DataSpace(1, dims);
  H5::StrType strdatatype(H5::PredType::C_S1,
                          H5T_VARIABLE);  // of variable length characters
  H5::Attribute att = group_->createAttribute(name, strdatatype, attr_ds);
  att.write(strdatatype, val);
}

void Hdf5Lib::WriteAttribute(const char name[],
                             const std::vector<std::string>& val) {
  if (val.empty()) {
    return;
  }
  size_t size = val.size();
  H5::StrType strdatatype(H5::PredType::C_S1,
                          H5T_VARIABLE);  // of variable length characters
  hsize_t dims[] = {size};
  H5::DataSpace attr_ds = H5::DataSpace(1, dims);
  H5::Attribute att = group_->createAttribute(name, strdatatype, attr_ds);
  const char* c_str_array[size];

  for (size_t i = 0; i < size; ++i) {
    c_str_array[i] = val[i].c_str();
  }
  att.write(strdatatype, c_str_array);
}
};  // namespace Agora_recorder
