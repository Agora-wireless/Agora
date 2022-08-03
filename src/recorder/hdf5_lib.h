/*
Copyright (c) 2018-2022
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 
----------------------------------------------------------------------
 API to read and write hdf5 file
---------------------------------------------------------------------
*/

#ifndef AGORA_HDF5LIB_H_
#define AGORA_HDF5LIB_H_

#include <algorithm>
#include <array>
#include <complex>
#include <map>
#include <memory>

#include "H5Cpp.h"

namespace Agora_recorder {
static constexpr size_t kDsDimsNum = 5;

class Hdf5Lib {
 public:
  Hdf5Lib(H5std_string hdf5_name, H5std_string group_name);
  ~Hdf5Lib();

  void CreateDataset(const std::string& dataset_name,
                     const std::array<hsize_t, kDsDimsNum>& chunk_dims,
                     const std::array<hsize_t, kDsDimsNum>& init_dims,
                     const ssize_t extend_dimension = 0,
                     const H5::PredType& type = H5::PredType::STD_I16BE);
  void FinalizeDataset(const std::string& dataset_name);

  void ExtendDataset(const std::string& dataset_name,
                     const std::array<hsize_t, kDsDimsNum>& extended_dims);
  herr_t WriteDataset(const std::string& dataset_name,
                      const std::array<hsize_t, kDsDimsNum>& start,
                      const std::array<hsize_t, kDsDimsNum>& count,
                      const short* wrt_data);

  herr_t WriteDataset(const std::string& dataset_name,
                      const std::array<hsize_t, kDsDimsNum>& start,
                      const std::array<hsize_t, kDsDimsNum>& count,
                      const float* wrt_data);

  void WriteAttribute(const char name[], double val);
  void WriteAttribute(const char name[], const std::vector<double>& val);
  void WriteAttribute(const char name[],
                      const std::vector<std::complex<int16_t>>& val);
  void WriteAttribute(const char name[],
                      const std::vector<std::complex<float>>& val);
  void WriteAttribute(const char name[], size_t val);
  void WriteAttribute(const char name[], int val);
  void WriteAttribute(const char name[], const std::vector<size_t>& val);
  void WriteAttribute(const char name[], const std::string& val);
  void WriteAttribute(const char name[], const std::vector<std::string>& val);

 private:
  H5std_string hdf5_name_;
  H5std_string group_name_;
  std::unique_ptr<H5::H5File> file_;
  std::unique_ptr<H5::Group> group_;

  ///Datset lookup table
  std::map<std::string, size_t> ds_name_id_;
  std::vector<std::unique_ptr<H5::DataSet>> datasets_;
};
};      // namespace Agora_recorder
#endif  // AGORA_HDF5LIB_H_
