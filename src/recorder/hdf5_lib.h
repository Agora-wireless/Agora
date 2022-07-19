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

#include "H5Cpp.h"

namespace Agora_recorder {
static constexpr size_t kDsDimsNum = 5;
class Hdf5Lib {
 public:
  Hdf5Lib(H5std_string file_name, H5std_string group_name);
  ~Hdf5Lib();
  void closeFile();
  int createDataset(std::string dataset_name,
                    std::array<hsize_t, kDsDimsNum> tot_dims,
                    std::array<hsize_t, kDsDimsNum> chunk_dims);
  void removeDataset(std::string dataset_name);
  void openDataset();
  void closeDataset();
  bool extendDataset(std::string dataset_name, size_t prim_dim_size);
  herr_t writeDataset(std::string dataset_name,
                      std::array<hsize_t, kDsDimsNum> target_id,
                      std::array<hsize_t, kDsDimsNum> wrt_dim, short* wrt_data);
  std::vector<short> readDataset(std::string dataset_name,
                                 std::array<hsize_t, kDsDimsNum> target_id,
                                 std::array<hsize_t, kDsDimsNum> read_dim);
  void setTargetPrimaryDimSize(hsize_t dim_size) {
    target_prim_dim_size = dim_size;
  }
  hsize_t getTargetPrimaryDimSize() { return target_prim_dim_size; }
  void setMaxPrimaryDimSize(hsize_t dim_size) { max_prim_dim_size = dim_size; }
  hsize_t getMaxPrimaryDimSize() { return max_prim_dim_size; }
  void write_attribute(const char name[], double val);
  void write_attribute(const char name[], const std::vector<double>& val);
  void write_attribute(const char name[],
                       const std::vector<std::complex<int16_t>>& val);
  void write_attribute(const char name[],
                       const std::vector<std::complex<float>>& val);
  void write_attribute(const char name[], size_t val);
  void write_attribute(const char name[], int val);
  void write_attribute(const char name[], const std::vector<size_t>& val);
  void write_attribute(const char name[], const std::string& val);
  void write_attribute(const char name[], const std::vector<std::string>& val);

 private:
  H5std_string hdf5_name_;
  H5std_string group_name_;
  H5::H5File* file_;
  H5::Group group_;

  std::vector<std::string> dataset_str_;
  std::vector<H5::DSetCreatPropList> prop_list_;
  std::vector<H5::DataSpace> dataspace_;
  std::vector<H5::DataSet*> datasets_;
  std::vector<std::array<hsize_t, kDsDimsNum>> dims_;
  hsize_t target_prim_dim_size;
  hsize_t max_prim_dim_size;

  std::map<std::string, size_t> ds_name_id;
};
};      // namespace Agora_recorder
#endif  // AGORA_HDF5LIB_H_
