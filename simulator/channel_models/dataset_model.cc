/**
 * @file dataset_model.cc
 * @brief Implementation file for the dataset channel model.  Can be imported with an HDF5 file
 */

#include "dataset_model.h"

#include <iostream>

#include "H5Cpp.h"
#include "logger.h"

bool kPrintDatasetOutput = true;

DatasetModel::DatasetModel(size_t bs_ant_num, size_t ue_ant_num,
                           size_t samples_per_sym,
                           const std::string& dataset_path)
    : ChannelModel(bs_ant_num, ue_ant_num, samples_per_sym,
                   ChannelModel::kSelective) {
  InstantiateDataset(dataset_path);
}

void DatasetModel::InstantiateDataset(const std::string& dataset_path) {
  hsize_t frames_num = 0;
  hsize_t scs_num = 0;
  hsize_t bss_num = 0;
  hsize_t ues_num = 0;

  current_frame_num_ = 0;

  try {
    H5::H5File file(dataset_path, H5F_ACC_RDONLY);
    H5::DataSet h_r_dataset =
        file.openDataSet("H_r");  //H Matrix Real part dataset
    H5::DataSet h_i_dataset =
        file.openDataSet("H_i");  //H Matrix Imaginary part dataset

    H5::DataSpace real_dataspace = h_r_dataset.getSpace();
    H5::DataSpace im_dataspace = h_i_dataset.getSpace();

    hsize_t real_dimensions[4];
    real_dataspace.getSimpleExtentDims(real_dimensions);

    hsize_t im_dimensions[4];
    im_dataspace.getSimpleExtentDims(im_dimensions);

    //Check the Matrix types dimensions
    if (im_dimensions[0] != real_dimensions[0] ||
        im_dimensions[1] != real_dimensions[1] ||
        im_dimensions[2] != real_dimensions[2]) {
      AGORA_LOG_ERROR("Invalid Dataset size\n");
    }

    frames_num = real_dimensions[0];
    scs_num = real_dimensions[1];
    bss_num = real_dimensions[2];
    ues_num = real_dimensions[3];

    //Instantiate the matrices data
    const size_t data_read_size = frames_num * scs_num * bss_num * ues_num;
    std::vector<float> real_temp_data(data_read_size);
    std::vector<float> im_temp_data(data_read_size);

    h_r_dataset.read(real_temp_data.data(), H5::PredType::NATIVE_FLOAT);
    h_i_dataset.read(im_temp_data.data(), H5::PredType::NATIVE_FLOAT);

    hsize_t data_index = 0;

    for (hsize_t frame = 0; frame < frames_num; frame++) {
      auto& h_subcarrier_matrices = h_matrices_frames_.emplace_back();

      for (hsize_t subcarrier = 0; subcarrier < scs_num; subcarrier++) {
        auto& h_mat = h_subcarrier_matrices.emplace_back(ues_num, bss_num);

        for (hsize_t bs = 0; bs < bss_num; bs++) {
          for (hsize_t ue = 0; ue < ues_num; ue++) {
            h_mat(ue, bs) = arma::cx_float(real_temp_data.at(data_index),
                                           im_temp_data.at(data_index));
            data_index++;
          }
        }
      }

      if (kPrintDatasetOutput) {
        std::printf(
            "Dataset Frame = %ld with %ld Subcarriers loaded successfully \n",
            h_matrices_frames_.size(), h_subcarrier_matrices.size());
        //Utils::PrintMat( h_matrices_frames_[0][0], "H_");
      }
    }

  } catch (H5::FileIException& error) {
    AGORA_LOG_ERROR("Read Dataset %s, FileIException - %s\n",
                    dataset_path.c_str(), error.getCDetailMsg());
    H5::FileIException::printErrorStack();
    throw error;

  } catch (H5::DataSetIException& error) {
    AGORA_LOG_ERROR("Read Dataset %s, DataSetIException - %s\n",
                    dataset_path.c_str(), error.getCDetailMsg());

    H5::DataSetIException::printErrorStack();
    throw error;

  } catch (H5::DataSpaceIException& error) {
    AGORA_LOG_ERROR("CreateDataset %s, DataSpaceIException - %s\n",
                    dataset_path.c_str(), error.getCDetailMsg());

    H5::DataSpaceIException::printErrorStack();
    throw error;
  }

  AGORA_LOG_INFO(
      "Dataset Succesfully loaded\n"
      "Path: %s \n"
      "Frames: %lld \n"
      "Subcarriers: %lld \n"
      "BSs: %lld \n"
      "UEs: %lld \n",
      dataset_path.c_str(), frames_num, scs_num, bss_num, ues_num);
}

void DatasetModel::UpdateModel() {
  h_selective_ = h_matrices_frames_[current_frame_num_];
  current_frame_num_++;
}