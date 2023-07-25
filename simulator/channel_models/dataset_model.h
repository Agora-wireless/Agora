/**
 * @file dataset_model.h
 * @brief Declaration file for the dataset channel model.  Can be imported with an HDF5 file
 */
#ifndef DATASET_MODEL_H_
#define DATASET_MODEL_H_

#include "armadillo"
#include "channel_model.h"
#include "config.h"

class DatasetModel : public ChannelModel {
 public:
  DatasetModel(size_t bs_ant_num, size_t ue_ant_num, size_t samples_per_sym,
               const std::string& dataset_path);

  void UpdateModel() final;

 private:
  arma::cx_fmat GetMatricesByFrame(hsize_t frame);
  void InstantiateDataset(const std::string& dataset_path);

  /*
  * Vector of complex H Matrices x NumFrames
  * Access to H_Matrix using:
  * h_matrices_frames_[ FrameIndex ][ SubcarrierIndex ]
  */
  std::vector<std::vector<arma::cx_fmat>> h_matrices_frames_;
  hsize_t current_frame_num_;
};

#endif  //DATASET_MODEL_H_