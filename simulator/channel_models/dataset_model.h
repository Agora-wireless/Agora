#ifndef DATASET_MODEL_H
#define DATASET_MODEL_H

#include "armadillo"
#include "channel_model.h"
#include "config.h"

class DatasetModel : public ChannelModel {
    public:
        DatasetModel(const Config* config, std::string dataset_path) : ChannelModel(config) 
        {
            InstantiateDataset(dataset_path);
        }

    void InstantiateDataset(std::string dataset_path);

    arma::cx_fmat GetAndUpdateMatrix() override;

    protected:
        hsize_t current_frame_num;

        arma::Cube<float> real_matrices_data;
        arma::Cube<float> im_matrices_data;

        arma::cx_fmat GetMatrixByFrame(hsize_t frame);
};

#endif