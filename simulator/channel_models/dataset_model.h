#ifndef DATASET_MODEL_H
#define DATASET_MODEL_H

#include "armadillo"
#include "channel_model.h"
#include "config.h"

class DatasetModel : public ChannelModel 
{
    
    public:
        DatasetModel(const Config* config, std::string dataset_path);

        void InstantiateDataset(std::string dataset_path);
        void UpdateModel() override;

    protected:

        /*
        * Vector of complex H Matrices x NumFrames
        * Access to H_Matrix using:
        * h_matrices_frames[ FrameIndex ][ SubcarrierIndex ]
        */
        std::vector< std::vector<arma::cx_fmat> > h_matrices_frames;

        hsize_t current_frame_num;

        arma::cx_fmat GetMatricesByFrame(hsize_t frame);

};

#endif