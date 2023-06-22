#ifndef DATASET_MODEL_H
#define DATASET_MODEL_H

#include <iostream>
#include <H5Cpp.h>

#include "armadillo"
#include "utils.h"

class DatasetChannelModel {
    
    protected:
        DatasetChannelModel( std::string dataset_path );
        ~DatasetChannelModel();

    public:        
        static void InstantiateDataset( std::string dataset_path );
        static arma::cx_fmat GetAndUpdateMatrix();

    private:

        static DatasetChannelModel* instance;

        hsize_t current_frame_num;

        arma::Cube<float> real_matrices_data;
        arma::Cube<float> im_matrices_data;

        arma::cx_fmat GetMatrixByFrame( hsize_t frame );

};

#endif
