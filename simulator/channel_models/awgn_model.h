#ifndef AWGN_MODEL_H
#define AWGN_MODEL_H

#include "armadillo"
#include "channel_model.h"

class AwgnModel: public ChannelModel {

    public:
        AwgnModel( const Config* config ) : ChannelModel( config, ChannelModel::kFlat ) {}

        void UpdateModel() override
        {

            arma::fmat rmat(ues_num_, bss_num_, arma::fill::ones);
            arma::fmat imat(ues_num_, bss_num_, arma::fill::zeros);

            h_flat_ = arma::cx_fmat(rmat, imat);

        }

};

#endif