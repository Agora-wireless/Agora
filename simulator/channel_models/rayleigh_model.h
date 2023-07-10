#ifndef RAYLEIGH_MODEL_H
#define RAYLEIGH_MODEL_H

#include "armadillo"
#include "channel_model.h"

//Flat fading Rayleigh channel model
class RayleighModel: public ChannelModel {

    public:
        RayleighModel( const Config* config ) : ChannelModel( config, ChannelModel::kFlat ) {}

        void UpdateModel() override
        {

            arma::fmat rmat(ues_num_, bss_num_, arma::fill::randn);
            arma::fmat imat(ues_num_, bss_num_, arma::fill::randn);

            h_flat_ = arma::cx_fmat(rmat, imat);

        }

};

#endif