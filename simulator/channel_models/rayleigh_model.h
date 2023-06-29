#ifndef RAYLEIGH_MODEL_H
#define RAYLEIGH_MODEL_H

#include "armadillo"
#include "channel_model.h"

//Flat fading Rayleigh channel model
class RayleighModel: public ChannelModel {

    public:
        RayleighModel( const Config* config ) : ChannelModel( config ) {}

        arma::cx_fmat GetAndUpdateMatrix() override 
        {

            arma::fmat rmat(ues_num, bss_num, arma::fill::randn);
            arma::fmat imat(ues_num, bss_num, arma::fill::randn);

            return arma::cx_fmat(rmat, imat);

        }
       
};

#endif