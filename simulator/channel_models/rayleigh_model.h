#ifndef RAYLEIGH_MODEL_H
#define RAYLEIGH_MODEL_H

#include "armadillo"
#include "channel_model.h"

class RayleighModel: public ChannelModel {

    public:
        RayleighModel( const Config* config ) : ChannelModel( config ) {}

        //Build the channel matrix here
        arma::cx_fmat GetAndUpdateMatrix() override 
        {

            arma::fmat rmat(ues_num, bss_num, arma::fill::randn);
            arma::fmat imat(ues_num, bss_num, arma::fill::randn);

            return arma::cx_fmat(rmat, imat);

        }
       
};

#endif