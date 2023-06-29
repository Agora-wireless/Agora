#ifndef AWGN_MODEL_H
#define AWGN_MODEL_H

#include "armadillo"
#include "channel_model.h"

class AwgnModel: public ChannelModel {

    public:
        AwgnModel( const Config* config ) : ChannelModel( config ) {}

        arma::cx_fmat GetAndUpdateMatrix() override 
        {

            arma::fmat rmat(ues_num, bss_num, arma::fill::ones);
            arma::fmat imat(ues_num, bss_num, arma::fill::zeros);

            return arma::cx_fmat(rmat, imat);

        }
       
};

#endif