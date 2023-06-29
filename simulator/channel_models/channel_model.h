#ifndef CHANNEL_MODEL_H
#define CHANNEL_MODEL_H

#include "config.h"
#include "armadillo"

class ChannelModel {

    protected:
        size_t bss_num;
        size_t ues_num;
        size_t n_samps_;

    public:
        ChannelModel( const Config* config )
        {
            
            bss_num = config->BsAntNum();
            ues_num = config->UeAntNum();
            n_samps_ = config->SampsPerSymbol();

        }

        virtual ~ChannelModel() = default;
        virtual arma::cx_fmat GetAndUpdateMatrix(){ return arma::cx_fmat(); }

};

#endif
