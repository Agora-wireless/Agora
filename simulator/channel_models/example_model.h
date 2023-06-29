#ifndef EXAMPLE_MODEL_H
#define EXAMPLE_MODEL_H

#include "armadillo"
#include "channel_model.h"

class ExampleModel: public ChannelModel {

    public:
        ExampleModel( const Config* config ) : ChannelModel( config ) {}

        arma::cx_fmat GetAndUpdateMatrix() override 
        {

            //GetAndUpdateMatrix is called each frame 
            return arma::cx_fmat();

        }
       
};

#endif