#ifndef EXAMPLE_MODEL_H
#define EXAMPLE_MODEL_H

#include "armadillo"
#include "channel_model.h"

class ExampleModel: public ChannelModel {

    public:

        //Example model constructor, call parent class ChannelModel( Config, FadingType )
        ExampleModel( const Config* config ) : ChannelModel( config, ChannelModel::kFlat ) {}

        void UpdateModel() override
        {

            //Set a value for h_flat_ if Flat Fading
            //Set a value for h_selective_ if Selective Fading

        }

};

#endif