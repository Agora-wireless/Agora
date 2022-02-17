#ifndef DIAGNOSIS_HPP
#define DIAGNOSIS_HPP

#include "shared_counters.hpp"

struct BottleneckSubcarrier {
    double csi;
    double zf;
    double demul;
    double idle;
};
struct BottleneckDecode {
    double decode;
    double idle;
};

class Diagnosis {

    public:
        Diagnosis(Config* in_config, SharedState* in_state, 
            std::vector<BottleneckSubcarrier>& in_subcarrier_bottleneck,
            std::vector<BottleneckDecode>& in_decode_bottleneck);
        ~Diagnosis();

    private:
        
};

#endif
