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
struct BottleneckEncode {
    double encode;
    double idle;
};

class Diagnosis {

    public:
        Diagnosis(Config* in_config, SharedState* in_state, 
            std::vector<BottleneckSubcarrier>& in_subcarrier_bottleneck,
            std::vector<BottleneckDecode>& in_decode_bottleneck,
            std::vector<BottleneckEncode>& in_encode_bottleneck);
        ~Diagnosis();

    private:
        
};

#endif
