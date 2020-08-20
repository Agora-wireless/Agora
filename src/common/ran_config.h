#ifndef RANCONFIG
#define RANCONFIG

class RanConfig {
public:
    size_t nAntennas; /// Number of antennas
};

struct ControlPacket {
    size_t tti; // TTI index
    size_t ue_id; // UE ID
    size_t mod_type; // modulation type
    ControlPacket(size_t t, size_t u, size_t m)
        : tti(t)
        , ue_id(u)
        , mod_type(m)
    {
    }
};

#endif
