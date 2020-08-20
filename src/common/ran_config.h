#ifndef RANCONFIG
#define RANCONFIG

class RanConfig {
public:
    size_t n_antennas; /// Number of active antennas at the base station
};

class ControlPacket {
public:
    size_t ue_id; /// UE ID
    size_t mod_type; /// modulation type
};

#endif
