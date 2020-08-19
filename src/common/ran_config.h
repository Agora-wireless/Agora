#ifndef RANCONFIG
#define RANCONFIG

class RanConfig {
public:
    size_t nAntennas; /// Number of antennas
};

class ControlInformation {
public:
    size_t ueId; /// UE ID
    size_t modType; /// modulation type
};

#endif
