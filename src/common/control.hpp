#ifndef CONTROL_HEAD
#define CONTROL_HEAD

#include <uchar.h>

static const size_t kNumUESetting = 4;
static const size_t kNumLoadSetting = 10;
static const size_t kNumSlot = kNumUESetting * kNumLoadSetting;

struct MCSItem
{
    size_t mod_order_bits;
    size_t code_rate;
};

// MCSItem MCS_table[32] = {{2, 120}, {2, 157}, {2, 193}, {2, 251}, {2, 308}, {2, 379}, 
//     {2, 449}, {2, 526}, {2, 602}, {2, 679}, {4, 340}, {4, 378}, {4, 434}, {4, 490},
//     {4, 553}, {4, 616}, {4, 658}, {6, 438}, {6, 466}, {6, 517}, {6, 567}, {6, 616},
//     {6, 666}, {6, 719}, {6, 772}, {6, 822}, {6, 873}, {6, 910}, {6, 948}};

struct ControlInfo
{
    size_t slot_id;
    size_t ue_id;
    size_t sc_start;
    size_t sc_end;
    size_t mod_order_bits;
    size_t Bg;
    size_t Zc;
};

#endif