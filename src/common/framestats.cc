// Copyright (c) 2018-2020, Rice University
// RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

/**
 * @file framestats.cc
 * @brief Class implementation for frame tracking
 */
#include "framestats.h"
#include <algorithm>
#include <cassert>

FrameStats::FrameStats(std::string new_frame_id)
    : frame_identifier_(new_frame_id)
    , client_ul_pilot_symbols_(0)
    , client_dl_pilot_symbols_(0)
{
    for (size_t i = 0; i < frame_identifier_.length(); i++) {
        char symbol = frame_identifier_.at(i);
        switch (symbol) {
        case 'B': {
            beacon_symbols_.push_back(i);
            break;
        }

        case 'C': {
            dl_cal_symbols_.push_back(i);
            break;
        }

        case 'D': {
            dl_symbols_.push_back(i);
            break;
        }

        case 'G': {
            break;
        }

        case 'L': {
            ul_cal_symbols_.push_back(i);
            break;
        }

        case 'P': {
            pilot_symbols_.push_back(i);
            break;
        }

        case 'U': {
            ul_symbols_.push_back(i);
            break;
        }

        default: {
            std::printf("!!!!! Unknown symbol in frame: %c : %s !!!!!\n",
                symbol, frame_identifier_.c_str());
        }
        }
    }
}

FrameStats::FrameStats(std::string new_frame_id, size_t ul, size_t dl)
    : FrameStats(new_frame_id)
{
    this->SetClientPilotSyms(ul, dl);
}

void FrameStats::SetClientPilotSyms(size_t ul, size_t dl)
{
    /* Client pilot symbols must be strictly less than the number of corresponding data symbols */
    assert((this->ul_symbols_.size() == 0) || (ul < this->ul_symbols_.size()));
    this->client_ul_pilot_symbols_ = ul;
    assert((this->dl_symbols_.size() == 0) || (dl < this->dl_symbols_.size()));
    this->client_dl_pilot_symbols_ = dl;
}

size_t FrameStats::NumDLCalSyms(void) const
{
    return this->dl_cal_symbols_.size();
}

size_t FrameStats::NumULCalSyms(void) const
{
    return this->ul_cal_symbols_.size();
}

size_t FrameStats::NumDLSyms(void) const { return this->dl_symbols_.size(); }

size_t FrameStats::NumULSyms(void) const { return this->ul_symbols_.size(); }

size_t FrameStats::NumBeaconSyms(void) const
{
    return this->beacon_symbols_.size();
}

size_t FrameStats::NumPilotSyms(void) const
{
    return this->pilot_symbols_.size();
}

size_t FrameStats::NumTotalSyms(void) const
{
    return this->frame_identifier_.length();
}

bool FrameStats::IsRecCalEnabled(void) const
{
    return ((this->ul_cal_symbols_.size() > 0)
        && (this->dl_cal_symbols_.size() > 0));
}

size_t FrameStats::NumDataSyms(void) const
{
    return (this->NumTotalSyms()
        - (this->pilot_symbols_.size() + this->beacon_symbols_.size()));
}

size_t FrameStats::GetDLSymbol(size_t location) const
{
    return this->dl_symbols_.at(location);
}

size_t FrameStats::GetDLCalSymbol(size_t location) const
{
    return this->dl_cal_symbols_.at(location);
}

size_t FrameStats::GetULSymbol(size_t location) const
{
    return this->ul_symbols_.at(location);
}

size_t FrameStats::GetULCalSymbol(size_t location) const
{
    return this->ul_cal_symbols_.at(location);
}

size_t FrameStats::GetPilotSymbol(size_t location) const
{
    return this->pilot_symbols_.at(location);
}

size_t FrameStats::GetSymbolIdx(
    const std::vector<size_t>& search_vector, size_t symbol_number)
{
    /* Todo Optimize this, probably with a precomputed table -- assumes sorted list*/
    const auto [start, finish] = std::equal_range(
        search_vector.begin(), search_vector.end(), symbol_number);

    if (start == finish) {
        return SIZE_MAX;
    } else if ((start + 1) == finish) {
        return (start - search_vector.begin());
    } else { /* Invalid, multiple locations */
        assert(false);
        return SIZE_MAX;
    }
}

size_t FrameStats::GetDLSymbolIdx(size_t symbol_number) const
{
    return FrameStats::GetSymbolIdx(this->dl_symbols_, symbol_number);
}

size_t FrameStats::GetULSymbolIdx(size_t symbol_number) const
{
    return FrameStats::GetSymbolIdx(this->ul_symbols_, symbol_number);
}

size_t FrameStats::GetPilotSymbolIdx(size_t symbol_number) const
{
    return FrameStats::GetSymbolIdx(this->pilot_symbols_, symbol_number);
}
