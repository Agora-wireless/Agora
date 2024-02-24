// Copyright (c) 2018-2020, Rice University
// RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

/**
 * @file framestats.h
 * @brief Class defination for frame tracking
 * @author Rice University
 */

#ifndef FRAMESTATS_H_
#define FRAMESTATS_H_

#include <cassert>
#include <stdexcept>
#include <string>
#include <vector>

#include "symbols.h"

class FrameStats {
 public:
  explicit FrameStats(std::string new_frame_id);
  FrameStats(std::string new_frame_id, size_t ul, size_t dl);

  void SetClientPilotSyms(size_t ul, size_t dl);

  size_t NumDLCalSyms() const;
  size_t NumULCalSyms() const;
  size_t NumDLSyms() const;
  size_t NumULSyms() const;
  size_t NumPilotSyms() const;
  size_t NumBeaconSyms() const;
  size_t NumDlControlSyms() const;
  size_t NumDlBcastSyms() const;
  size_t NumTotalSyms() const;

  /* Returns SIZE_MAX if symbol number is not a beacon */
  size_t GetBeaconSymbolIdx(size_t symbol_number) const;
  size_t GetDLDataSymbolStart() const;
  size_t GetDLControlSymbolIdx(size_t symbol_number) const;
  size_t GetDLSymbol(size_t location) const;
  inline size_t GetDLDataSymbol(size_t location) const {
    return GetDLSymbol(location + client_dl_pilot_symbols_);
  }
  inline size_t GetDLSymbolLast() const {
    return ((this->dl_symbols_.empty()) ? SIZE_MAX : this->dl_symbols_.back());
  }
  /* Returns SIZE_MAX if there are no DL symbols */
  size_t GetDLSymbolIdx(size_t symbol_number) const;

  size_t GetULDataSymbolStart() const;
  size_t GetULSymbol(size_t location) const;
  inline size_t GetULDataSymbol(size_t location) const {
    return GetULSymbol(location + client_ul_pilot_symbols_);
  }
  inline size_t GetULSymbolLast() const {
    return ((this->ul_symbols_.empty()) ? SIZE_MAX : this->ul_symbols_.back());
  }
  /* Returns SIZE_MAX if there are no UL symbols */
  size_t GetULSymbolIdx(size_t symbol_number) const;

  size_t GetPilotSymbol(size_t location) const;
  size_t GetPilotSymbolIdx(size_t symbol_number) const;

  size_t GetDLCalSymbol(size_t location) const;
  size_t GetDLCalSymbolIdx(size_t symbol_number) const;
  size_t GetULCalSymbol(size_t location) const;

  size_t GetBeaconSymbol(size_t location) const;
  inline size_t GetBeaconSymbolLast() const {
    return ((this->beacon_symbols_.empty()) ? SIZE_MAX
                                            : this->beacon_symbols_.back());
  }
  size_t GetDLControlSymbol(size_t location) const;

  bool IsRecCalEnabled() const;
  size_t NumDataSyms() const;

  /* Accessors */
  inline const std::string& FrameIdentifier() const {
    return frame_identifier_;
  }
  inline size_t ClientUlPilotSymbols() const {
    return client_ul_pilot_symbols_;
  }
  inline size_t ClientDlPilotSymbols() const {
    return client_dl_pilot_symbols_;
  }

  inline size_t NumUlDataSyms() const {
    return this->NumULSyms() - this->ClientUlPilotSymbols();
  }
  inline size_t NumDlDataSyms() const {
    return this->NumDLSyms() - this->ClientDlPilotSymbols();
  }

  //Returns Beacon+Dl symbol index
  inline size_t GetBeaconDlIdx(size_t symbol_id) const {
    size_t symbol_idx = SIZE_MAX;
    const auto type = GetSymbolType(symbol_id);
    if (type == SymbolType::kBeacon) {
      symbol_idx = GetBeaconSymbolIdx(symbol_id);
    } else if (type == SymbolType::kControl) {
      symbol_idx = GetDLControlSymbolIdx(symbol_id) + NumBeaconSyms();
    } else if (type == SymbolType::kDL) {
      symbol_idx = GetDLSymbolIdx(symbol_id) + NumDlBcastSyms();
    } else {
      throw std::runtime_error("Invalid BS Beacon or DL symbol id " +
                               std::to_string(symbol_id));
    }
    return symbol_idx;
  }

  //Returns Pilot+Ul symbol index
  inline size_t GetPilotUlIdx(size_t symbol_id) const {
    size_t symbol_idx = SIZE_MAX;
    const auto type = this->GetSymbolType(symbol_id);
    if (type == SymbolType::kPilot) {
      symbol_idx = GetPilotSymbolIdx(symbol_id);
    } else if (type == SymbolType::kUL) {
      symbol_idx = GetULSymbolIdx(symbol_id) + NumPilotSyms();
    } else {
      throw std::runtime_error("Invalid Ue Pilot or UL symbol id " +
                               std::to_string(symbol_id));
    }
    return symbol_idx;
  }

  /* Returns True if it is a dl pilot_
   * False otherwise */
  inline bool IsDlPilot(size_t symbol_id) const {
    bool is_pilot = false;
    assert(symbol_id < this->NumTotalSyms());
    const auto type = this->GetSymbolType(symbol_id);
    if (type == SymbolType::kDL && (this->ClientDlPilotSymbols() > 0)) {
      size_t dl_index = this->GetDLSymbolIdx(symbol_id);
      is_pilot = (this->ClientDlPilotSymbols() > dl_index);
    }
    return is_pilot;
  }

  /* Public functions that do not meet coding standard format */
  /// Return the symbol type of this symbol in this frame
  SymbolType GetSymbolType(size_t symbol_id) const;

 private:
  std::string frame_identifier_;

  std::vector<size_t> beacon_symbols_;
  std::vector<size_t> pilot_symbols_;
  std::vector<size_t> ul_symbols_;
  std::vector<size_t> ul_cal_symbols_;
  std::vector<size_t> dl_symbols_;
  std::vector<size_t> dl_cal_symbols_;
  std::vector<size_t> dl_control_symbols_;

  size_t client_ul_pilot_symbols_;
  size_t client_dl_pilot_symbols_;
  size_t client_ul_data_symbols_;
  size_t client_dl_data_symbols_;

  /* Helper function */
  static size_t GetSymbolIdx(const std::vector<size_t>& search_vector,
                             size_t symbol_number);
}; /* class FrameStats */

#endif /* FRAMESTATS_H_ */
