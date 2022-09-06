// Copyright (c) 2018-2020, Rice University
// RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

/**
 * @file framestats.h
 * @brief Class defination for frame tracking
 * @author Rice University
 */

#ifndef FRAMESTATS_H_
#define FRAMESTATS_H_

#include <string>
#include <vector>

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
  size_t NumTotalSyms() const;

  /* Returns SIZE_MAX if symbol number is not a beacon */
  size_t GetBeaconSymbolIdx(size_t symbol_number) const;
  size_t GetDLSymbol(size_t location) const;
  inline size_t GetDLDataSymbol(size_t location) const {
    return GetDLSymbol(location + client_dl_pilot_symbols_);
  }
  inline size_t GetDLSymbolLast() const {
    return ((this->dl_symbols_.empty()) ? SIZE_MAX : this->dl_symbols_.back());
  }
  /* Returns SIZE_MAX if there are no DL symbols */
  size_t GetDLSymbolIdx(size_t symbol_number) const;

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

 private:
  std::string frame_identifier_;

  std::vector<size_t> beacon_symbols_;
  std::vector<size_t> pilot_symbols_;
  std::vector<size_t> ul_symbols_;
  std::vector<size_t> ul_cal_symbols_;
  std::vector<size_t> dl_symbols_;
  std::vector<size_t> dl_cal_symbols_;

  size_t client_ul_pilot_symbols_;
  size_t client_dl_pilot_symbols_;
  size_t client_ul_data_symbols_;
  size_t client_dl_data_symbols_;

  /* Helper function */
  static size_t GetSymbolIdx(const std::vector<size_t>& search_vector,
                             size_t symbol_number);
}; /* class FrameStats */

#endif /* FRAMESTATS_H_ */
