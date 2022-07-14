/**
 * @file radio_data_plane_soapy.h
 * @brief Declaration file for the RadioDataPlaneSoapy Class
 */
#ifndef RADIO_DATA_PLANE_SOAPY_H_
#define RADIO_DATA_PLANE_SOAPY_H_

#include "radio_data_plane.h"

class RadioDataPlaneSoapy : public RadioDataPlane {
 public:
  RadioDataPlaneSoapy();
  // Allow move and disallow copy
  RadioDataPlaneSoapy(RadioDataPlaneSoapy&&) noexcept = default;
  explicit RadioDataPlaneSoapy(const RadioDataPlaneSoapy&) = delete;
  ~RadioDataPlaneSoapy() final = default;

  void Init(Radio* radio, const Config* cfg, bool hw_framer) final;
  void Setup() final;
  void Activate(Radio::ActivationTypes type = Radio::ActivationTypes::kActivate,
                long long act_time_ns = 0, size_t samples = 0) final;
  void Deactivate() final;
  void Close() final;

  int Rx(std::vector<std::vector<std::complex<int16_t>>>& rx_data,
         size_t rx_size, Radio::RxFlags& out_flags,
         long long& rx_time_ns) final;

  int Rx(std::vector<std::vector<std::complex<int16_t>>*>& rx_buffs,
         size_t rx_size, Radio::RxFlags& out_flags,
         long long& rx_time_ns) final;

  int Rx(std::vector<void*>& rx_locations, size_t rx_size,
         Radio::RxFlags& out_flags, long long& rx_time_ns) final;

  void Flush() final;

 private:
};
#endif  // RADIO_DATA_PLANE_SOAPY_H_
