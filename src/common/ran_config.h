#ifndef RAN_CONFIG_H_
#define RAN_CONFIG_H_

#include <cstddef>

/**
 * @brief The struct that contains the RAN configuration that Agora must
 * apply for a particular frame.
 *
 * The RanConfig class contains the RAN parameters that Agora must configure
 * before processing a particular frame.
 * TODO: this is a work in progress.
 */
class RanConfig {
 public:
  size_t n_antennas_;      /// Number of active antennas at the base station
  size_t mod_order_bits_;  /// modulation type (number of bits)
  size_t frame_id_;        /// frame ID
};

/**
 * @brief The packet that contains the control information (DCI) that tells
 * each UE which uplink resource blocks it was scheduled.
 *
 * The RBIndicator class contains the information that a UE needs to determine
 * the particular resource blocks, coding rate, modulation, etc. that the
 * scheduler has assigned it.
 * TODO: this is a work in progress.
 */
class RBIndicator {
 public:
  size_t ue_id_;           /// UE ID
  size_t mod_order_bits_;  /// modulation type (number of bits)
};

#endif  // RAN_CONFIG_H_
