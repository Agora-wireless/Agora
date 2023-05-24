/**
 * @file test_control_channel.cc
 * @brief Test function for generating and decoding the control channel symbols.
 */

#include <gtest/gtest.h>
// For some reason, gtest include order matters

#include "config.h"
#include "gettime.h"
#include "memory_manage.h"
#include "utils.h"

TEST(TestControl, VerifyCorrectness) {
  auto cfg =
      std::make_unique<Config>("files/config/ci/tddconfig-sim-ctrl.json");
  cfg->GenData();
  std::vector<std::complex<int16_t>*> data_buffer(
      cfg->Frame().NumDlControlSyms());
  for (size_t i = 0; i < cfg->Frame().NumDlControlSyms(); i++) {
    data_buffer.at(i) =
        static_cast<std::complex<int16_t>*>(Agora_memory::PaddedAlignedAlloc(
            Agora_memory::Alignment_t::kAlign64,
            2 * cfg->SampsPerSymbol() * sizeof(int16_t)));
  }

  std::vector<size_t> msg;
  msg.push_back(4);
  cfg->GenBroadcastSlots(data_buffer, msg);
  auto decoded_msg =
      cfg->DecodeBroadcastSlots(reinterpret_cast<int16_t*>(data_buffer[0]));
  ASSERT_EQ(msg.at(0), decoded_msg);
  for (size_t i = 0; i < cfg->Frame().NumDlControlSyms(); i++) {
    std::free(data_buffer.at(i));
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
