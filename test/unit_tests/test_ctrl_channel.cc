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
  Table<std::complex<int16_t>> data_buffer;
  data_buffer.Calloc(cfg->Frame().NumDLBcastSyms(), cfg->SampsPerSymbol(),
                     Agora_memory::Alignment_t::kAlign64);

  size_t msg = 2534;
  cfg->GenBroadcastSlots(data_buffer, msg);
  auto decoded_msg =
      cfg->DecodeBroadcastSlots(reinterpret_cast<int16_t*>(data_buffer[0]));
  ASSERT_EQ(msg, decoded_msg);
  data_buffer.Free();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
