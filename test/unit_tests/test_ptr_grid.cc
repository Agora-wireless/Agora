#include <gtest/gtest.h>
#define private public
#include "memory_manage.h"

static constexpr size_t kRows = 4;
static constexpr size_t kCols = 12;
static constexpr size_t kCol2s = 64;
const size_t kNEntries = 64;

TEST(TestPtrGrid, Basic) {
  PtrGrid<kRows, kCols, float> ptr_grid(kNEntries);

  // Test basic accesses and zero-initialization
  float sum = 0;
  for (size_t i = 0; i < kRows; i++) {
    for (size_t j = 0; j < kCols; j++) {
      sum += ptr_grid[i][j][0] + ptr_grid[i][j][kNEntries - 1];
    }
  }

  ASSERT_EQ(sum, 0.0);

  // Test that [] operator returns a reference, not a copy
  ptr_grid[0][0] = nullptr;
  ASSERT_EQ(ptr_grid.mat_[0][0], nullptr);
}

TEST(TestPtrCube, Basic) {
  PtrCube<kRows, kCols, kCol2s, float> ptr_cube(kNEntries);

  // Test basic accesses and zero-initialization
  float sum = 0;
  for (size_t i = 0; i < kRows; i++) {
    for (size_t j = 0; j < kCols; j++) {
      for (size_t k = 0; k < kCol2s; k++) {
        sum += ptr_cube[i][j][k][0] + ptr_cube[i][j][k][kNEntries - 1];
      }
    }
  }

  ASSERT_EQ(sum, 0.0);

  // Test that [] operator returns a reference, not a copy
  ptr_cube[0][0][0] = nullptr;
  ASSERT_EQ(ptr_cube.cube_[0][0][0], nullptr);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}