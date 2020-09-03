#include "memory_manage.h"
#include <gtest/gtest.h>

static constexpr size_t kRows = 40;
static constexpr size_t kCols = 1200;
static constexpr size_t kCol2s = 64;
const size_t n_entries = 64 * 32;

TEST(TestPtrGrid, Basic)
{
    PtrGrid<kRows, kCols, float> ptr_grid(n_entries);

    float sum = 0;
    for (size_t i = 0; i < kRows; i++) {
        for (size_t j = 0; j < kCols; j++) {
            sum += ptr_grid[i][j][0] + ptr_grid[i][j][n_entries - 1];
        }
    }

    ASSERT_EQ(sum, 0.0);
}

TEST(TestPtrCube, Basic)
{
    PtrCube<kRows, kCols, kCol2s, float> ptr_cube(n_entries);

    float sum = 0;
    for (size_t i = 0; i < kRows; i++) {
        for (size_t j = 0; j < kCols; j++) {
            for (size_t k = 0; k < kCol2s; k++) {
                sum += ptr_cube[i][j][k][0] + ptr_cube[i][j][k][n_entries - 1];
            }
        }
    }

    ASSERT_EQ(sum, 0.0);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}