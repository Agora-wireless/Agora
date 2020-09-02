#include "memory_manage.h"
#include <gtest/gtest.h>

static constexpr size_t kRows = 40;
static constexpr size_t kCols = 1200;
const size_t n_entries = 64 * 32;

void dummy_ptr_grid_function(PtrGrid<kRows, kCols, float>& ptr_grid)
{
    ptr_grid[0][0][0]++;
}

TEST(TestPtrGrid, Basic)
{
    PtrGrid<kRows, kCols, float> ptr_grid(n_entries);

    float sum = 0;
    for (size_t i = 0; i < kRows; i++) {
        for (size_t j = 0; j < kCols; j++) {
            sum += ptr_grid[i][j][0] + ptr_grid[i][j][n_entries - 1];
        }
    }

    dummy_ptr_grid_function(ptr_grid);
    ASSERT_EQ(sum, 0.0);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}