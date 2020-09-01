#include "memory_manage.h"
#include <gtest/gtest.h>

TEST(TestPtrGrid, Basic)
{
    static constexpr size_t kRows = 40;
    static constexpr size_t kCols = 1200;
    const size_t n_entries = 64 * 32;

    PtrGrid<kRows, kCols, float> ptr_grid(n_entries);

    float sum = 0;
    for (size_t i = 0; i < kRows; i++) {
        for (size_t j = 0; j < kCols; j++) {
            sum += ptr_grid[i][j][0];
        }
    }

    ASSERT_EQ(sum, 0.0);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}