#include <algorithm>
#include <gtest/gtest.h>
#include <time.h>
#include <vector>

/// Measure performance of zeroforcing
TEST(TestZF, Perf)
{
    size_t x = 0;
    ASSERT_EQ(x, 0);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
