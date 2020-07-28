#include <gtest/gtest.h>
// For some reason, gtest include order matters
#include "concurrentqueue.h"
#include "config.hpp"
#include "dozf.hpp"
#include "gettime.h"
#include "mac_thread.hpp"
#include "utils.h"

TEST(TestMacThread, Constructor)
{
    static constexpr size_t kNumIters = 10000;
    auto* cfg = new Config("data/tddconfig-sim-ul.json");
    cfg->genData();

    auto rx_queue = moodycamel::ConcurrentQueue<Event_data>(2 * kNumIters);
    auto tx_queue = moodycamel::ConcurrentQueue<Event_data>(2 * kNumIters);

    MacThread mac_thread(
        MacThread::Mode::kServer, cfg, 0 /* core offset */, nullptr, nullptr,
        nullptr, nullptr /* ul and dl bufs */, &rx_queue, &tx_queue, ""
        /* log filename */);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
