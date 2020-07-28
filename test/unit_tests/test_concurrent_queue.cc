#include "concurrentqueue.h"
#include <gtest/gtest.h>
#include <thread>

static constexpr size_t kNumWorkers = 14;
static constexpr size_t kMaxTestNum = (1 << 24);

struct item_t {
    size_t value;
    size_t padding[7];

    item_t(){};
    item_t(size_t value)
        : value(value)
    {
    }
};
static_assert(sizeof(item_t) == 64, "");

// Test if the master can enqueue to specific workers
void MasterToWorkerStatic_master(moodycamel::ConcurrentQueue<item_t>* queue,
    moodycamel::ProducerToken** ptoks)
{
    for (size_t i = 0; i < kMaxTestNum; i++) {
        queue->enqueue(*ptoks[i % kNumWorkers], item_t(i));
    }
}

void MasterToWorkerStatic_worker(size_t worker_id,
    moodycamel::ConcurrentQueue<item_t>* queue, moodycamel::ProducerToken* ptok)
{
    size_t next_expected = worker_id;
    while (next_expected < kMaxTestNum) {
        item_t item;
        if (queue->try_dequeue_from_producer(*ptok, item)) {
            ASSERT_EQ(item.value, next_expected);
            next_expected += kNumWorkers;
        }
    }
}

TEST(TestConcurrentQueue, MasterToWorkerStatic)
{
    moodycamel::ConcurrentQueue<item_t> queue;
    moodycamel::ProducerToken* ptoks[kNumWorkers];
    for (size_t i = 0; i < kNumWorkers; i++) {
        ptoks[i] = new moodycamel::ProducerToken(queue);
    }

    auto master = std::thread(MasterToWorkerStatic_master, &queue, ptoks);
    std::thread workers[kNumWorkers];
    for (size_t i = 0; i < kNumWorkers; i++) {
        workers[i]
            = std::thread(MasterToWorkerStatic_worker, i, &queue, ptoks[i]);
    }
    master.join();
    for (auto& w : workers)
        w.join();

    for (size_t i = 0; i < kNumWorkers; i++) {
        delete ptoks[i];
    }
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}