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

// Test if the token affects the performance when master dequeues items from workers
void WorkerToMaster_master(moodycamel::ConcurrentQueue<item_t>* queue)
{
    item_t item;
    size_t sum = 0;
    while (sum < kMaxTestNum) {
        if (queue->try_dequeue(item)) {
            sum++;
        }
    }
}

void WorkerToMaster_worker_with_token(size_t worker_id,
    moodycamel::ConcurrentQueue<item_t>* queue, moodycamel::ProducerToken* ptok)
{
    size_t next_expected = worker_id;
    while (next_expected < kMaxTestNum) {
        item_t item(next_expected);
        if (queue->enqueue(*ptok, item)) {
            next_expected += kNumWorkers;
        }
    }
}

void WorkerToMaster_worker_without_token(
    size_t worker_id, moodycamel::ConcurrentQueue<item_t>* queue)
{
    size_t next_expected = worker_id;
    while (next_expected < kMaxTestNum) {
        item_t item(next_expected);
        if (queue->enqueue(item)) {
            next_expected += kNumWorkers;
        }
    }
}

TEST(TestConcurrentQueue, WorkerToMasterWithTokens)
{
    moodycamel::ConcurrentQueue<item_t> queue;
    moodycamel::ProducerToken* ptoks[kNumWorkers];
    for (size_t i = 0; i < kNumWorkers; i++) {
        ptoks[i] = new moodycamel::ProducerToken(queue);
    }

    auto master = std::thread(WorkerToMaster_master, &queue);
    std::thread workers[kNumWorkers];
    for (size_t i = 0; i < kNumWorkers; i++) {
        workers[i] = std::thread(
            WorkerToMaster_worker_with_token, i, &queue, ptoks[i]);
    }
    master.join();
    for (auto& w : workers)
        w.join();

    for (size_t i = 0; i < kNumWorkers; i++) {
        delete ptoks[i];
    }
}

TEST(TestConcurrentQueue, WorkerToMasterWithoutTokens)
{
    moodycamel::ConcurrentQueue<item_t> queue;

    auto master = std::thread(WorkerToMaster_master, &queue);
    std::thread workers[kNumWorkers];
    for (size_t i = 0; i < kNumWorkers; i++) {
        workers[i]
            = std::thread(WorkerToMaster_worker_without_token, i, &queue);
    }
    master.join();
    for (auto& w : workers)
        w.join();
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}