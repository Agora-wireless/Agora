#include <gtest/gtest.h>

#include <thread>

#include "concurrentqueue.h"

static constexpr size_t kNumWorkers = 14;
static constexpr size_t kMaxTestNum = (1 << 24);

struct ItemT {
  size_t value_;
  size_t padding_[7];

  ItemT() = default;
  ;
  explicit ItemT(size_t value) : value_(value) {}
};
static_assert(sizeof(ItemT) == 64);

// Test if the master can enqueue to specific workers
void MasterToWorkerStaticMaster(moodycamel::ConcurrentQueue<ItemT>* queue,
                                moodycamel::ProducerToken** ptoks) {
  for (size_t i = 0; i < kMaxTestNum; i++) {
    queue->enqueue(*ptoks[i % kNumWorkers], ItemT(i));
  }
}

void MasterToWorkerStaticWorker(size_t worker_id,
                                moodycamel::ConcurrentQueue<ItemT>* queue,
                                moodycamel::ProducerToken* ptok) {
  size_t next_expected = worker_id;
  while (next_expected < kMaxTestNum) {
    ItemT item;
    if (queue->try_dequeue_from_producer(*ptok, item)) {
      ASSERT_EQ(item.value_, next_expected);
      next_expected += kNumWorkers;
    }
  }
}

TEST(TestConcurrentQueue, MasterToWorkerStatic) {
  moodycamel::ConcurrentQueue<ItemT> queue;
  moodycamel::ProducerToken* ptoks[kNumWorkers];
  for (auto& ptok : ptoks) {
    ptok = new moodycamel::ProducerToken(queue);
  }

  auto master = std::thread(MasterToWorkerStaticMaster, &queue, ptoks);
  std::thread workers[kNumWorkers];
  for (size_t i = 0; i < kNumWorkers; i++) {
    workers[i] = std::thread(MasterToWorkerStaticWorker, i, &queue, ptoks[i]);
  }
  master.join();
  for (auto& w : workers) {
    w.join();
  }

  for (auto& ptok : ptoks) {
    delete ptok;
  }
}

// Test if the token affects the performance when master dequeues items from
// workers
void WorkerToMasterMaster(moodycamel::ConcurrentQueue<ItemT>* queue) {
  ItemT item;
  size_t sum = 0;
  while (sum < kMaxTestNum) {
    if (queue->try_dequeue(item)) {
      sum++;
    }
  }
}

void WorkerToMasterWorkerWithToken(size_t worker_id,
                                   moodycamel::ConcurrentQueue<ItemT>* queue,
                                   moodycamel::ProducerToken* ptok) {
  size_t next_expected = worker_id;
  while (next_expected < kMaxTestNum) {
    ItemT item(next_expected);
    if (queue->enqueue(*ptok, item)) {
      next_expected += kNumWorkers;
    }
  }
}

void WorkerToMasterWorkerWithoutToken(
    size_t worker_id, moodycamel::ConcurrentQueue<ItemT>* queue) {
  size_t next_expected = worker_id;
  while (next_expected < kMaxTestNum) {
    ItemT item(next_expected);
    if (queue->enqueue(item)) {
      next_expected += kNumWorkers;
    }
  }
}

TEST(TestConcurrentQueue, WorkerToMasterWithTokens) {
  moodycamel::ConcurrentQueue<ItemT> queue;
  moodycamel::ProducerToken* ptoks[kNumWorkers];
  for (auto& ptok : ptoks) {
    ptok = new moodycamel::ProducerToken(queue);
  }

  auto master = std::thread(WorkerToMasterMaster, &queue);
  std::thread workers[kNumWorkers];
  for (size_t i = 0; i < kNumWorkers; i++) {
    workers[i] =
        std::thread(WorkerToMasterWorkerWithToken, i, &queue, ptoks[i]);
  }
  master.join();
  for (auto& w : workers) {
    w.join();
  }

  for (auto& ptok : ptoks) {
    delete ptok;
  }
}

TEST(TestConcurrentQueue, WorkerToMasterWithoutTokens) {
  moodycamel::ConcurrentQueue<ItemT> queue;

  auto master = std::thread(WorkerToMasterMaster, &queue);
  std::thread workers[kNumWorkers];
  for (size_t i = 0; i < kNumWorkers; i++) {
    workers[i] = std::thread(WorkerToMasterWorkerWithoutToken, i, &queue);
  }
  master.join();
  for (auto& w : workers) {
    w.join();
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}