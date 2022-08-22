/**
 * @file message_box_thread.h
 * @brief Declaration file for MessageBoxThread class
 */
#ifndef MESSAGE_BOX_THREAD_H_
#define MESSAGE_BOX_THREAD_H_

#include <thread>

#include "concurrentqueue.h"
#include "message.H"

class MessageBoxThread {
 public:
  MessageBoxThread(int core_id, size_t thread_id, size_t queue_size);
  ~MessageBoxThread();

  void Start();
  void Stop();
  bool DispatchWork(const EventData &event);
  //bool Subscribe(moodycamel::ConcurrentQueue<EventData> &completion_q);

 private:
  void EventLoop();
  void HandleEvent(const EventData &event);
  void Finalize();

  moodycamel::ConcurrentQueue<EventData> event_queue_;
  moodycamel::ProducerToken producer_token_;
  std::thread thread_;

  size_t id_;

  /* >= 0 to assign a core to the thread
   * <0   to disable thread core assignment */
  int core_alloc_;

  //bool wait_signal_;
  //std::mutex sync_;
  //std::condition_variable condition_;
  bool running_;
};

#endif /* MESSAGE_BOX_THREAD_H_ */
