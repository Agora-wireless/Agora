#ifndef WORKER_BUFFERS_H
#define WORKER_BUFFERS_H

#include "buffer.h"

template<class T>
union mem_tag_t {
  T *memory_;
  size_t tag_;

  explicit mem_tag_t(size_t tag): tag_(tag){}
  explicit mem_tag_t(T &memory): memory_(&memory){}
  explicit mem_tag_t(T *memory): memory_(memory){}
};

class ResultMemory {
  public:
  explicit ResultMemory():
    frame_id_(0),
    symbol_id_(0),
    ant_id_(0) {}

  explicit ResultMemory(uint32_t frame_id, uint32_t symbol_id, uint32_t ant_id):
    frame_id_(frame_id),
    symbol_id_(symbol_id),
    ant_id_(ant_id) {}

  void Set(uint32_t frame_id, uint32_t symbol_id, uint32_t ant_id) {
    frame_id_ = frame_id;
    symbol_id_ = symbol_id;
    ant_id_ = ant_id;
  }

  virtual void *RawData() {
    throw std::runtime_error("ResultMemory RawResult should not be called");
  }

  uint32_t frame_id_;
  uint32_t symbol_id_;
  uint32_t ant_id_;
};
static_assert(sizeof(mem_tag_t<ResultMemory>) == sizeof(size_t));

class FFTResult : public ResultMemory {
  private:
  void *data_;

  public:
  explicit FFTResult():
    ResultMemory(),
    data_(nullptr) {}

  explicit FFTResult(uint32_t frame_id, uint32_t symbol_id, uint32_t ant_id):
    ResultMemory(frame_id, symbol_id, ant_id),
    data_(nullptr) {}

  void Set(uint32_t frame_id, uint32_t symbol_id, uint32_t ant_id, void *data) {
    ResultMemory::Set(frame_id, symbol_id, ant_id);
    data_ = data;
  }

  void *RawData() override {
    return data_;
  }
};
static_assert(sizeof(mem_tag_t<FFTResult>) == sizeof(size_t));

#endif
