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

  virtual ~ResultMemory() = default;

  void Set(uint32_t frame_id, uint32_t symbol_id, uint32_t ant_id) {
    frame_id_ = frame_id;
    symbol_id_ = symbol_id;
    ant_id_ = ant_id;
  }

  inline size_t GetFrameID() const {
    return frame_id_;
  }

  inline size_t GetSymbolId() const {
    return symbol_id_;
  }

  inline size_t GetAntId() const {
    return ant_id_;
  }

  private:
  uint32_t frame_id_;
  uint32_t symbol_id_;
  uint32_t ant_id_;
};
static_assert(sizeof(mem_tag_t<ResultMemory>) == sizeof(size_t));

class FFTResult : public ResultMemory {
  private:
  float *data_;

  public:
  explicit FFTResult():
    ResultMemory(),
    data_(nullptr) {}

  explicit FFTResult(uint32_t frame_id, uint32_t symbol_id, uint32_t ant_id):
    ResultMemory(frame_id, symbol_id, ant_id),
    data_(nullptr) {}

  void Set(uint32_t frame_id, uint32_t symbol_id, uint32_t ant_id, float *data) {
    ResultMemory::Set(frame_id, symbol_id, ant_id);
    data_ = data;
  }

  float *RawData() {
    return data_;
  }
};
static_assert(sizeof(mem_tag_t<FFTResult>) == sizeof(size_t));

class DeMulResult : public ResultMemory {
  private:
  int8_t *data_;

  public:
  explicit DeMulResult():
    ResultMemory(),
    data_(nullptr) {}

  explicit DeMulResult(uint32_t frame_id, uint32_t symbol_id, uint32_t ant_id):
    ResultMemory(frame_id, symbol_id, ant_id),
    data_(nullptr) {}

  void Set(uint32_t frame_id, uint32_t symbol_id, uint32_t ant_id, int8_t *data) {
    ResultMemory::Set(frame_id, symbol_id, ant_id);
    data_ = data;
  }

  int8_t *RawData() {
    return data_;
  }
};
static_assert(sizeof(mem_tag_t<DeMulResult>) == sizeof(size_t));

#endif
