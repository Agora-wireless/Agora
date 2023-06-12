#include "memory_manage.h"

namespace Agora_memory {
inline size_t PaddedAllocSize(Alignment_t alignment, size_t size) {
  auto align = static_cast<size_t>(alignment);
  size_t padded_size = size;
  size_t padding = align - (size % align);

  if (padding < align) {
    padded_size += padding;
  }
  return padded_size;
}

void* PaddedAlignedAlloc(Alignment_t alignment, size_t size) {
  return std::aligned_alloc(static_cast<size_t>(alignment),
                            PaddedAllocSize(alignment, size));
}
};  // namespace Agora_memory