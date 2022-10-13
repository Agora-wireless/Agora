/**
 * @file test_armadillo.cc
 * @brief Utility functions for file and text processing.
 */

#include <gtest/gtest.h>

#include <memory>

#include "armadillo"
#include "memory_manage.h"

static void CheckArmaMemoryState(arma::uhword state, bool dynamic) {
  const size_t x_dim = 100;
  const size_t y_dim = 100;
  const size_t memory_size = x_dim * y_dim * sizeof(arma::cx_float);

  bool strict = false;
  bool copy_aux_memory = true;
  if (state == 1) {
    copy_aux_memory = false;
  } else if (state == 2) {
    strict = true;
    copy_aux_memory = false;
  }
  auto* storage =
      PaddedAlignedAlloc(Agora_memory::Alignment_t::kAlign64, memory_size);

  arma::cx_fmat test_matrix;
  arma::cx_fmat* test_matrix_ptr = nullptr;
  std::unique_ptr<arma::cx_fmat> mem_clean_ptr;
  if (dynamic == false) {
    test_matrix =
        std::move(arma::cx_fmat(reinterpret_cast<arma::cx_float*>(storage),
                                x_dim, y_dim, copy_aux_memory, strict));
    test_matrix_ptr = &test_matrix;
  } else {
    mem_clean_ptr = std::make_unique<arma::cx_fmat>(
        reinterpret_cast<arma::cx_float*>(storage), x_dim, y_dim,
        copy_aux_memory, strict);
    test_matrix_ptr = mem_clean_ptr.get();
  }

  arma::uhword memory_state = test_matrix_ptr->mem_state;
  arma::uword matrix_elements = test_matrix_ptr->n_elem;
  arma::uword alloc_elements = test_matrix_ptr->n_alloc;

  std::printf("Memory state %d\n", memory_state);
  EXPECT_EQ(memory_state, state);
  EXPECT_EQ(matrix_elements, x_dim * y_dim);
  if (state == 0) {
    EXPECT_EQ(alloc_elements, matrix_elements);
  } else {
    EXPECT_EQ(test_matrix_ptr->memptr(), storage);
    EXPECT_EQ(alloc_elements, 0);
  }
  std::free(storage);
}

//EXPECT_FALSE // EXPECT_TRUE / EXPECT_EQ
TEST(TestArmadillo, CorrectnessStackMemoryState0) {
  CheckArmaMemoryState(0, false);
}
TEST(TestArmadillo, CorrectnessStackMemoryState1) {
  CheckArmaMemoryState(1, false);
}
TEST(TestArmadillo, CorrectnessStackMemoryState2) {
  CheckArmaMemoryState(2, false);
}
TEST(TestArmadillo, CorrectnessHeapMemoryState0) {
  CheckArmaMemoryState(0, true);
}
TEST(TestArmadillo, CorrectnessHeapMemoryState1) {
  CheckArmaMemoryState(1, true);
}
TEST(TestArmadillo, CorrectnessHeapMemoryState2) {
  CheckArmaMemoryState(2, true);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}