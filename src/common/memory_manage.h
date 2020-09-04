/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#ifndef MEMORY_MANAGE
#define MEMORY_MANAGE
#include <array>
#include <cstdlib>
#include <cstring>
#include <malloc.h>
#include <random>
#include <stdio.h>

template <typename T> class Table {
private:
    size_t dimension;
    void* data;

public:
    Table(void)
        : dimension(0)
        , data(NULL)
    {
    }
    void malloc(size_t dim1, size_t dim2, size_t aligned_bytes)
    {
        aligned_bytes = (aligned_bytes) / 32 * 32;
        dimension = (dim2 * sizeof(T) + aligned_bytes - 1) & -aligned_bytes;
        data = aligned_alloc(aligned_bytes, dim1 * dimension);
    }
    void calloc(size_t dim1, size_t dim2, size_t aligned_bytes)
    {
        malloc(dim1, dim2, aligned_bytes);
        memset(data, 0, dim1 * dimension);
    }

    // Allocate the table and fill it with random floating point values between
    // -1.0 and 1.0
    void rand_alloc_float(size_t dim1, size_t dim2, size_t aligned_bytes)
    {
        std::default_random_engine generator;
        std::uniform_real_distribution<float> distribution(-1.0, 1.0);

        malloc(dim1, dim2, aligned_bytes);
        auto* base = reinterpret_cast<float*>(data);
        for (size_t i = 0; i < dim1 * dim2; i++) {
            base[i] = distribution(generator);
        }
    }

    // Allocate the table and fill it with random complex floating point values
    // between -1.0 and 1.0
    void rand_alloc_cx_float(size_t dim1, size_t dim2, size_t aligned_bytes)
    {
        std::default_random_engine generator;
        std::uniform_real_distribution<float> distribution(-1.0, 1.0);

        malloc(dim1, dim2, aligned_bytes);
        auto* base = reinterpret_cast<float*>(data);
        for (size_t i = 0; i < dim1 * dim2 * 2; i++) {
            base[i] = distribution(generator);
        }
    }

    bool is_allocated() { return dimension > 0; }

    void free(void)
    {
        std::free(data);
        dimension = 0;
        data = NULL;
    }

    T* operator[](int dim1) { return (T*)((char*)data + dim1 * dimension); }
};

template <typename T, typename U>
static void alloc_buffer_1d(T** buffer, U dim, int aligned_bytes, int init_zero)
{
    aligned_bytes = (aligned_bytes) / 32 * 32;
    *buffer = (T*)aligned_alloc(aligned_bytes, dim * sizeof(T));
    if (init_zero)
        memset(*buffer, 0, dim * sizeof(T));
};

template <typename T> static void free_buffer_1d(T** buffer) { free(*buffer); };

// PtrGrid is a 2D grid of pointers with [ROWS] rows and [COLS] columns. Each
// entry of the grid is a pointer to an array of [T].
template <size_t ROWS, size_t COLS, class T> class PtrGrid {
public:
    PtrGrid() {}

    /// Create a grid of pointers where each grid cell points to an array of
    /// [n_entries]
    PtrGrid(size_t n_entries) { alloc(n_entries); }

    ~PtrGrid()
    {
        if (is_allocated)
            free(backing_buf);
    }

    /// Allocate [n_entries] entries per pointer cell in the pointer matrix
    void alloc(size_t n_entries)
    {
        const size_t alloc_sz = ROWS * COLS * n_entries * sizeof(T);
        backing_buf = reinterpret_cast<T*>(memalign(64, alloc_sz));
        memset(reinterpret_cast<uint8_t*>(backing_buf), 0, alloc_sz);
        is_allocated = true;

        // Fill-in the grid with pointers into backing_buf
        size_t offset = 0;
        for (auto& row : mat) {
            for (auto& entry : row) {
                entry = &backing_buf[offset];
                offset += n_entries;
            }
        }
        if (offset * sizeof(T) != alloc_sz) {
            fprintf(stderr, "Error Error Error\n");
            exit(-1);
        }
    }

    /// Allocate [n_entries] entries per pointer cell in the pointer matrix.
    /// Each entry is a random float between -1.0 and 1.0.
    void rand_alloc_cx_float(size_t n_entries)
    {
        static_assert(
            sizeof(T) == 2 * sizeof(float), "T must be complex_float");
        alloc(n_entries);

        std::default_random_engine generator;
        std::uniform_real_distribution<float> distribution(-1.0, 1.0);

        for (auto& row : mat) {
            for (auto& entry : row) {
                auto* base = reinterpret_cast<float*>(entry);
                for (size_t i = 0; i < n_entries * 2; i++) {
                    base[i] = distribution(generator);
                }
            }
        }
    }

    T** operator[](size_t row_idx)
    {
        return reinterpret_cast<T**>(&mat[row_idx][0]);
    }

    // Delete copy constructor and copy assignment
    PtrGrid(PtrGrid const&) = delete;
    PtrGrid& operator=(PtrGrid const&) = delete;

private:
    std::array<std::array<T*, COLS>, ROWS> mat; /// The pointer cells

    /// The backing buffer for the per-cell arrays. Having a common buffer
    /// reduces the number of memory allocations.
    T* backing_buf;

    /// True iff we've allocated the backing buffer
    bool is_allocated = false;
};

#endif
