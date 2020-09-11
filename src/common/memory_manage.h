#ifndef MEMORY_MANAGE
#define MEMORY_MANAGE
#include <array>
#include <assert.h>
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

    /// Allocate [n_entries] entries per pointer cell
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
    }

    /// Allocate [n_entries] entries per pointer cell.
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

    std::array<T*, COLS> operator[](size_t row_idx) { return mat[row_idx]; }

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

// PtrCube is a 3D cube of pointers. Each entry of the cube is a pointer to an
// array of [T].
template <size_t DIM1, size_t DIM2, size_t DIM3, class T> class PtrCube {
public:
    PtrCube() {}

    /// Create a cube of pointers with dimensions [DIM1, DIM2, DIM3], where each
    /// cube cell points to an array of [n_entries]
    PtrCube(size_t n_entries) { alloc(DIM1, DIM2, DIM3, n_entries); }

    /// Create a cube of pointers with dimensions [DIM1, DIM2, DIM3], where
    /// only the cube with dimensions [dim_1, dim_2, dim_3] has cells
    /// pointing to an array of [n_entries]. This can use less memory than a
    /// fully-allocated cube.
    PtrCube(size_t dim_1, size_t dim_2, size_t dim_3, size_t n_entries)
    {
        assert(dim_1 <= DIM1 && dim_2 <= DIM2 && dim_3 <= DIM3);
        alloc(dim_1, dim_2, dim_3, n_entries);
    }

    ~PtrCube()
    {
        if (is_allocated)
            free(backing_buf);
    }

    /// Allocate [n_entries] entries per pointer cell
    void alloc(size_t dim_1, size_t dim_2, size_t dim_3, size_t n_entries)
    {
        const size_t alloc_sz = dim_1 * dim_2 * dim_3 * n_entries * sizeof(T);
        backing_buf = reinterpret_cast<T*>(memalign(64, alloc_sz));
        memset(reinterpret_cast<uint8_t*>(backing_buf), 0, alloc_sz);
        is_allocated = true;

        // Fill-in the grid with pointers into backing_buf
        for (auto& mat : cube) {
            for (auto& row : mat) {
                for (auto& entry : row) {
                    entry = nullptr;
                }
            }
        }

        size_t offset = 0;
        for (size_t i = 0; i < dim_1; i++) {
            for (size_t j = 0; j < dim_2; j++) {
                for (size_t k = 0; k < dim_3; k++) {
                    cube[i][j][k] = &backing_buf[offset];
                    offset += n_entries;
                }
            }
        }
    }

    std::array<std::array<T*, DIM3>, DIM2> operator[](size_t row_idx)
    {
        return cube[row_idx];
    }

    // Delete copy constructor and copy assignment
    PtrCube(PtrCube const&) = delete;
    PtrCube& operator=(PtrCube const&) = delete;

private:
    /// The pointer cells
    std::array<std::array<std::array<T*, DIM3>, DIM2>, DIM1> cube;

    /// The backing buffer for the per-cell arrays. Having a common buffer
    /// reduces the number of memory allocations.
    T* backing_buf;

    /// True iff we've allocated the backing buffer
    bool is_allocated = false;
};

#endif
