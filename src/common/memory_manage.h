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

// PMat2D is a 2D pointer matrix with [ROWS] rows and [COLS] columns. Each entry
// of the matrix is a pointer to an array of [T].
template <size_t ROWS, size_t COLS, class T> class PMat2D {
public:
    PMat2D() {}

    /// Create a pointer matrix with [n_entries] entries per matrix cell
    PMat2D(size_t n_entries) { alloc(n_entries); }

    ~PMat2D()
    {
        if (allocated) {
            for (auto& row : mat) {
                for (auto& entry : row) {
                    free(entry);
                }
            }
        }
    }

    /// Allocate [n_entries] entries per pointer cell in the pointer matrix
    void alloc(size_t n_entries)
    {
        for (auto& row : mat) {
            for (auto& entry : row) {
                entry
                    = reinterpret_cast<T*>(memalign(64, n_entries * sizeof(T)));
                memset(reinterpret_cast<uint8_t*>(entry), 0,
                    n_entries * sizeof(T));
            }
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

    /// Free the memory used by the pointer cells
    T** operator[](size_t row_idx)
    {
        return reinterpret_cast<T**>(&mat[row_idx][0]);
    }

private:
    std::array<std::array<T*, COLS>, ROWS> mat; /// The pointer cells

    /// True iff we've allocated memory for the pointer cells
    bool allocated = false;
};

#endif
