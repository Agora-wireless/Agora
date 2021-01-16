#ifndef MEMORY_MANAGE
#define MEMORY_MANAGE
#include <array>
#include <cassert>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>

namespace Agora_memory {
enum class Alignment_t : size_t {
    k32Align = 32,
    k64Align = 64,
    k4096Align = 4096
};

void* padded_aligned_alloc(Alignment_t alignment, size_t size);
}

template <typename T> class Table {
private:
    size_t dim2_;
    size_t dim1_;
    T* data_;

public:
    Table(void)
        : dim2_(0)
        , dim1_(0)
        , data_(nullptr)
    {
    }

    void malloc(size_t dim1, size_t dim2, Agora_memory::Alignment_t alignment)
    {
        this->dim2_ = dim2;
        this->dim1_ = dim1;
        size_t alloc_size = (this->dim1_ * this->dim2_ * sizeof(T));
        this->data_ = static_cast<T*>(
            Agora_memory::padded_aligned_alloc(alignment, alloc_size));
    }
    void calloc(size_t dim1, size_t dim2, Agora_memory::Alignment_t alignment)
    {
        //assert(!((dim1 == 0) || (dim2 == 0)));
        this->malloc(dim1, dim2, alignment);
        std::memset(this->data_, 0, (this->dim1_ * this->dim2_ * sizeof(T)));
    }

    // Allocate the table and fill it with random floating point values between
    // -1.0 and 1.0
    void rand_alloc_float(
        size_t dim1, size_t dim2, Agora_memory::Alignment_t alignment)
    {
        std::default_random_engine generator;
        std::uniform_real_distribution<float> distribution(-1.0, 1.0);

        assert(sizeof(T) >= sizeof(float));
        this->malloc(dim1, dim2, alignment);
        auto* base = reinterpret_cast<float*>(this->data_);
        for (size_t i = 0; i < (dim1 * dim2); i++) {
            base[i] = distribution(generator);
        }
    }

    // Allocate the table and fill it with random complex floating point values
    // between -1.0 and 1.0
    void rand_alloc_cx_float(
        size_t dim1, size_t dim2, Agora_memory::Alignment_t alignment)
    {
        std::default_random_engine generator;
        std::uniform_real_distribution<float> distribution(-1.0, 1.0);

        assert(sizeof(T) >= sizeof(std::complex<float>));
        this->malloc(dim1, dim2, alignment);
        auto* base = reinterpret_cast<std::complex<float>*>(this->data_);
        for (size_t i = 0; i < (dim1 * dim2); i++) {
            base[i] = { distribution(generator), distribution(generator) };
        }
    }

    bool is_allocated(void) { return (this->data_ != nullptr); }

    void free(void)
    {
        if (this->data_ != nullptr) {
            std::free(this->data_);
        }
        this->dim2_ = 0;
        this->dim1_ = 0;
        this->data_ = nullptr;
    }

    T* at(size_t dim1) const { return (*this)[dim1]; }

    T* operator[](size_t dim1)
    {
        assert(this->dim1_ > dim1);
        return (this->data_ + (dim1 * this->dim2_));
    }
};

template <typename T, typename U>
static void alloc_buffer_1d(
    T** buffer, U dim, Agora_memory::Alignment_t alignment, int init_zero)
{
    size_t size = dim * sizeof(T);
    *buffer
        = static_cast<T*>(Agora_memory::padded_aligned_alloc(alignment, size));
    if (init_zero) {
        std::memset(*buffer, 0u, size);
    }
};

template <typename T> static void free_buffer_1d(T** buffer)
{
    std::free(*buffer);
};

// PtrGrid is a 2D grid of pointers with [ROWS] rows and [COLS] columns. Each
// entry of the grid is a pointer to an array of [T].
template <size_t ROWS, size_t COLS, class T> class PtrGrid {
public:
    PtrGrid(void)
        : backing_buf_(nullptr)
    {
    }

    /// Create a grid of pointers where each grid cell points to an array of
    /// [n_entries]
    PtrGrid(size_t n_entries) { this->alloc(ROWS, COLS, n_entries); }

    /// Create a grid of pointers with dimensions [ROWS, COLS], where
    /// only the grid with dimensions [n_rows, n_cols] has cells pointing to an
    /// array of [n_entries]. This can use less memory than a fully-allocated
    /// grid.
    PtrGrid(size_t n_rows, size_t n_cols, size_t n_entries)
    {
        assert(n_rows <= ROWS && n_cols <= COLS);
        this->alloc(n_rows, n_cols, n_entries);
    }

    ~PtrGrid(void)
    {
        if (this->backing_buf_ != nullptr) {
            std::free(this->backing_buf_);
            this->backing_buf_ = nullptr;
        }
    }

    /// Allocate [n_entries] entries per pointer cell
    void alloc(size_t n_rows, size_t n_cols, size_t n_entries)
    {
        const size_t alloc_sz = n_rows * n_cols * n_entries * sizeof(T);
        this->backing_buf_ = static_cast<T*>(Agora_memory::padded_aligned_alloc(
            Agora_memory::Alignment_t::k64Align, alloc_sz));
        std::memset(static_cast<void*>(this->backing_buf_), 0, alloc_sz);

        // Fill-in the grid with pointers into backing_buf
        size_t offset = 0;
        for (size_t i = 0; i < n_rows; i++) {
            for (size_t j = 0; j < n_cols; j++) {
                mat_[i][j] = &this->backing_buf_[offset];
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
        alloc(ROWS, COLS, n_entries);

        std::default_random_engine generator;
        std::uniform_real_distribution<float> distribution(-1.0, 1.0);

        for (auto& row : mat_) {
            for (auto& entry : row) {
                auto* base = reinterpret_cast<float*>(entry);
                for (size_t i = 0; i < n_entries * 2; i++) {
                    base[i] = distribution(generator);
                }
            }
        }
    }

    std::array<T*, COLS>& operator[](size_t row_idx)
    {
        return this->mat_[row_idx];
    }

    // Delete copy constructor and copy assignment
    PtrGrid(PtrGrid const&) = delete;
    PtrGrid& operator=(PtrGrid const&) = delete;

private:
    std::array<std::array<T*, COLS>, ROWS> mat_; /// The pointer cells

    /// The backing buffer for the per-cell arrays. Having a common buffer
    /// reduces the number of memory allocations.
    T* backing_buf_;
};

// PtrCube is a 3D cube of pointers. Each entry of the cube is a pointer to an
// array of [T].
template <size_t DIM1, size_t DIM2, size_t DIM3, class T> class PtrCube {
public:
    PtrCube(void)
        : backing_buf_(nullptr)
    {
    }

    /// Create a cube of pointers with dimensions [DIM1, DIM2, DIM3], where each
    /// cube cell points to an array of [n_entries]
    PtrCube(size_t n_entries) { this->alloc(DIM1, DIM2, DIM3, n_entries); }

    /// Create a cube of pointers with dimensions [DIM1, DIM2, DIM3], where
    /// only the cube with dimensions [dim_1, dim_2, dim_3] has cells
    /// pointing to an array of [n_entries]. This can use less memory than a
    /// fully-allocated cube.
    PtrCube(size_t dim_1, size_t dim_2, size_t dim_3, size_t n_entries)
    {
        assert(dim_1 <= DIM1 && dim_2 <= DIM2 && dim_3 <= DIM3);
        this->alloc(dim_1, dim_2, dim_3, n_entries);
    }

    ~PtrCube(void)
    {
        if (this->backing_buf_ != nullptr) {
            std::free(this->backing_buf_);
            this->backing_buf_ = nullptr;
        }
    }

    /// Allocate [n_entries] entries per pointer cell
    void alloc(size_t dim_1, size_t dim_2, size_t dim_3, size_t n_entries)
    {
        const size_t alloc_sz = dim_1 * dim_2 * dim_3 * n_entries * sizeof(T);
        this->backing_buf_ = static_cast<T*>(Agora_memory::padded_aligned_alloc(
            Agora_memory::Alignment_t::k64Align, alloc_sz));
        std::memset(static_cast<void*>(this->backing_buf_), 0, alloc_sz);

        // Fill-in the grid with pointers into backing_buf
        for (auto& mat : this->cube_) {
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
                    this->cube_[i][j][k] = &this->backing_buf_[offset];
                    offset += n_entries;
                }
            }
        }
    }

    std::array<std::array<T*, DIM3>, DIM2>& operator[](size_t row_idx)
    {
        return this->cube_[row_idx];
    }

    // Delete copy constructor and copy assignment
    PtrCube(PtrCube const&) = delete;
    PtrCube& operator=(PtrCube const&) = delete;

private:
    /// The pointer cells
    std::array<std::array<std::array<T*, DIM3>, DIM2>, DIM1> cube_;

    /// The backing buffer for the per-cell arrays. Having a common buffer
    /// reduces the number of memory allocations.
    T* backing_buf_;
};

#endif
