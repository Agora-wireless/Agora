#include <armadillo>
#include <complex.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

using namespace arma;

static double test_get_time(void)
{
    struct timespec tv;
    clock_gettime(CLOCK_MONOTONIC, &tv);
    return tv.tv_sec + tv.tv_nsec / 1000000000.0;
}


static double bench_ZF(unsigned Nx, unsigned Ny, unsigned iterations)
{
    srand(0);
    fmat real_H = randn<fmat>(Nx, Ny);
    fmat imag_H = randn<fmat>(Nx, Ny);
    cx_fmat mat_input(real_H, imag_H);
    cx_fmat mat_output(Ny, Nx);

    double start_time = test_get_time();
    for (unsigned i = 0; i < iterations; i++)
    {
        pinv(mat_output, mat_input, 1e-1, "dc");
    }
    double end_time = test_get_time();

    return end_time - start_time;
}

static double bench_multiply_dim1(unsigned Nx, unsigned Ny, unsigned iterations)
{
    srand(0);
    fmat real_right = randn<fmat>(Nx, Ny);
    fmat imag_right = randn<fmat>(Nx, Ny);
    fmat real_left = randn<fmat>(1, Nx);
    fmat imag_left = randn<fmat>(1, Nx);
    cx_fmat mat_right(real_right, imag_right);
    cx_fmat mat_left(real_left, imag_left);
    cx_fmat result(1, Ny);

    double start_time = test_get_time();
    for (unsigned i = 0; i < iterations; i++)
    {
        result = mat_left * mat_right;
    }
    double end_time = test_get_time();

    return end_time - start_time;
}

static double bench_multiply_dim2(unsigned Nx, unsigned Ny, unsigned iterations)
{
    srand(0);
    fmat real_left = randn<fmat>(Nx, Ny);
    fmat imag_left = randn<fmat>(Nx, Ny);
    fmat real_right = randn<fmat>(Ny, 1);
    fmat imag_right = randn<fmat>(Ny, 1);
    cx_fmat mat_right(real_right, imag_right);
    cx_fmat mat_left(real_left, imag_left);
    cx_fmat result(Nx, 1);

    double start_time = test_get_time();
    for (unsigned i = 0; i < iterations; i++)
    {
        result = mat_left * mat_right;
    }
    double end_time = test_get_time();

    return end_time - start_time;
}

static double bench_multiply_transpose(unsigned Nx, unsigned Ny, unsigned iterations)
{
    srand(0);
    fmat real_left = randn<fmat>(Nx, Ny);
    fmat imag_left = randn<fmat>(Nx, Ny);
    fmat real_right = randn<fmat>(Nx, 1);
    fmat imag_right = randn<fmat>(Nx, 1);
    cx_fmat mat_right(real_right, imag_right);
    cx_fmat mat_left(real_left, imag_left);
    cx_fmat result(Nx, 1);

    double start_time = test_get_time();
    for (unsigned i = 0; i < iterations; i++)
    {
        result = mat_left.t() * mat_right;
    }
    double end_time = test_get_time();

    return end_time - start_time;
}


static void run_benchmark_ZF(unsigned Nx, unsigned Ny, unsigned iterations)
{
    double zf_time1 = bench_ZF(Nx, Ny, iterations);
    double zf_time2 = bench_ZF(Nx, Ny, iterations);
    double zf_time3 = bench_ZF(Nx, Ny, iterations);
    double zf_time4 = bench_ZF(Nx, Ny, iterations);
    double zf_time5 = bench_ZF(Nx, Ny, iterations);

    printf("ZF:              %04u x %04u  %12.3f us iteration\n",
            Nx, Ny, 1000000.0 * zf_time1 / iterations);
    printf("ZF:              %04u x %04u  %12.3f us iteration\n",
            Nx, Ny, 1000000.0 * zf_time2 / iterations);
    printf("ZF:              %04u x %04u  %12.3f us iteration\n",
            Nx, Ny, 1000000.0 * zf_time3 / iterations);
    printf("ZF:              %04u x %04u  %12.3f us iteration\n",
            Nx, Ny, 1000000.0 * zf_time4 / iterations);
    printf("ZF:              %04u x %04u  %12.3f us iteration\n",
            Nx, Ny, 1000000.0 * zf_time5 / iterations);
}

static void run_benchmark_multiply(int dim, unsigned Nx, unsigned Ny, unsigned iterations)
{
    double mul_time1 = (dim == 1) ? bench_multiply_dim1(Nx, Ny, iterations) : bench_multiply_dim2(Nx, Ny, iterations);
    double mul_time2 = (dim == 1) ? bench_multiply_dim1(Nx, Ny, iterations) : bench_multiply_dim2(Nx, Ny, iterations);
    double mul_time3 = (dim == 1) ? bench_multiply_dim1(Nx, Ny, iterations) : bench_multiply_dim2(Nx, Ny, iterations);
    double mul_time4 = (dim == 1) ? bench_multiply_dim1(Nx, Ny, iterations) : bench_multiply_dim2(Nx, Ny, iterations);
    double mul_time5 = (dim == 1) ? bench_multiply_dim1(Nx, Ny, iterations) : bench_multiply_dim2(Nx, Ny, iterations);

    if (dim == 1)
        printf("(1 x %04u) * (%04u x %04u)\n", Nx, Nx, Ny);
    if (dim == 2) 
        printf("(%04u x %04u) * (%04u x 1)\n", Nx, Ny, Ny);
    printf("Multiply:              %04u x %04u  %12.3f us iteration\n",
            Nx, Ny, 1000000.0 * mul_time1 / iterations);
    printf("Multiply:              %04u x %04u  %12.3f us iteration\n",
            Nx, Ny, 1000000.0 * mul_time2 / iterations);
    printf("Multiply:              %04u x %04u  %12.3f us iteration\n",
            Nx, Ny, 1000000.0 * mul_time3 / iterations);
    printf("Multiply:              %04u x %04u  %12.3f us iteration\n",
            Nx, Ny, 1000000.0 * mul_time4 / iterations);
    printf("Multiply:              %04u x %04u  %12.3f us iteration\n",
            Nx, Ny, 1000000.0 * mul_time5 / iterations);
}


static void run_benchmark_precode(unsigned Nx, unsigned Ny, unsigned iterations)
{
    double mul_time1_equal = bench_multiply_dim2(Nx, Ny, iterations);
    double mul_time2_equal = bench_multiply_dim2(Nx, Ny, iterations);
    double mul_time3_equal = bench_multiply_dim2(Nx, Ny, iterations);
    double mul_time4_equal = bench_multiply_dim2(Nx, Ny, iterations);
    double mul_time5_equal = bench_multiply_dim2(Nx, Ny, iterations);

    double mul_time1_precode = bench_multiply_transpose(Nx, Ny, iterations);
    double mul_time2_precode = bench_multiply_transpose(Nx, Ny, iterations);
    double mul_time3_precode = bench_multiply_transpose(Nx, Ny, iterations);
    double mul_time4_precode = bench_multiply_transpose(Nx, Ny, iterations);
    double mul_time5_precode = bench_multiply_transpose(Nx, Ny, iterations);

    printf("Equalization: (%04u x %04u) * (%04u x 1)\n", Nx, Ny, Ny);
    printf("Multiply:              %04u x %04u  %12.3f us iteration\n",
            Nx, Ny, 1000000.0 * mul_time1_equal / iterations);
    printf("Multiply:              %04u x %04u  %12.3f us iteration\n",
            Nx, Ny, 1000000.0 * mul_time2_equal / iterations);
    printf("Multiply:              %04u x %04u  %12.3f us iteration\n",
            Nx, Ny, 1000000.0 * mul_time3_equal / iterations);
    printf("Multiply:              %04u x %04u  %12.3f us iteration\n",
            Nx, Ny, 1000000.0 * mul_time4_equal / iterations);
    printf("Multiply:              %04u x %04u  %12.3f us iteration\n",
            Nx, Ny, 1000000.0 * mul_time5_equal / iterations);

    printf("Precoding: (%04u x %04u) * (%04u x 1)\n", Ny, Nx, Nx);
    printf("Multiply:              %04u x %04u  %12.3f us iteration\n",
            Nx, Ny, 1000000.0 * mul_time1_precode / iterations);
    printf("Multiply:              %04u x %04u  %12.3f us iteration\n",
            Nx, Ny, 1000000.0 * mul_time2_precode / iterations);
    printf("Multiply:              %04u x %04u  %12.3f us iteration\n",
            Nx, Ny, 1000000.0 * mul_time3_precode / iterations);
    printf("Multiply:              %04u x %04u  %12.3f us iteration\n",
            Nx, Ny, 1000000.0 * mul_time4_precode / iterations);
    printf("Multiply:              %04u x %04u  %12.3f us iteration\n",
            Nx, Ny, 1000000.0 * mul_time5_precode / iterations);
}

int main(int argc, char *argv[])
{
    if (argc != 5)
    {
        fprintf(stderr, "Usage: %s [iterations] [Nx] [Ny] mode\n",
                argv[0]);
        return 1;
    }

	if (argc == 5)
    {
        unsigned iterations = strtoul(argv[1], NULL, 0);
        unsigned Nx = strtoul(argv[2], NULL, 0);
        unsigned Ny = strtoul(argv[3], NULL, 0);
        unsigned mode = strtoul(argv[4], NULL, 0);
        if (mode == 0)
            run_benchmark_ZF(Nx, Ny, iterations);
        else if (mode == 1)
            run_benchmark_multiply(1, Nx, Ny, iterations);
        else if (mode == 2)
            run_benchmark_multiply(2, Nx, Ny, iterations);
        else if (mode == 3)
            run_benchmark_precode(Nx, Ny, iterations);
        else
            printf("Mode is not supported!\n");
    }
}