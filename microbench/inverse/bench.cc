#include "timer.h"
#include <armadillo>
#include <eigen3/Eigen/Dense>
#include <gflags/gflags.h>
#include <iostream>
#include <mkl.h>

static constexpr size_t kNumIters = 1000;
double freq_ghz = -1.0;

DEFINE_uint64(n_rows, 64, "Number of matrix rows");
DEFINE_uint64(n_cols, 32, "Number of matrix columns");
DEFINE_uint64(check_condition, 0, "Check the square matrix's condition number");

enum class PinvMode { kFormula, kSVD };

arma::cx_float test_arma(const arma::cx_float* _in_mat_arr, PinvMode mode)
{
    const size_t tot_size = FLAGS_n_rows * FLAGS_n_cols;
    auto* in_mat_arr = new arma::cx_float[tot_size];
    auto* out_mat_arr = new arma::cx_float[tot_size];
    for (size_t i = 0; i < tot_size; i++)
        in_mat_arr[i] = _in_mat_arr[i];

    arma::cx_fmat input(in_mat_arr, FLAGS_n_rows, FLAGS_n_cols, false);
    arma::cx_fmat output(out_mat_arr, FLAGS_n_cols, FLAGS_n_rows, false);

    TscTimer timer;
    timer.start();
    arma::cx_float ret(0.0, 0.0);
    for (size_t iter = 0; iter < kNumIters; iter++) {
        if (mode == PinvMode::kFormula) {
            arma::cx_fmat A = input.t() * input;
            if (FLAGS_check_condition == 1)
                ret += rcond(A);
            output = A.i() * input.t();
        } else {
            output = pinv(input);
        }
        ret += arma::accu(output);
        if (iter == 0) {
            std::cout << arma::accu(output) << std::endl;
        }
        out_mat_arr[0] = in_mat_arr[0];
    }

    timer.stop();
    printf("Armadillo: Average time for %s-based pseudo-inverse of "
           "%zux%zu matrix = %.3f ms\n",
        mode == PinvMode::kFormula ? "formula" : "SVD", FLAGS_n_rows,
        FLAGS_n_cols, timer.avg_usec(freq_ghz) / (1000.0 * kNumIters));
    return ret;
}

std::complex<float> test_eigen(
    const std::complex<float>* _in_mat_arr, PinvMode mode)
{
    const size_t tot_size = FLAGS_n_rows * FLAGS_n_cols;
    auto* in_mat_arr = new std::complex<float>[tot_size];
    auto* out_mat_arr = new std::complex<float>[tot_size];
    for (size_t i = 0; i < tot_size; i++)
        in_mat_arr[i] = _in_mat_arr[i];

    Eigen::Map<Eigen::Matrix<std::complex<float>, Eigen::Dynamic,
        Eigen::Dynamic, Eigen::ColMajor>>
        input(in_mat_arr, FLAGS_n_rows, FLAGS_n_cols);

    Eigen::Map<Eigen::Matrix<std::complex<float>, Eigen::Dynamic,
        Eigen::Dynamic, Eigen::ColMajor>>
        output(out_mat_arr, FLAGS_n_rows, FLAGS_n_cols);

    std::complex<float> ret(0.0, 0.0);
    TscTimer timer;
    timer.start();
    for (size_t iter = 0; iter < kNumIters; iter++) {
        if (mode == PinvMode::kFormula) {
            output = (input.adjoint() * input).inverse() * input.adjoint();
        } else {
            output = input.completeOrthogonalDecomposition().pseudoInverse();
        }

        ret += output.sum();
        if (iter == 0) {
            std::cout << output.sum() << std::endl;
        }
        in_mat_arr[0] = out_mat_arr[0];
    }

    timer.stop();
    printf("Eigen: Average time for %s-based pseudo-inverse of "
           "%zux%zu matrix = %.3f ms\n",
        mode == PinvMode::kFormula ? "formula" : "SVD", FLAGS_n_rows,
        FLAGS_n_cols, timer.avg_usec(freq_ghz) / (1000.0 * kNumIters));
    return ret;
}

int main(int argc, char** argv)
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    freq_ghz = measure_rdtsc_freq();
    mkl_set_num_threads(1);

    arma::cx_float* in_base_mat_arr
        = new arma::cx_float[FLAGS_n_rows * FLAGS_n_cols];
    for (size_t i = 0; i < FLAGS_n_rows * FLAGS_n_cols; i++) {
        auto re = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        auto im = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        in_base_mat_arr[i] = arma::cx_float(re, im);
    }

    {
        arma::cx_float ret_1 = test_arma(in_base_mat_arr, PinvMode::kFormula);
        arma::cx_float ret_2 = test_arma(in_base_mat_arr, PinvMode::kSVD);
        printf("{%.5f, %.5f}, {%.5f, %.5f}\n", ret_1.real(), ret_1.imag(),
            ret_2.real(), ret_2.imag());
    }

    {
        std::complex<float> ret_1
            = test_eigen(in_base_mat_arr, PinvMode::kFormula);
        std::complex<float> ret_2 = test_eigen(in_base_mat_arr, PinvMode::kSVD);
        printf("{%.5f, %.5f}, {%.5f, %.5f}\n", ret_1.real(), ret_1.imag(),
            ret_2.real(), ret_2.imag());
    }
}
