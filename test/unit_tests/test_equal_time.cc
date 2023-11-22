#include <gtest/gtest.h>
// For some reason, gtest include order matters

#include "config.h"
#include "dodemul.h"
#include "gettime.h"

// set static constexpr bool kExportConstellation = true; at symbol.h to enable
// this unit test. otherwise, the correctness check is not reliable.
// TODO: Test the case that kExportConstellation = false;

// Operator overload for correctness comparison
bool operator==(const complex_float lhs, const complex_float& rhs)
{
  return (lhs.re == rhs.re) && (lhs.im == rhs.im);
}

bool operator!=(const complex_float lhs, const complex_float& rhs)
{
  // return (lhs.re != rhs.re) || (lhs.im != rhs.im);

  // approx_eq, enable this if you would like to compare between armadillo & MKL
  float threshold = 0.0001;
  return (fabs(lhs.re - rhs.re) > threshold) ||
         (fabs(lhs.im - rhs.im) > threshold);
}

void equal_op_profile_1x1_complex() {
  auto cfg_ = std::make_shared<Config>(
      "files/config/ci/tddconfig-sim-ul-fr2.json");
  cfg_->GenData();
  arma::arma_rng::set_seed_random();

  // operator var
  size_t max_sc_ite = 768;
  arma::cx_fvec vec_equaled(max_sc_ite);
  arma::cx_fvec vec_equaled_final;
  arma::cx_fvec vec_data(max_sc_ite, arma::fill::randu);
  arma::cx_fvec vec_ul_beam(max_sc_ite, arma::fill::randu);
  arma::cx_fmat mat_phase_shift(1, 1);
  static arma::fvec theta_vec;
  static float theta_inc;
  size_t symbol_idx_ul;
  
  // profile var
  size_t num_iter = 100000;
  size_t num_ul_per_frame = 16;
  size_t tsc_equal_0, tsc_equal_1;
  size_t tsc_reset_0, tsc_reset_1;
  size_t tsc_acc_0, tsc_acc_1;
  size_t tsc_unit_0, tsc_unit_1;
  size_t tsc_apply_0, tsc_apply_1, tsc_apply_2, tsc_apply_3, tsc_apply_4;
  double ms_equal = 0, ms_reset = 0, ms_acc = 0, ms_unit = 0, ms_apply = 0;
  double ms_apply_0 = 0, ms_apply_1 = 0, ms_apply_2 = 0, ms_apply_3 = 0;

  for (size_t i = 0; i < num_iter; ++i) {
    symbol_idx_ul = i % num_ul_per_frame;

    tsc_equal_0 = GetTime::Rdtsc();
    vec_equaled = vec_ul_beam % vec_data;
    tsc_equal_1 = GetTime::Rdtsc();
    ms_equal += GetTime::CyclesToMs(tsc_equal_1 - tsc_equal_0, cfg_->FreqGhz());

    if (symbol_idx_ul == 0) {
      // Reset previous frame
      arma::cx_fmat mat_phase_shift(cfg_->UeAntNum(), cfg_->Frame().ClientUlPilotSymbols());
      
      tsc_reset_0 = GetTime::Rdtsc();
      mat_phase_shift.fill(0);
      tsc_reset_1 = GetTime::Rdtsc();
      ms_reset += GetTime::CyclesToMs(tsc_reset_1 - tsc_reset_0, cfg_->FreqGhz());
    }

    // Calc new phase shift
    if (symbol_idx_ul < cfg_->Frame().ClientUlPilotSymbols()) {
      arma::cx_fvec vec_ue_pilot_data_(max_sc_ite, arma::fill::randu);

      tsc_acc_0 = GetTime::Rdtsc();
      // mat_phase_shift += sum(sign(vec_equaled % conj(vec_ue_pilot_data_)));
      mat_phase_shift += sum(vec_equaled % conj(vec_ue_pilot_data_));
      tsc_acc_1 = GetTime::Rdtsc();
      ms_acc += GetTime::CyclesToMs(tsc_acc_1 - tsc_acc_0, cfg_->FreqGhz());
    }

    // Calculate the unit phase shift based on the first subcarrier
    // Check the special case condition to avoid reading wrong memory location
    if (symbol_idx_ul == cfg_->Frame().ClientUlPilotSymbols()) { 
      arma::cx_fvec pilot_corr_vec(cfg_->Frame().ClientUlPilotSymbols(), arma::fill::randu);

      tsc_unit_0 = GetTime::Rdtsc();
      theta_vec = arg(pilot_corr_vec);
      theta_inc = theta_vec(cfg_->Frame().ClientUlPilotSymbols()-1) - theta_vec(0);
      tsc_unit_1 = GetTime::Rdtsc();
      ms_unit += GetTime::CyclesToMs(tsc_unit_1 - tsc_unit_0, cfg_->FreqGhz());
      // theta_inc /= (float)std::max(
      //     1, static_cast<int>(cfg_->Frame().ClientUlPilotSymbols() - 1));
    }

    // Apply previously calc'ed phase shift to data
    if (symbol_idx_ul >= cfg_->Frame().ClientUlPilotSymbols()) {

      tsc_apply_0 = GetTime::Rdtsc();
      float cur_theta_f = theta_vec(0) + (symbol_idx_ul * theta_inc);
      // vec_equaled *= arma::cx_float(cos(-cur_theta_f), sin(-cur_theta_f));
      tsc_apply_1 = GetTime::Rdtsc();
      float cos_f = cos(-cur_theta_f);
      float sin_f = sin(-cur_theta_f);
      tsc_apply_2 = GetTime::Rdtsc();
      arma::cx_float cx_shift = arma::cx_float(cos_f, sin_f);
      tsc_apply_3 = GetTime::Rdtsc();
      vec_equaled *= cx_shift;
      tsc_apply_4 = GetTime::Rdtsc();
      // RtAssert(arma::approx_equal(vec_equaled, vec_equaled_org, "both", 0.01, 0.01), "Correctness check.");
      ms_apply_0 += GetTime::CyclesToMs(tsc_apply_1 - tsc_apply_0, cfg_->FreqGhz());
      ms_apply_1 += GetTime::CyclesToMs(tsc_apply_2 - tsc_apply_1, cfg_->FreqGhz());
      ms_apply_2 += GetTime::CyclesToMs(tsc_apply_3 - tsc_apply_2, cfg_->FreqGhz());
      ms_apply_3 += GetTime::CyclesToMs(tsc_apply_4 - tsc_apply_3, cfg_->FreqGhz());
      ms_apply += GetTime::CyclesToMs(tsc_apply_4 - tsc_apply_0, cfg_->FreqGhz());
    }
  }

  printf("Time measured:\n");
  printf(" . ms_equal = %.2f ms\n", ms_equal);
  printf(" . ms_reset = %.2f ms\n", ms_reset);
  printf(" . ms_acc = %.2f ms\n", ms_acc);
  printf(" . ms_unit = %.2f ms\n", ms_unit);
  printf(" . ms_apply = %.2f ms\n", ms_apply);
  printf("     . ms_apply_0 = %.2f ms (cal theta)\n", ms_apply_0);
  printf("     . ms_apply_1 = %.2f ms (sin/cos)\n", ms_apply_1);
  printf("     . ms_apply_2 = %.2f ms (form cx_float)\n", ms_apply_2);
  printf("     . ms_apply_3 = %.2f ms (complex mult)\n", ms_apply_3);
}

void equal_op_profile_1x1_real() {
  auto cfg_ = std::make_shared<Config>(
      "files/config/ci/tddconfig-sim-ul-fr2.json");
  cfg_->GenData();
  arma::arma_rng::set_seed_random();

  // operator var
  size_t max_sc_ite = 768;
  arma::cx_fvec vec_equaled(max_sc_ite);
  arma::cx_fvec vec_equaled_final;
  arma::cx_fvec vec_data(max_sc_ite, arma::fill::randu);
  arma::cx_fvec vec_ul_beam(max_sc_ite, arma::fill::randu);
  arma::cx_fmat mat_phase_shift(1, 1);
  arma::fvec vec_equaled_real;
  arma::fvec vec_equaled_imag;
  arma::fvec vec_equaled_real_final;
  arma::fvec vec_equaled_imag_final;
  arma::fvec vec_data_real = arma::real(vec_data);
  arma::fvec vec_data_imag = arma::imag(vec_data);
  arma::fvec vec_ul_data_real = arma::real(vec_ul_beam);
  arma::fvec vec_ul_data_imag = arma::imag(vec_ul_beam);
  arma::fmat mat_phase_shift_real = arma::real(mat_phase_shift);
  arma::fmat mat_phase_shift_imag = arma::imag(mat_phase_shift);
  static arma::fvec theta_vec;
  static float theta_inc;
  size_t symbol_idx_ul;
  
  // profile var
  size_t num_iter = 100000;
  size_t num_ul_per_frame = 16;
  size_t tsc_equal_0, tsc_equal_1;
  size_t tsc_reset_0, tsc_reset_1;
  size_t tsc_acc_0, tsc_acc_1;
  size_t tsc_unit_0, tsc_unit_1;
  size_t tsc_apply_0, tsc_apply_1, tsc_apply_2, tsc_apply_3, tsc_apply_4, tsc_apply_5;
  double ms_equal = 0, ms_reset = 0, ms_acc = 0, ms_unit = 0, ms_apply = 0;
  double ms_apply_0 = 0, ms_apply_1 = 0, ms_apply_2 = 0, ms_apply_3 = 0, ms_apply_4 = 0;

  for (size_t i = 0; i < num_iter; ++i) {
    symbol_idx_ul = i % num_ul_per_frame;

    tsc_equal_0 = GetTime::Rdtsc();
    vec_equaled_real =
      vec_ul_data_real % vec_data_real - vec_ul_data_imag % vec_data_imag;
    vec_equaled_imag =
      vec_ul_data_real % vec_data_imag + vec_ul_data_imag % vec_data_real;
    // vec_equaled = vec_ul_beam % vec_data;
    tsc_equal_1 = GetTime::Rdtsc();
    ms_equal += GetTime::CyclesToMs(tsc_equal_1 - tsc_equal_0, cfg_->FreqGhz());

    if (symbol_idx_ul == 0) {
      // Reset previous frame
      arma::cx_fmat mat_phase_shift(cfg_->UeAntNum(), cfg_->Frame().ClientUlPilotSymbols());
      
      tsc_reset_0 = GetTime::Rdtsc();
      mat_phase_shift.fill(0);
      tsc_reset_1 = GetTime::Rdtsc();
      ms_reset += GetTime::CyclesToMs(tsc_reset_1 - tsc_reset_0, cfg_->FreqGhz());
    }

    // Calc new phase shift
    if (symbol_idx_ul < cfg_->Frame().ClientUlPilotSymbols()) {
      arma::cx_fvec vec_ue_pilot_data_(max_sc_ite, arma::fill::randu);

      arma::fvec vec_ue_pilot_data_real_ = arma::real(vec_ue_pilot_data_);
      arma::fvec vec_ue_pilot_data_imag_ = arma::imag(vec_ue_pilot_data_);

      tsc_acc_0 = GetTime::Rdtsc();
      mat_phase_shift_real += sum(
        vec_equaled_real % vec_ue_pilot_data_real_ + 
        vec_equaled_imag % vec_ue_pilot_data_imag_
      );
      mat_phase_shift_imag += sum(
        vec_equaled_imag % vec_ue_pilot_data_real_ - 
        vec_equaled_real % vec_ue_pilot_data_imag_
      );
      // mat_phase_shift += sum(sign(vec_equaled % conj(vec_ue_pilot_data_)));
      // mat_phase_shift += sum(vec_equaled % conj(vec_ue_pilot_data_));
      tsc_acc_1 = GetTime::Rdtsc();
      ms_acc += GetTime::CyclesToMs(tsc_acc_1 - tsc_acc_0, cfg_->FreqGhz());
    }

    // Calculate the unit phase shift based on the first subcarrier
    // Check the special case condition to avoid reading wrong memory location
    if (symbol_idx_ul == cfg_->Frame().ClientUlPilotSymbols()) { 
      arma::cx_fvec pilot_corr_vec(cfg_->Frame().ClientUlPilotSymbols(), arma::fill::randu);

      arma::fvec pilot_corr_vec_real = arma::real(pilot_corr_vec);
      arma::fvec pilot_corr_vec_imag = arma::imag(pilot_corr_vec);

      tsc_unit_0 = GetTime::Rdtsc();

      theta_vec = arma::atan(pilot_corr_vec_imag/pilot_corr_vec_real);
      // theta_vec = arg(pilot_corr_vec);
      theta_inc = theta_vec(cfg_->Frame().ClientUlPilotSymbols()-1) - theta_vec(0);
      tsc_unit_1 = GetTime::Rdtsc();
      ms_unit += GetTime::CyclesToMs(tsc_unit_1 - tsc_unit_0, cfg_->FreqGhz());
      // theta_inc /= (float)std::max(
      //     1, static_cast<int>(cfg_->Frame().ClientUlPilotSymbols() - 1));
    }

    // Apply previously calc'ed phase shift to data
    if (symbol_idx_ul >= cfg_->Frame().ClientUlPilotSymbols()) {

      tsc_apply_0 = GetTime::Rdtsc();
      float cur_theta_f = theta_vec(0) + (symbol_idx_ul * theta_inc);
      // vec_equaled *= arma::cx_float(cos(-cur_theta_f), sin(-cur_theta_f));
      tsc_apply_1 = GetTime::Rdtsc();
      float cos_f = cos(-cur_theta_f);
      float sin_f = sin(-cur_theta_f);
      tsc_apply_2 = GetTime::Rdtsc();
      arma::cx_float cx_shift = arma::cx_float(cos_f, sin_f);
      tsc_apply_3 = GetTime::Rdtsc();
      vec_equaled_real = arma::real(vec_equaled);
      vec_equaled_imag = arma::imag(vec_equaled);
      tsc_apply_4 = GetTime::Rdtsc();
      // not in-place
      vec_equaled_real_final = vec_equaled_real * cos_f - vec_equaled_imag * sin_f;
      vec_equaled_imag_final = vec_equaled_real * sin_f + vec_equaled_imag * cos_f;
      // vec_equaled = arma::cx_fvec(vec_equaled_real * cos_f - vec_equaled_imag * sin_f, vec_equaled_real * sin_f + vec_equaled_imag * cos_f);
      // in-place
      // vec_equaled_real = vec_equaled_real * cos_f - vec_equaled_imag * sin_f;
      // vec_equaled_imag = vec_equaled_real * sin_f + vec_equaled_imag * cos_f;
      tsc_apply_5 = GetTime::Rdtsc();
      ms_apply_0 += GetTime::CyclesToMs(tsc_apply_1 - tsc_apply_0, cfg_->FreqGhz());
      ms_apply_1 += GetTime::CyclesToMs(tsc_apply_2 - tsc_apply_1, cfg_->FreqGhz());
      ms_apply_2 += GetTime::CyclesToMs(tsc_apply_3 - tsc_apply_2, cfg_->FreqGhz());
      ms_apply_3 += GetTime::CyclesToMs(tsc_apply_4 - tsc_apply_3, cfg_->FreqGhz());
      ms_apply_4 += GetTime::CyclesToMs(tsc_apply_5 - tsc_apply_4, cfg_->FreqGhz());
      ms_apply += GetTime::CyclesToMs(tsc_apply_5 - tsc_apply_0, cfg_->FreqGhz());
    }
  }

  printf("Time measured:\n");
  printf(" . ms_equal = %.2f ms\n", ms_equal);
  printf(" . ms_reset = %.2f ms\n", ms_reset);
  printf(" . ms_acc = %.2f ms\n", ms_acc);
  printf(" . ms_unit = %.2f ms\n", ms_unit);
  printf(" . ms_apply = %.2f ms\n", ms_apply);
  printf("     . ms_apply_0 = %.2f ms (cal theta)\n", ms_apply_0);
  printf("     . ms_apply_1 = %.2f ms (sin/cos)\n", ms_apply_1);
  printf("     . ms_apply_2 = %.2f ms (form cx_float)\n", ms_apply_2);
  printf("     . ms_apply_3 = %.2f ms (read real/imag)\n", ms_apply_3);
  printf("     . ms_apply_4 = %.2f ms (real mult)\n", ms_apply_4);
}

void equal_op_profile_2x2_complex() {
  auto cfg_ = std::make_shared<Config>(
      "files/config/ci/tddconfig-sim-ul-fr2.json");
  cfg_->GenData();
  arma::arma_rng::set_seed_random();

  RtAssert(cfg_->BsAntNum() == 2, "BS ant num must be 2.");
  RtAssert(cfg_->UeAntNum() == 2, "UE ant num must be 2.");

  // operator var
  size_t max_sc_ite = 768;
  arma::cx_fcube cub_equaled(cfg_->BsAntNum(), 1, max_sc_ite);
  arma::cx_fcube cub_data(cfg_->BsAntNum(), 1, max_sc_ite, arma::fill::randu);
  arma::cx_fcube cub_ul_beam(
    cfg_->UeAntNum(), cfg_->BsAntNum(), max_sc_ite, arma::fill::randu);
  arma::cx_fmat mat_phase_shift(cfg_->UeAntNum(), 1);
  static arma::fmat theta_mat;
  static arma::fmat theta_inc;
  size_t symbol_idx_ul;
  
  // profile var
  size_t num_iter = 100000;
  size_t num_ul_per_frame = 16;
  size_t tsc_equal_0, tsc_equal_1;
  size_t tsc_reset_0, tsc_reset_1;
  size_t tsc_acc_0, tsc_acc_1;
  size_t tsc_unit_0, tsc_unit_1;
  size_t tsc_apply_0, tsc_apply_1, tsc_apply_2, tsc_apply_3;
  double ms_equal = 0, ms_reset = 0, ms_acc = 0, ms_unit = 0, ms_apply = 0;
  double ms_apply_0 = 0, ms_apply_1 = 0, ms_apply_2 = 0;

  for (size_t i = 0; i < num_iter; ++i) {
    symbol_idx_ul = i % num_ul_per_frame;

    tsc_equal_0 = GetTime::Rdtsc();
    // for (size_t i = 0; i < max_sc_ite; ++i) {
    //   cub_equaled.slice(i) = cub_ul_beam.slice(i) * cub_data.slice(i);
    // }
    cub_equaled.tube(0, 0) =
      cub_ul_beam.tube(0, 0) % cub_data.tube(0, 0) +
      cub_ul_beam.tube(0, 1) % cub_data.tube(1, 0);
    cub_equaled.tube(1, 0) =
      cub_ul_beam.tube(1, 0) % cub_data.tube(0, 0) +
      cub_ul_beam.tube(1, 1) % cub_data.tube(1, 0);
    tsc_equal_1 = GetTime::Rdtsc();
    ms_equal += GetTime::CyclesToMs(tsc_equal_1 - tsc_equal_0, cfg_->FreqGhz());

    if (symbol_idx_ul == 0) {
      // Reset previous frame
      arma::cx_fmat mat_phase_shift(
        cfg_->UeAntNum(), cfg_->Frame().ClientUlPilotSymbols());
      
      tsc_reset_0 = GetTime::Rdtsc();
      mat_phase_shift.fill(0);
      tsc_reset_1 = GetTime::Rdtsc();
      ms_reset +=
        GetTime::CyclesToMs(tsc_reset_1 - tsc_reset_0, cfg_->FreqGhz());
    }

    // Calc new phase shift
    if (symbol_idx_ul < cfg_->Frame().ClientUlPilotSymbols()) {
      arma::cx_fmat mat_ue_pilot_data_(
        cfg_->UeAntNum(), max_sc_ite, arma::fill::randu);

      tsc_acc_0 = GetTime::Rdtsc();
      arma::cx_frowvec vec_tube_equal_0 =
        cub_equaled(arma::span(0), arma::span(0), arma::span::all);
      arma::cx_frowvec vec_tube_equal_1 =
        cub_equaled(arma::span(1), arma::span(0), arma::span::all);

      mat_phase_shift.col(0).row(0) += sum(
        vec_tube_equal_0 % arma::conj(mat_ue_pilot_data_.row(0))
      );
      mat_phase_shift.col(0).row(1) += sum(
        vec_tube_equal_1 % arma::conj(mat_ue_pilot_data_.row(1))
      );
      tsc_acc_1 = GetTime::Rdtsc();
      ms_acc += GetTime::CyclesToMs(tsc_acc_1 - tsc_acc_0, cfg_->FreqGhz());
    }

    // Calculate the unit phase shift based on the first subcarrier
    // Check the special case condition to avoid reading wrong memory location
    if (symbol_idx_ul == cfg_->Frame().ClientUlPilotSymbols()) { 
      arma::cx_fmat pilot_corr_mat(cfg_->UeAntNum(),
        cfg_->Frame().ClientUlPilotSymbols(), arma::fill::randu);

      tsc_unit_0 = GetTime::Rdtsc();
      theta_mat = arg(pilot_corr_mat);
      theta_inc =
        theta_mat.col(cfg_->Frame().ClientUlPilotSymbols()-1) -
        theta_mat.col(0);
      tsc_unit_1 = GetTime::Rdtsc();
      ms_unit += GetTime::CyclesToMs(tsc_unit_1 - tsc_unit_0, cfg_->FreqGhz());
      // theta_inc /= (float)std::max(
      //     1, static_cast<int>(cfg_->Frame().ClientUlPilotSymbols() - 1));
    }

    // Apply previously calc'ed phase shift to data
    if (symbol_idx_ul >= cfg_->Frame().ClientUlPilotSymbols()) {


      tsc_apply_0 = GetTime::Rdtsc();
      arma::fmat cur_theta = theta_mat.col(0) + (symbol_idx_ul * theta_inc);
      tsc_apply_1 = GetTime::Rdtsc();
      arma::cx_fmat mat_phase_correct =
          arma::cx_fmat(cos(-cur_theta), sin(-cur_theta));
      tsc_apply_2 = GetTime::Rdtsc();
      cub_equaled.each_slice() %= mat_phase_correct;
      tsc_apply_3 = GetTime::Rdtsc();
      ms_apply_0 +=
        GetTime::CyclesToMs(tsc_apply_1 - tsc_apply_0, cfg_->FreqGhz());
      ms_apply_1 +=
        GetTime::CyclesToMs(tsc_apply_2 - tsc_apply_1, cfg_->FreqGhz());
      ms_apply_2 +=
        GetTime::CyclesToMs(tsc_apply_3 - tsc_apply_2, cfg_->FreqGhz());
      ms_apply +=
        GetTime::CyclesToMs(tsc_apply_3 - tsc_apply_0, cfg_->FreqGhz());
    }
  }

  printf("Time measured:\n");
  printf(" . ms_equal = %.2f ms\n", ms_equal);
  printf(" . ms_reset = %.2f ms\n", ms_reset);
  printf(" . ms_acc = %.2f ms\n", ms_acc);
  printf(" . ms_unit = %.2f ms\n", ms_unit);
  printf(" . ms_apply = %.2f ms\n", ms_apply);
  printf("     . ms_apply_0 = %.2f ms (cal theta)\n", ms_apply_0);
  printf("     . ms_apply_1 = %.2f ms (sin/cos + form mat)\n", ms_apply_1);
  printf("     . ms_apply_2 = %.2f ms (broadcast mat mult)\n", ms_apply_2);
}

TEST(TestEqual, VecOpTime) {
  // equal_op_profile_1x1_complex();
  // equal_op_profile_1x1_real();
  equal_op_profile_2x2_complex();
}

int main(int argc, char** argv) {
  printf("Running main() from %s\n", __FILE__);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
