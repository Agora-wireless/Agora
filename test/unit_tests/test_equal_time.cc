#include <gtest/gtest.h>
// For some reason, gtest include order matters
#include <thread>

#include "concurrentqueue.h"
#include "config.h"
#include "dodemul.h"
#include "gettime.h"
#include "modulation.h"
#include "phy_stats.h"
#include "utils.h"

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

void equal_vec(Config* cfg_,
               Table<complex_float>& data_buffer_,
               Table<complex_float>& equal_buffer_,
               Table<complex_float>& ue_spec_pilot_buffer_,
               PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& ul_beam_matrices_,
               size_t frame_id_, size_t symbol_id_, size_t base_sc_id_) {
  
  RtAssert(cfg_->BsAntNum() == 1 && cfg_->UeAntNum() == 1,
           "Correctness is only guaranteed in special case of antenna 1x1!");
  // RtAssert(kUsePartialTrans == false,
  //          "If set kUsePartialTrans = true, the test case might fail (with a probability)");
  RtAssert(kExportConstellation == true,
           "Set kExportConstellation to evaluate the correctness (export equal_buffer_)");

  // ---------------------------------------------------------------------------
  // Class definition of DoDemul
  // ---------------------------------------------------------------------------

  // Intermediate buffers for equalized data
  complex_float* equaled_buffer_temp_;
  arma::cx_fmat ue_pilot_data_;

  // For efficient phase shift calibration
  static arma::fvec theta_vec;
  static float theta_inc;

  // ---------------------------------------------------------------------------
  // Constructor of DoDemul
  // ---------------------------------------------------------------------------

  equaled_buffer_temp_ =
      static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
          Agora_memory::Alignment_t::kAlign64,
          cfg_->DemulBlockSize() * kMaxUEs * sizeof(complex_float)));

  // phase offset calibration data
  arma::cx_float* ue_pilot_ptr =
      reinterpret_cast<arma::cx_float*>(cfg_->UeSpecificPilot()[0]);
  arma::cx_fvec vec_pilot_data(ue_pilot_ptr, cfg_->OfdmDataNum(), false);

  // ---------------------------------------------------------------------------
  // First part of DoDemul: equalization + phase shift calibration
  // ---------------------------------------------------------------------------

  size_t frame_id = frame_id_;
  size_t symbol_id = symbol_id_;
  size_t base_sc_id = base_sc_id_;

  // ---------------------------------------------------------------------------

  const size_t symbol_idx_ul = cfg_->Frame().GetULSymbolIdx(symbol_id);
  const size_t total_data_symbol_idx_ul =
      cfg_->GetTotalDataSymbolIdxUl(frame_id, symbol_idx_ul);
  const complex_float* data_buf = data_buffer_[total_data_symbol_idx_ul];

  const size_t frame_slot = frame_id % kFrameWnd;

  size_t max_sc_ite =
      std::min(cfg_->DemulBlockSize(), cfg_->OfdmDataNum() - base_sc_id);
  assert(max_sc_ite % kSCsPerCacheline == 0);

  // Step 1: Equalization
  arma::cx_float* equal_ptr = nullptr;
  if (kExportConstellation) {
    equal_ptr = (arma::cx_float*)(&equal_buffer_[total_data_symbol_idx_ul]
                                                [base_sc_id]);
  } else {
    equal_ptr = (arma::cx_float*)(&equaled_buffer_temp_[0]);
  }
  arma::cx_fvec vec_equaled(equal_ptr, max_sc_ite, false);

  arma::cx_float* data_ptr = (arma::cx_float*)(&data_buf[base_sc_id]);
  // not consider multi-antenna case (antena offset is omitted)
  arma::cx_float* ul_beam_ptr = reinterpret_cast<arma::cx_float*>(
      ul_beam_matrices_[frame_slot][0]); // pick the first element

  // assuming cfg_->BsAntNum() == 1, reducing a dimension
  arma::cx_fvec vec_data(data_ptr, max_sc_ite, false);
  arma::cx_fvec vec_ul_beam(max_sc_ite); // init empty vec
  for (size_t i = 0; i < max_sc_ite; ++i) {
    vec_ul_beam(i) = ul_beam_ptr[cfg_->GetBeamScId(base_sc_id + i)];
  }
  vec_equaled = vec_ul_beam % vec_data;

  // Step 2: Phase shift calibration

  // Enable phase shift calibration
  if (cfg_->Frame().ClientUlPilotSymbols() > 0) {

    if (symbol_idx_ul == 0 && base_sc_id == 0) {
      // Reset previous frame
      arma::cx_float* phase_shift_ptr = reinterpret_cast<arma::cx_float*>(
          ue_spec_pilot_buffer_[(frame_id - 1) % kFrameWnd]);
      arma::cx_fmat mat_phase_shift(phase_shift_ptr, cfg_->UeAntNum(),
                                    cfg_->Frame().ClientUlPilotSymbols(),
                                    false);
      mat_phase_shift.fill(0);
    }

    // Calc new phase shift
    if (symbol_idx_ul < cfg_->Frame().ClientUlPilotSymbols()) {
      arma::cx_float* phase_shift_ptr = reinterpret_cast<arma::cx_float*>(
        &ue_spec_pilot_buffer_[frame_id % kFrameWnd]
                              [symbol_idx_ul * cfg_->UeAntNum()]);
      arma::cx_fmat mat_phase_shift(phase_shift_ptr, cfg_->UeAntNum(), 1,
                                false);
      // printf("base_sc_id = %ld, end = %ld\n", base_sc_id, base_sc_id+max_sc_ite-1);
      arma::cx_fvec vec_ue_pilot_data_ = vec_pilot_data.subvec(base_sc_id, base_sc_id+max_sc_ite-1);

      // mat_phase_shift += sum(vec_equaled % conj(vec_ue_pilot_data_));
      mat_phase_shift += sum(sign(vec_equaled % conj(vec_ue_pilot_data_)));
      // sign should be able to optimize out but the result will be different
    }

    // Calculate the unit phase shift based on the first subcarrier
    // Check the special case condition to avoid reading wrong memory location
    RtAssert(cfg_->UeAntNum() == 1 && cfg_->Frame().ClientUlPilotSymbols() == 2);
    if (symbol_idx_ul == cfg_->Frame().ClientUlPilotSymbols() && base_sc_id == 0) { 
      arma::cx_float* pilot_corr_ptr = reinterpret_cast<arma::cx_float*>(
          ue_spec_pilot_buffer_[frame_id % kFrameWnd]);
      arma::cx_fvec pilot_corr_vec(pilot_corr_ptr,
                                   cfg_->Frame().ClientUlPilotSymbols(), false);
      theta_vec = arg(pilot_corr_vec);
      theta_inc = theta_vec(cfg_->Frame().ClientUlPilotSymbols()-1) - theta_vec(0);
      // theta_inc /= (float)std::max(
      //     1, static_cast<int>(cfg_->Frame().ClientUlPilotSymbols() - 1));
    }

    // Apply previously calc'ed phase shift to data
    if (symbol_idx_ul >= cfg_->Frame().ClientUlPilotSymbols()) {
      float cur_theta_f = theta_vec(0) + (symbol_idx_ul * theta_inc);
      vec_equaled *= arma::cx_float(cos(-cur_theta_f), sin(-cur_theta_f));
    }
  }
}

void equal_op_profile() {
  auto cfg_ = std::make_shared<Config>("files/config/ci/tddconfig-sim-ul-fr2.json");
  cfg_->GenData();
  arma::arma_rng::set_seed_random();

  // operator var
  size_t max_sc_ite = 768;
  arma::cx_fvec vec_equaled(max_sc_ite);
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
  printf("     . ms_apply_3 = %.2f ms (mult)\n", ms_apply_3);
}

TEST(TestEqual, VecFunc) {
  auto cfg_ = std::make_shared<Config>("files/config/ci/tddconfig-sim-ul-fr2.json");
  cfg_->GenData();

  // ---------------------------------------------------------------------------
  // Prepare buffers
  // ---------------------------------------------------------------------------

  // From agora_buffer.h
  Table<complex_float> data_buffer_;
  Table<complex_float> equal_buffer_;
  Table<complex_float> ue_spec_pilot_buffer_;
  PtrGrid<kFrameWnd, kMaxDataSCs, complex_float> ul_beam_matrices_;

  // From agora_buffer.cc
  const size_t task_buffer_symbol_num_ul =
    cfg_->Frame().NumULSyms() * kFrameWnd;
  data_buffer_.RandAllocCxFloat(task_buffer_symbol_num_ul,
                     cfg_->OfdmDataNum() * cfg_->BsAntNum(),
                     Agora_memory::Alignment_t::kAlign64);
  equal_buffer_.RandAllocCxFloat(task_buffer_symbol_num_ul,
                       cfg_->OfdmDataNum() * cfg_->SpatialStreamsNum(),
                       Agora_memory::Alignment_t::kAlign64);
  ue_spec_pilot_buffer_.RandAllocCxFloat(
      kFrameWnd,
      cfg_->Frame().ClientUlPilotSymbols() * cfg_->SpatialStreamsNum(),
      Agora_memory::Alignment_t::kAlign64);

  ul_beam_matrices_.RandAllocCxFloat(cfg_->BsAntNum() * cfg_->SpatialStreamsNum());

  // ---------------------------------------------------------------------------
  // Run
  // ---------------------------------------------------------------------------

  for (size_t frame_id = 0; frame_id <= kFrameWnd; ++frame_id) {
    for (size_t symbol_id = 1; symbol_id < cfg_->Frame().NumULSyms(); ++symbol_id) {
      for (size_t base_sc_id = 0; base_sc_id < cfg_->OfdmDataNum(); base_sc_id += cfg_->DemulBlockSize()) {
        equal_vec(cfg_.get(), data_buffer_, equal_buffer_,
                  ue_spec_pilot_buffer_, ul_beam_matrices_,
                  frame_id, symbol_id, base_sc_id);
      }
    }
  }
}

TEST(TestEqual, VecOpTime) {
    equal_op_profile();
}

int main(int argc, char** argv) {
  printf("Running main() from %s\n", __FILE__);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
