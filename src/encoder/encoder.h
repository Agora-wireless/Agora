/**
 * @file encoder.h
 * @brief Definitions for Agora's AVX2-based LDPC encoder.
 *
 * We need an AVX2-based LDPC encoder because FlexRAN's LDPC encoder requires
 * AVX-512.
 */
#ifndef ENCODER_H_
#define ENCODER_H_

#include "gcc_phy_ldpc_encoder_5gnr_internal.h"
#include "phy_ldpc_encoder_5gnr.h"

#define BITMASKU8(x) ((1U << (x)) - 1)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

namespace avx2enc {

// Maximum 5G NR LDPC expansion factor (Zc) supported by the AVX2-based encoder
static constexpr size_t kZcMax = 255;

static constexpr size_t kProcBytes = 32;
int32_t BblibLdpcEncoder5gnr(struct bblib_ldpc_encoder_5gnr_request* request,
                             struct bblib_ldpc_encoder_5gnr_response* response);
};  // namespace avx2enc

// PROC_BYTES (maximum bytes processed as an LDPC chunk) is 64 bytes in
// FlexRAN's LDPC encoder and 32 bytes in Agora's derived LDPC encoder.
// Using the larger of the two works for padding buffers.
static constexpr size_t kMaxProcBytes = 64;

#endif  // ENCODER_H_
