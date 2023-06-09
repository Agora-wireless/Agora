#ifndef UTILS_LDPC_H_
#define UTILS_LDPC_H_

#include <cstdlib> /* for std::aligned_alloc */

#include "encoder.h"
#include "iobuffer.h"
#include "phy_ldpc_encoder_5gnr.h"
#include "symbols.h"
#include "utils.h"

// Since the LDPC helper function input parameter undergoes 32byte read/write it
// is necessary to pad the input buffers 32 bytes max (ie 32 will work for all
// configurations)

LDPC_ADAPTER_P LdpcSelectAdapterFunc(uint16_t zcSize, uint8_t num_ways);

template <typename T>
T* AlignedMalloc(const int size, const unsigned alignment) {
#ifdef _BBLIB_DPDK_
  return reinterpret_cast<T*>(rte_malloc(NULL, sizeof(T) * size, alignment));
#else
#ifndef _WIN64
  return reinterpret_cast<T*>(std::aligned_alloc(alignment, sizeof(T) * size));
#else
  return reinterpret_cast<T*>(_aligned_malloc(sizeof(T) * size, alignment));
#endif
#endif
}

#ifndef __has_builtin
#define __has_builtin(x) 0
#endif

static inline uint8_t Bitreverse8(uint8_t x) {
#if __has_builtin(__builtin_bireverse8)
  return (__builtin_bitreverse8(x));
#else
  x = (x << 4) | (x >> 4);
  x = ((x & 0x33) << 2) | ((x >> 2) & 0x33);
  x = ((x & 0x55) << 1) | ((x >> 1) & 0x55);
  return (x);
#endif
}

/**
 * \brief Fill-in the bytes of \p bytes_out with \p mod_type bits per byte,
 * taken from the bit sequence \p bit_seq_in
 *
 * \param bit_seq_in The input bit sequence
 *
 * \param bytes_out The output byte array with \p mod_type bits per byte. It
 * must have space for ceil(len * 8.0 / mod_type) bytes.
 *
 * \param len The number of bytes in \p bit_seq_in
 *
 * \param mod_type The number of bits in one modulated symbol (e.g., mod_type =
 * 6 for 64-QAM modulation)
 */
static inline void AdaptBitsForMod(const uint8_t* bit_seq_in,
                                   uint8_t* bytes_out, size_t len,
                                   size_t mod_type) {
  uint16_t bits = 0;      // Bits collected from the input
  size_t bits_avail = 0;  // Number of valid bits filled into [bits]
  for (size_t i = 0; i < len; i++) {
    bits |= static_cast<uint32_t>(Bitreverse8(bit_seq_in[i]))
            << (8 - bits_avail);
    bits_avail += 8;
    while (bits_avail >= mod_type) {
      *bytes_out++ = bits >> (16 - mod_type);
      bits <<= mod_type;
      bits_avail -= mod_type;
    }
  }

  if (bits_avail > 0) {
    *bytes_out++ = bits >> (16 - mod_type);
  }
}

/*
 * Copy packed, bit-reversed 8-bit fields stored in
 * vec_in[0..len-1] into unpacked m-bit vec_out (m == mod_type).
 * Storage at vec_out must be at least (m*len+7)/8 bytes.
 */
static inline void AdaptBitsFromMod(const uint8_t* vec_in, uint8_t* vec_out,
                                    int len, int mod_type) {
  int bits_avail = 0;
  uint16_t bits = 0;
  for (int i = 0; i < len; i++) {
    bits |= (Bitreverse8(vec_in[i]) >> (8 - mod_type)) << bits_avail;
    bits_avail += mod_type;
    while (bits_avail >= 8) {
      *vec_out++ = bits & 0xff;
      bits >>= 8;
      bits_avail -= 8;
    }
  }
}

static inline uint8_t SelectBaseMatrixEntry(uint16_t Zc) {
  uint8_t i_ls;
  if ((Zc % 15) == 0) {
    i_ls = 7;
  } else if ((Zc % 13) == 0) {
    i_ls = 6;
  } else if ((Zc % 11) == 0) {
    i_ls = 5;
  } else if ((Zc % 9) == 0) {
    i_ls = 4;
  } else if ((Zc % 7) == 0) {
    i_ls = 3;
  } else if ((Zc % 5) == 0) {
    i_ls = 2;
  } else if ((Zc % 3) == 0) {
    i_ls = 1;
  } else {
    i_ls = 0;
  }
  return i_ls;
}

// Return the number of bytes needed to store n_bits bits
static inline size_t BitsToBytes(size_t n_bits) { return (n_bits + 7) / 8; }

// Return the number of non-expanded base graph columns used for information
// bits for this base graph
static inline size_t LdpcNumInputCols(size_t base_graph) {
  return (base_graph == 1 ? BG1_COL_INF_NUM : BG2_COL_INF_NUM);
}

// Return the maximum number of rows in this non-expanded base graph
static inline size_t LdpcMaxNumRows(size_t base_graph) {
  return (base_graph == 1 ? BG1_ROW_TOTAL : BG2_ROW_TOTAL);
}

// Return the number of input information bits per codeword with this base
// graph and expansion factor
static inline size_t LdpcNumInputBits(size_t base_graph, size_t zc) {
  return zc * LdpcNumInputCols(base_graph);
}

// Return the number of parity bits per codeword with this base graph and
// expansion factor
static inline size_t LdpcMaxNumParityBits(size_t base_graph, size_t zc) {
  return zc * LdpcMaxNumRows(base_graph);
}

// Return the maximum number of total bits per code block with this base graph
// and expansion factor
static inline size_t LdpcMaxNumEncodedBits(size_t base_graph, size_t zc) {
  static size_t num_punctured_cols = 2;
  return zc * (base_graph == 1 ? (BG1_COL_TOTAL - num_punctured_cols)
                               : (BG2_COL_TOTAL - num_punctured_cols));
}

// Return the number of total bits per codeword (i.e., including both input
// bits and parity bits) with this base graph and expansion factor
static inline size_t LdpcNumEncodedBits(size_t base_graph, size_t zc,
                                        size_t nRows) {
  static size_t num_punctured_cols = 2;
  return zc * (LdpcNumInputCols(base_graph) + nRows - num_punctured_cols);
}

// Return the number of bytes required in the input buffer used for LDPC
// encoding
static inline size_t LdpcEncodingInputBufSize(size_t base_graph, size_t zc) {
  // We add kMaxProcBytes as padding for the encoder's scatter function
  return BitsToBytes(LdpcNumInputBits(base_graph, zc)) + kMaxProcBytes;
}

// Return the number of bytes required in the parity buffer used for LDPC
// encoding
static inline size_t LdpcEncodingParityBufSize(size_t base_graph, size_t zc) {
  // We add kMaxProcBytes as padding for the encoder's gather function
  return BitsToBytes(LdpcMaxNumParityBits(base_graph, zc)) + kMaxProcBytes;
}

// Return the number of bytes required in the output encoded codeword buffer
// used for LDPC encoding
static inline size_t LdpcEncodingEncodedBufSize(size_t base_graph, size_t zc) {
  // We add kMaxProcBytes as padding for the encoder's gather function
  return BitsToBytes(LdpcMaxNumEncodedBits(base_graph, zc)) + kMaxProcBytes;
}

// Return the minimum LDPC expansion factor supported
static inline size_t LdpcGetMinZc() { return kUseAVX2Encoder ? 2 : 6; }

// Return the maximum LDPC expansion factor supported
static inline size_t LdpcGetMaxZc() {
  return kUseAVX2Encoder ? avx2enc::kZcMax : ZC_MAX;
}

// Generate the codeword output and parity buffer for this input buffer
static inline void LdpcEncodeHelper(size_t base_graph, size_t zc, size_t nRows,
                                    int8_t* encoded_buffer,
                                    int8_t* parity_buffer,
                                    const int8_t* input_buffer) {
  const size_t num_input_bits = LdpcNumInputBits(base_graph, zc);
  const size_t num_parity_bits = nRows * zc;

  bblib_ldpc_encoder_5gnr_request req;
  bblib_ldpc_encoder_5gnr_response resp;
  req.baseGraph = base_graph;
  req.nRows = kUseAVX2Encoder ? LdpcMaxNumRows(base_graph) : nRows;
  req.Zc = zc;
  req.nRows = nRows;
  req.numberCodeblocks = 1;
  req.input[0] = const_cast<int8_t*>(input_buffer);
  resp.output[0] = parity_buffer;

  kUseAVX2Encoder ? avx2enc::BblibLdpcEncoder5gnr(&req, &resp)
                  : bblib_ldpc_encoder_5gnr(&req, &resp);

  // Copy punctured input bits from the encoding request, and parity bits from
  // the encoding response into encoded_buffer
  static size_t k_num_punctured_cols = 2;
  if (zc % 4 == 0) {
    // In this case, the start and end of punctured input bits is
    // byte-aligned, so we can std::memcpy
    const size_t num_punctured_bytes = BitsToBytes(zc * k_num_punctured_cols);
    const size_t num_input_bytes_to_copy =
        BitsToBytes(num_input_bits) - num_punctured_bytes;

    std::memcpy(encoded_buffer, input_buffer + num_punctured_bytes,
                num_input_bytes_to_copy);
    std::memcpy(encoded_buffer + num_input_bytes_to_copy, parity_buffer,
                BitsToBytes(num_parity_bits));
  } else {
    // Otherwise, we need to std::memcpy from/to byte-unaligned locations. A
    // simple but perhaps inefficient way to do this is to use the encoder's
    // internal scatter/gather functions. We don't have access to these
    // functions for FlexRAN's internal AVX-512 encoder.
    if (zc >= avx2enc::kProcBytes * 8) {
      std::fprintf(stderr,
                   "Zc values >= %zu that are not multiples of four are not "
                   "supported yet.\n",
                   zc);
      throw std::runtime_error(
          "Zc values >= values that are not multiples of four");
    }

    __attribute__((aligned(avx2enc::kProcBytes)))
    int8_t internal_buffer0[BG1_COL_INF_NUM * avx2enc::kProcBytes] = {0};
    __attribute__((aligned(avx2enc::kProcBytes)))
    int8_t internal_buffer1[BG1_ROW_TOTAL * avx2enc::kProcBytes] = {0};
    __attribute__((aligned(avx2enc::kProcBytes)))
    int8_t internal_buffer2[BG1_COL_TOTAL * avx2enc::kProcBytes] = {0};

    auto adapter_func = avx2enc::LdpcSelectAdapterFunc(zc);

    // Scatter input and parity into zc-bit chunks
    adapter_func((int8_t*)input_buffer, internal_buffer0, zc, num_input_bits,
                 1);
    adapter_func(parity_buffer, internal_buffer1, zc, num_parity_bits, 1);

    // Concactenate the chunks for input and parity
    std::memcpy(internal_buffer2,
                internal_buffer0 + k_num_punctured_cols * avx2enc::kProcBytes,
                (LdpcNumInputCols(base_graph) - k_num_punctured_cols) *
                    avx2enc::kProcBytes);
    std::memcpy(internal_buffer2 +
                    (LdpcNumInputCols(base_graph) - k_num_punctured_cols) *
                        avx2enc::kProcBytes,
                internal_buffer1,
                LdpcMaxNumRows(base_graph) * avx2enc::kProcBytes);

    // Gather the concatenated chunks to create the encoded buffer
    adapter_func(encoded_buffer, internal_buffer2, zc,
                 LdpcNumEncodedBits(base_graph, zc, nRows), 0);
  }
}

#endif  // UTILS_LDPC_H_
