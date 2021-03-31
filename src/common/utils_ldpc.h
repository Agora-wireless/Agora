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
constexpr size_t kLdpcHelperFunctionInputBufferSizePaddingBytes = 32;

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

// Lookup table for determining modulation order based on modulation and coding
// scheme (MCS)
// 3GPP TS38.214-Table 5.1.3.1-2
static constexpr size_t McsToModOrderBits[28] = {2, 2, 2, 2, 2, 4, 4, 4, 4, 4,
                                                 4, 6, 6, 6, 6, 6, 6, 6, 6, 6,
                                                 8, 8, 8, 8, 8, 8, 8, 8};

// Lookup table for determining code rate based on MCS
// 3GPP TS38.214-Table 5.1.3.1-2
static constexpr float McsToCodeRate[28] = {
    0.1172, 0.1885, 0.3008, 0.4385, 0.5879, 0.3691, 0.4238,
    0.4785, 0.5400, 0.6016, 0.6426, 0.4551, 0.5049, 0.5537,
    0.6016, 0.6504, 0.7021, 0.7539, 0.8027, 0.8525, 0.6665,
    0.6943, 0.7363, 0.7783, 0.8213, 0.8643, 0.8950, 0.9258};

// Lookup table for transport block size from 3GPP TS38.214-Table 5.1.3.2-2
static std::vector<size_t> LutNInfo = {
    24,   32,   40,   48,   56,   64,   72,   80,   88,   96,   104,  112,
    120,  128,  136,  144,  152,  160,  168,  176,  184,  192,  208,  224,
    240,  256,  272,  288,  304,  320,  336,  352,  368,  384,  408,  432,
    456,  480,  504,  528,  552,  576,  608,  640,  672,  704,  736,  768,
    808,  848,  928,  984,  1032, 1064, 1128, 1160, 1192, 1224, 1256, 1288,
    1320, 1352, 1416, 1480, 1544, 1608, 1672, 1736, 1800, 1864, 1928, 2024,
    2088, 2152, 2216, 2280, 2408, 2472, 2536, 2600, 2664, 2728, 2792, 2856,
    2976, 3104, 3240, 3368, 3496, 3624, 3752, 3824};

// Set of LDPC lifting size Zc from 3GPP TS38.212-Table 5.3.2-1
static std::vector<size_t> LutZc = {
    2,   3,   4,   5,   6,   7,   8,   9,   10,  11,  12,  13,  14,
    15,  16,  18,  20,  22,  24,  26,  28,  30,  32,  36,  40,  44,
    48,  52,  56,  60,  64,  72,  80,  88,  96,  104, 112, 120, 128,
    144, 160, 176, 192, 208, 224, 240, 256, 288, 320, 352, 384};

// Select base graph based on transport block size and code rate.
// 3GPP TS38.212-7.2.2
static inline uint16_t SelectBaseGraph(size_t tb_size, float code_rate) {
  if (tb_size < 292 or (tb_size <= 3824 and code_rate <= 0.67) or
      code_rate <= 0.25)
    return 2;
  return 1;
}

// Return the closest value in a vector that is not less than the input value.
static inline size_t Closest(std::vector<size_t> const& vec, size_t value) {
  auto const it = std::lower_bound(vec.begin(), vec.end(), value);
  if (it == vec.end()) return -1;
  return *it;
}

// Return lifting size Zc based on 3GPP TS38.212-5.2.2
static inline size_t SelectZc(size_t tb_size, size_t n_info_per_cb,
                              uint16_t base_graph) {
  size_t kb;
  if (base_graph == 1) {
    kb = 22;
  } else {
    if (tb_size > 640)
      kb = 10;
    else if (tb_size > 560)
      kb = 9;
    else if (tb_size > 192)
      kb = 8;
    else
      kb = 6;
  }

  size_t zc = Closest(LutZc, std::ceil(1.f * n_info_per_cb / kb));
  return zc;
}

// Code block segmentation based on 3GPP TS38.212-5.2.2
static inline void CodeBlockSegmentation(size_t tb_size, uint16_t base_graph,
                                         size_t& n_cb, uint32_t& n_info_per_cb,
                                         uint16_t& zc) {
  size_t max_k_cb = base_graph == 1 ? 8448 : 3840;
  size_t tb_size_prime = tb_size;
  // Determine number of code blocks
  if (tb_size < max_k_cb) {
    n_cb = 1;
    n_info_per_cb = tb_size;
  } else {
    size_t n_crc_bits = 24;
    n_cb = std::ceil(1.f * tb_size / (max_k_cb - n_crc_bits));
    tb_size_prime = tb_size + n_cb * n_crc_bits;
  }

  // Determine number of info bits in each code block
  n_info_per_cb = tb_size_prime / n_cb;

  // Determine LDPC lifting size
  zc = SelectZc(tb_size_prime, n_info_per_cb, base_graph);

  // Update number of info bits in each code block
  n_info_per_cb = LdpcNumInputBits(base_graph, zc);
}

// Compute total number of information bits in a frame
static inline size_t ComputeNInfo(size_t n_symbol, size_t n_sc,
                                  size_t mod_order_bits, float code_rate) {
  return std::floor(n_symbol * n_sc * mod_order_bits * code_rate);
}

// Return transport block size based on number of info bits avaible in a slot
// and LDPC code rate
// 3GPP TS38.214-5.1.3.2
static inline size_t ComputeTbSize(size_t n_info, float code_rate) {
  size_t transport_block_size;
  if (n_info <= 3824) {
    size_t n = std::max(3, static_cast<int>(std::floor(std::log2(n_info)) - 6));
    size_t n_info_prime = std::max(
        24,
        static_cast<int>(std::pow(2, n) * std::floor(n_info / std::pow(2, n))));
    transport_block_size = Closest(LutNInfo, n_info_prime);
  } else {
    size_t n = std::floor(std::log2(n_info - 24)) - 5;
    size_t n_info_prime =
        std::pow(2, n) * std::round(1.f * (n_info - 24) / std::pow(2, n));
    size_t c = 1;
    if (code_rate < 0.25)
      c = std::ceil(1.f * (n_info_prime + 24) / 3816);
    else if (n_info_prime >= 8424)
      c = std::ceil(1.f * (n_info_prime + 24) / 8424);
    transport_block_size =
        8 * c * std::ceil(1.f * (n_info_prime + 24) / (8 * c)) - 24;
  }
  return transport_block_size;
}

static inline size_t ComputeNRows(float target_code_rate, uint16_t base_graph) {
  size_t n_info_cols = LdpcNumInputCols(base_graph);
  size_t n_rows = std::max(
      (size_t)4, std::min(LdpcMaxNumRows(base_graph),
                          static_cast<size_t>(
                              std::round(1.f * n_info_cols / target_code_rate) -
                              n_info_cols)));
  return n_rows;
}

static inline float ComputeCodeRate(size_t n_rows, uint16_t base_graph) {
  static size_t num_punctured_cols = 2;
  size_t n_info_cols = LdpcNumInputCols(base_graph);
  return (1.f * n_info_cols / (n_info_cols + n_rows - num_punctured_cols));
}

#endif  // UTILS_LDPC_H_