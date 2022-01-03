/// <COPYRIGHT_TAG>

#pragma once

#include "LdpcDecoder.hpp"

namespace SimdLdpc
{
  /// Each basegraph (as defined by 3GPP) "kernel" rows as the top 4 rows
  /// These rows have the highest row weights in the systematic and parity regions of
  /// the parity check matrix
  static constexpr int k_numKernelRows = 4;
  /// Basegraph#1 (BG1) has a maximum row-weight of 19
  static constexpr unsigned k_maxRowWeight = 19;
  /// The maximum number of rows is 46
  /// The parity-check matrix derived from BG1 is 46 rows x 68 cols
  /// The parity-check matrix derived from BG2 is 42 rows x 52 cols
  static constexpr unsigned k_maxRows = 46;
  static constexpr unsigned k_maxCols = 68;
  /// The maximum allowable value of z (the lifting factor) is defined as 384 in TS38.212v15 Section 5.3.2
  /// As the decoder does not use modulo addressing, this must be extended by the AVX512 SIMD length because
  /// the highest circulant address will be a SIMD32 read at position 383 (383, 384, 385, ..., 415)
  static constexpr unsigned k_maxZ = 384 + 32; //Extra 32 (64 bytes) for SIMD Decoder
   /// The maximum message size in bits
  static constexpr unsigned k_maxMessageSize = (k_maxCols - k_maxRows) * k_maxZ;
  /// The maximum codeword size
  static constexpr unsigned k_maxCodewordSize = k_maxCols * k_maxZ;
  /// Maximum number of non-negative entries in 3GPP BaseGraph#1
  static constexpr unsigned k_maxCirculants = 316;
  /// Maximum number of non-negative entries in 3GPP BaseGraph#2
  static constexpr unsigned k_bg2Circulants = 197;
  /// The maximum number of parity nodes that form the nRows * z parity-check equations
  /// This is the number of entries in table 5.3.2-2 of TS38.212v15 for one column of the table
  /// It is also equal to the number of circulants defined for BG1, and is also equal to the
  /// sum of the row-weights of BG1. BG1 has the highest row-weight sum of 316.
  static constexpr unsigned k_maxCheckNodes = 316 * k_maxCirculants;
  /// The number of int16_t values that form one cache-line.
  static constexpr int k_cacheInt16Alignment = k_cacheByteAlignment / 2;
  /// LLR "filler" or "shortened" bits are assigned a logical zero at the encoder.
  /// The code is systematic, and so these bits are never transmitted.
  /// At the decoder, LLRs at these bit positions are replaced with the highest
  /// value of the LLR (+ve LLRs --> logical zero) to represent "certainty" of a transmitted zero.
  static constexpr int8_t k_fillLlrValue = +127;

  /// \struct DecoderParamsInt16
  /// API request for a decoder, with base-graph parameters
  struct DecoderParamsInt16
  {
    /// Pointer to the buffer used to store the code word 8-bit integer LLRs into the
    /// top-level of the decoder. These int8_t values are then cast and stored as int16_t
    /// values for use in the decoder core in the BuildVarNodes() function for bit-growth
    /// in the decoder.
    int8_t* varNodes;

    /// The set of circulant values in use in the basegraph. This is a list of the circular
    /// shifts that form the quasi-cyclic parity-check matrix. For example, in BG1 the first 4 rows
    /// have weight 19, so the first 76 values of this array contain the circulants of the first 4 rows
    /// reading from left to right for each row.
    CACHE_ALIGNED int16_t circulants[SimdLdpc::k_maxCirculants];

    /// The column positions of each of the circulants described above. For the first row of BG1, there
    /// will be 19 positions - one for each circulant value.
    CACHE_ALIGNED int16_t circulantsColPositions[SimdLdpc::k_maxCirculants];

    /// The number of circulants defined for each row of the parity check matrix. For BG1, the first 4 values will be 19
    CACHE_ALIGNED int16_t rowWeights[SimdLdpc::k_maxRows];

    /// The "expansion" or "lifting" factor, as defined in Table 5.3.2-1 of TS38.212v15
    /// The same value as SimdLdpc::Request.z
    int16_t z;

    /// The number of post rate-matched output LLR values.
    /// The same value as SimdLdpc::Request.numChannelLlrs
    int16_t numChannelLlrs;

    /// The number of filler bits used in the encoding.
    /// The same value as SimdLdpc::Request.numFillerBits
    int16_t numFillerBits;

    /// The number of LDPC parity-check rows used in the encoding.
    /// The same value as SimdLdpc::Request.nRows
    int16_t nRows;

    /// The number of columns in the parity-check matrix
    /// This is internally derived from the basegraph type
    int16_t nCols;

    /// The "Offset Min-Sum" correction factor.
    /// 2*atanh(tanh(x[0]/2.0) * tanh(x[1]/2.0 * tanh(x[2]/2.0)) ~=
    /// is approximated to:
    /// min( min(abs(x[0]) - beta, abs(x[1]) - beta, abs(x[2]) - beta), 0.0) * sign(x[0] * x[1] * x[2])
    /// This value sets the number of fractional bits in the LLR number representation.
    /// This value should represent "0.5" so a value of 8 implies that 8-->0.5
    int16_t beta;

    /// The maximum number of iterations that the decoder will perform before it is forced to terminate
    /// The same value as SimdLdpc::Request.maxIterations
    int16_t maxIterations;

    /// When true, the decoder is allowed to terminate before maxIterations if the parity-check equations
    /// all pass.
    bool enableEarlyTermination;

    /// In the 3GPP basegraphs, defined in table 5.3.2-2 and 5.3.2-3 of TS38.212, the top four rows
    /// have a special decoding status:
    /// 1) the entire of the systematic portion is spanned
    /// 2) the parity portion has weight > 1 and is *not* row-orthogonal
    /// Therefore:
    /// 1) parity-checks for early termination need only be performed on these rows
    /// 2) the variable nodes for the entire row need to be written back, as they will be
    ///    read by subsequent rows. In non "kernel" rows, we can be assured that the parity
    ///    variable nodes will never be read by other rows, so need not be written back.
    /// For the best BLER-vs-SINR performance this value should be set to 4.
    /// The other allowed values, of 3 and 2, speed-up the execution at the expense of SINR
    /// performance, as the remaining kernel rows are not scheduled for updates.
    int16_t kernelRowsExecuted;

    /// The basegraph type (BG1 or BG2) as defined in section 5.3.2 of TS38.212v15
    /// This is the same value as SimdLdpc::Request.basegraph
    BaseGraph basegraph;
  };

  /// \struct DecoderResponseInt16
  /// API response for a decoder
  struct DecoderResponseInt16
  {
    /// Pointer to the buffer used to store the code word 16-bit LLR outputs
    int16_t* varNodes;
    /// The number of parity-check matrix rows with errors in the final iteration
    /// A zero means that all of the parity-checks computed in kernelRowsExecuted rows
    /// were satisified, and the decoder has converged on a codeword that is very
    /// likely to be correct.
    int parityErrorCount;

    /// The number of message bits in the output. A value that is computable
    /// from the request, but is done once and stored here for convenience.
    int numMsgBits;

    /// The number of iterations that had executed before termination (either
    /// due to DecoderParamsInt16.maxIterations being reached or parityErrorCount
    /// at zero at the end of the iterations.)
    int iter;
  };

  /// \struct DecoderResponseInt16
  /// Fixed-point layer data. his records information for the active layer within the decoder.
  struct LayerParamsInt16
  {
    /// Parent decoder, which provides data which is used across all layers.
    DecoderParamsInt16* decoder;

    /// The doubled-up version of the parent decoder's LLRs.
    /// TODO: The aligned version doesn't need to be doubled, so reduce in size and rename.
    int16_t* varNodesDbl;

    /// The index of the decoder layer currently being processed.
    int layerIndex;

    /// Scratch pad data which is used across all layers of the decoder,
    /// but which isn't part of the decoder API.
    /// These parameters store the results of the min-sum approximation for each layer
    /// 2*atanh(tanh(x[0]/2.0) * tanh(x[1]/2.0 * tanh(x[2]/2.0)) ~=
    /// is approximated to:
    /// min( min(abs(x[0]) - beta, abs(x[1]) - beta, abs(x[2]) - beta), 0.0) * sign(x[0] * x[1] * x[2])
    /// So, for computing the LLR of x[0] the following is used:
    /// x[0] += min( abs(x[1]) - beta, abs(x[2]) - beta), 0.0) * sign(x[1] * x[2])
    /// And, for computing the LLR of x[1] the following is used:
    /// x[1] += min( abs(x[0]) - beta, abs(x[2]) - beta), 0.0) * sign(x[0] * x[2])
    /// This is implemented by recognising that the minimum function will only have two possible
    /// return values: 1st min and 2nd min. min1pos records the index of x[] where min1 occured,
    /// otherwise the resul is min2.
    /// addSub is the product of sign(x[0] * x[1] * x[2]), so sign(x[1] * x[2]) = addSub * sign(x[0])
    int16_t* min1;
    int16_t* min2;
    int16_t* min1pos;
    int32_t* addSub;

    /// The values of the circulants used in this layer
    int16_t circulants[SimdLdpc::k_maxRowWeight];

    /// The column positions of teh circulants used in this layer
    int16_t* circulantsColPositions;

    /// Double buffer states for the aligned version.
    bool bufferStates[SimdLdpc::k_maxCols] = {};

    /// Record the original circulant positions. Starts out at zero as the first adjustment is by zero.
    int16_t oldCirculantsInPosition[SimdLdpc::k_maxCols] = {};

    /// Column separator for the aligned version
    int16_t z_SIMD;

    /// The variable nodes are written back to each column-position in a ping-pong manner
    /// There are two buffers for the variable nodes, A and B. At the first layer in the
    /// first iteration, each column of varNodes points to a position in buffer#A for reading
    /// and buffer#B for writing. Each time a column is visited (i.e. it is in circulantsColPositions)
    /// the ping-pong state is inverted. This is done because buffer reads are from non-SIMD aligned
    /// positions determined by the values in "circulants", but the writing is aligned to the column-start
    /// position.
    int16_t* readBufferAddresses[SimdLdpc::k_maxCols];
    int16_t* writeBufferAddresses[SimdLdpc::k_maxCols];
  };

  /// \struct LayerOutputsInt16
  /// Layer response data. This consists of parity-checking results and
  /// data for layers scheduling if DecoderParamsInt16.kernelRowsExecuted is less than 4.
  struct LayerOutputsInt16
  {
    /// The status of the parity-check errors in this layer (zero means that the
    /// current codeword hypothesis meets the parity-checks in this layer)
    int32_t parityCheckErrors;

    /// Per-layer norm_1 of the variable-node updates.
    /// These are used to schedule layers, or determine whether this layer
    /// should be executed during the next LDPC iteration
    int32_t updateNorm;
  };

  /// The main decoder function used internally.
  /// \param [in] request structure
  /// \param [out] response structure
  template<typename SIMD>
  void LdpcLayeredDecoderAlignedInt16(SimdLdpc::DecoderParamsInt16& request,
                                      SimdLdpc::DecoderResponseInt16& response);

  /// The decoder function that processes one layer (row) of the parity-checks
  /// this is called once per row, per iteration by LdpcLayeredDecoderAlignedInt16
  /// \param [in] request structure
  /// \param [out] response structure
  template<typename SIMD>
  void LdpcLayerAlignedInt16(LayerParamsInt16& request, LayerOutputsInt16& response);

  /// The aligned decoder uses a non-aligned read/aligned-write strategy. This means that
  /// the variable-nodes are not written back at the expected circulant offsets and are
  /// therefore out of order. This function is called at the end of the decoding process
  /// and it re-aligns back into the natural ordering.
  /// \param [in] request structure
  /// \param [out] response structure
  template<typename SIMD>
  void LdpcAlignedRestore(LayerParamsInt16& request, DecoderResponseInt16& response);

}
