// INTEL CONFIDENTIAL
// Copyright 2009-2019 Intel Corporation All Rights Reserved.
// 
// The source code contained or described herein and all documents related to the
// source code ("Material") are owned by Intel Corporation or its suppliers or
// licensors. Title to the Material remains with Intel Corporation or its
// suppliers and licensors. The Material may contain trade secrets and proprietary
// and confidential information of Intel Corporation and its suppliers and
// licensors, and is protected by worldwide copyright and trade secret laws and
// treaty provisions. No part of the Material may be used, copied, reproduced,
// modified, published, uploaded, posted, transmitted, distributed, or disclosed
// in any way without Intel's prior express written permission.
// 
// No license under any patent, copyright, trade secret or other intellectual
// property right is granted to or conferred upon you by disclosure or delivery
// of the Materials, either expressly, by implication, inducement, estoppel or
// otherwise. Any license under such intellectual property rights must be
// express and approved by Intel in writing.
// 
// Unless otherwise agreed by Intel in writing, you may not remove or alter this
// notice or any other notice embedded in Materials by Intel or Intel's suppliers
// or licensors in any way.
// 
//  version: SDK-jenkins-FlexRAN-SDK-REL-448-g3be238

#include "LayerUtilities.hpp"
#include "LdpcDecoder.hpp"
#include "InternalApi.hpp"

#include <array>
#include <algorithm>
#include <numeric>

// Not all functions from LayerUtils are used, and since they
// are static the compiler will complain
#pragma warning(disable:177)

/// Note that the temporary memory storage for each layer decode is allocated statically per
/// thread. This storage is used by a single run of the decoder. Multiple runs of the decoder by
/// different threads will each get their own temporary storage.
thread_local static CACHE_ALIGNED int16_t g_min1[SimdLdpc::k_maxZ * SimdLdpc::k_maxRows];
thread_local static CACHE_ALIGNED int16_t g_min2[SimdLdpc::k_maxZ * SimdLdpc::k_maxRows];
thread_local static CACHE_ALIGNED int16_t g_min1pos[SimdLdpc::k_maxZ * SimdLdpc::k_maxRows];

// Buffer below is sized for the aligned version
// :TODO: rename and reduce in size - no longer double.
thread_local static CACHE_ALIGNED int16_t g_varNodesDbl[2 * SimdLdpc::k_maxCols * SimdLdpc::k_maxZ];

// This buffer must be big enough to store up to 19 bits for each row. A 32-bit storage is
// therefore overkill, but does give a bit of room for improving memory alignment within the
// storage. The layout of the storage is decided by the kernel code.
thread_local static CACHE_ALIGNED int32_t g_addSub[SimdLdpc::k_maxZ * SimdLdpc::k_maxRows];

thread_local static CACHE_ALIGNED int16_t g_circulantsInPosition[SimdLdpc::k_maxCols * SimdLdpc::k_maxRows];

/// A sorting function that returns the indices of the given values in ascending order.
static void SortIndex(int32_t (&vals)[SimdLdpc::k_numKernelRows],
                      int (&index)[SimdLdpc::k_numKernelRows])
{
  // initialize original index locations
  std::array<int, SimdLdpc::k_numKernelRows> idx;
  std::iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in vals
  // < for ascending result (min-to-max)
  // stable_sort to match Matlab behaviour - not necessary for performance
  std::sort(idx.begin(), idx.end(),
    [&vals](size_t i1, size_t i2) {return (vals[i1] < vals[i2]); });

  std::copy_n(idx.data(), SimdLdpc::k_numKernelRows, index);
}

/// A function that adjusts the base graph circulants so that each write
/// to each layer can be aligned. This really modifies the read circulants
/// for each column so that circlant read is non-aligned and each subsequent
/// column write is aligned.
static void AdjustCirculantReads(SimdLdpc::LayerParamsInt16& request,
                                 const int16_t* explicitCirculants,
                                 const int16_t* columnPositionIndex)
{
  int16_t adjustedCirculants[SimdLdpc::k_maxCols];
  const int rowWeight = request.decoder->rowWeights[request.layerIndex];

  if (request.decoder->nCols < 1) // Unlikely
    request.decoder->nCols = 1;

  for (int c = 0; c < request.decoder->nCols; ++c)
  {
    int16_t adjusted_val = (uint16_t)(explicitCirculants[c] - request.oldCirculantsInPosition[c]);
    // Modulo z
    adjustedCirculants[c] = (adjusted_val < (int16_t)0)
                            ? (int16_t)(request.decoder->z + adjusted_val)
                            : adjusted_val;
  }

  for (int c = 0; c < rowWeight; ++c)
  {
    int colPosition = columnPositionIndex[c];

    request.oldCirculantsInPosition[colPosition] = explicitCirculants[colPosition];

    // Compress the explicit circulants (which have the unused columns in them) into
    // a list of used columns only
    request.circulants[c] = adjustedCirculants[colPosition];
  }
}

template<typename SIMD>
static int SelectZSimd(int16_t z)
{
  // Get the SIMD equivalent for the expansion factor

  constexpr int SIMD_LEN = sizeof(SIMD) / sizeof(int16_t);
  int zSIMD;

  //Select zSIMD so that the modulo arithmetic works with the SIMD read/write structure
  if (z < SimdLdpc::k_cacheInt16Alignment)
  {
    // This is equivalent to ceil(z/SIMD_LEN) * SIMD_LEN;
    zSIMD = RoundUpDiv(z , SIMD_LEN) * SIMD_LEN;

    if ((zSIMD - z) < SIMD_LEN)
      zSIMD = zSIMD + SIMD_LEN;
  }
  else
  {
    // This is equivalent to ceil(z/k_cacheInt16Alignment) * k_cacheInt16Alignment;
    zSIMD = RoundUpDiv(z , SimdLdpc::k_cacheInt16Alignment) * SimdLdpc::k_cacheInt16Alignment;

    if ((zSIMD - z) < SIMD_LEN)
      zSIMD = zSIMD + SimdLdpc::k_cacheInt16Alignment;
  }

  return zSIMD;
}

// BlockCopy repeatedly copies nBlocks of length z plus a resdiual amount that is
// always less than z
template <typename FROM_TYPE, typename TO_TYPE = int16_t>
static void BlockCopy(const FROM_TYPE* from, int16_t z, int nBlocks, int nResidual, TO_TYPE* to)
{
  for (int n = 0; n < nBlocks; ++n)
    std::copy_n(from, z, to + n*z);

  std::copy_n(from, nResidual, to + nBlocks*z);
}


static int BuildVarNodes(const SimdLdpc::DecoderParamsInt16& request, int zSIMD)
{
  // Copy varNodesIn to request.varNodesDbl
  // Double buffered, but only the first buffer needs to be initialised
  // Using std::copy_n to convert from int8_t to int16_t
  // First two columns are always zeros
  std::fill_n(g_varNodesDbl, 2 * zSIMD, 0);

  //How many columns of systematic bits without filling?
  int nSysCols;
  if (request.basegraph == SimdLdpc::BaseGraph::BG2)
    nSysCols = 10 - 2;
  else
    nSysCols = 22 - 2;

  // This is equivalent to nSysCols - ceil(request.numFillerBits / request.z)
  int maxNumFillerCols = RoundUpDiv(request.numFillerBits, request.z);
  int nSystematicColsCopy = nSysCols - maxNumFillerCols;

  // Straight array copy of systematic bits, repeating numBlockRepeats times
  // plus numResidualRepeats where necessary to fill zSIMD
  int numBlockRepeats = zSIMD / request.z;
  int numResidualRepeats = zSIMD - request.z * numBlockRepeats;
  // 1st two columns never transmitted
  for (int n = 0; n < nSystematicColsCopy; ++n)
  {
    BlockCopy<int8_t>(request.varNodes + n * request.z,
      request.z,
      numBlockRepeats,
      numResidualRepeats,
      g_varNodesDbl + (n + 2) * zSIMD);
  }

  // Copy of remaining systematic bits + residual column fill of max +ve LLR
  if (request.numFillerBits > 0)
  {
    //One quick std::fill_n to write every column with fillers with max +ve LLR
    //Then write over with the residual LLRs that were sent
    std::fill_n(g_varNodesDbl + (nSystematicColsCopy + 2) * zSIMD,
                maxNumFillerCols * zSIMD,
                SimdLdpc::k_fillLlrValue);

    int residualSystematicBits = maxNumFillerCols*request.z - request.numFillerBits;

    std::copy_n(request.varNodes + nSystematicColsCopy * request.z,
      residualSystematicBits,
      g_varNodesDbl + (nSystematicColsCopy + 2) * zSIMD);

    // Now block repeat to fill the zSIMD column
    BlockCopy<int16_t>(g_varNodesDbl + (nSystematicColsCopy + 2) * zSIMD,
      request.z,
      numBlockRepeats,
      numResidualRepeats,
      g_varNodesDbl + (nSystematicColsCopy + 2) * zSIMD);
  }

  //The input LLR buffer won't always be a complete number of columns
  int nParityBitsIn = request.numChannelLlrs - (nSysCols * request.z) + request.numFillerBits;
  int nFullParityColumns = nParityBitsIn / request.z;
  int finalFullColumn = nSysCols + nFullParityColumns;
  int nParityResidual = nParityBitsIn - nFullParityColumns * request.z;

  //Straight copy of the remaining parity LLRs
  for (int n = nSysCols; n < finalFullColumn; ++n)
  {
    BlockCopy<int8_t>(request.varNodes + n * request.z - request.numFillerBits,
      request.z,
      numBlockRepeats,
      numResidualRepeats,
      g_varNodesDbl + (n + 2) * zSIMD);
  }

  // Copy of remaining parity bits that partially fill the final column
  int totalParityColumnsFilled = nFullParityColumns;
  if (nParityResidual > 0)
  {

    std::copy_n(request.varNodes + finalFullColumn * request.z - request.numFillerBits,
      nParityResidual,
      g_varNodesDbl + (finalFullColumn + 2) * zSIMD);

    // Rest are don't knows (punctured)
    std::fill_n(g_varNodesDbl + (finalFullColumn + 2) * zSIMD + nParityResidual,
      request.z - nParityResidual, 0);

    // Now this column has been filled: increment totalParityColumnsFilled
    totalParityColumnsFilled += 1;

    BlockCopy<int16_t>(g_varNodesDbl + (finalFullColumn + 2) * zSIMD,
      request.z,
      numBlockRepeats,
      numResidualRepeats,
      g_varNodesDbl + (finalFullColumn + 2) * zSIMD);
  }

  // In very high-rate cases for RV_IDX#0, the final columns may not be presented by the rate-matching
  int totalUnFilledColumns = SimdLdpc::k_numKernelRows - totalParityColumnsFilled;
  if (totalUnFilledColumns > 0)
  {
    //Safety fill of last columns
    int finalColumnsIdx = nSysCols + 2 + totalParityColumnsFilled;
    std::fill_n(g_varNodesDbl + zSIMD * finalColumnsIdx, totalUnFilledColumns * zSIMD, 0);
  }

  //It is convienient for the decoder core to return the number of message bits
  return request.z * (nSysCols + 2) - request.numFillerBits;
}

template<typename SIMD>
void SimdLdpc::LdpcLayeredDecoderAlignedInt16(SimdLdpc::DecoderParamsInt16& request,
                                              SimdLdpc::DecoderResponseInt16& response)
{
  int zSIMD = SelectZSimd<SIMD>(request.z);

  response.numMsgBits = BuildVarNodes(request, zSIMD);

  //Build the set of circulants in their column positions
  int rdCnt = 0;
  for (int r = 0; r < request.nRows; ++r)
  {
    for (int c = 0; c < request.rowWeights[r]; ++c)
    {
      g_circulantsInPosition[request.circulantsColPositions[rdCnt] + r * request.nCols] = request.circulants[rdCnt];
      ++rdCnt;
    }
  }

  //Fixed parameters for each layer
  SimdLdpc::LayerParamsInt16 layerRequest;

  //These request assignments do not change per iteration.
  layerRequest.varNodesDbl = g_varNodesDbl;

  layerRequest.min1 = g_min1;
  layerRequest.min2 = g_min2;
  layerRequest.min1pos = g_min1pos;
  layerRequest.addSub = g_addSub;
  layerRequest.z_SIMD = (int16_t)zSIMD;
  layerRequest.decoder = &request;

  // Min1/2 are assumed to be zeroed for the first iteration. If the first iteration is ever
  // specialised, this can be avoided.
  std::fill_n(g_min1, layerRequest.z_SIMD * request.nRows, 0);
  std::fill_n(g_min2, layerRequest.z_SIMD * request.nRows, 0);

  // Create an initial layer sequence.
  int layerSequence[SimdLdpc::k_numKernelRows] = {0, 1, 2, 3};

  // The locations of the circulant for the Kernel rows
  // 0,1,2,3 * 19 for BG1. Not for BG2.
  // Compute from the first k_numKernelRows (4) row-weights
  const int kernelRowPositions[SimdLdpc::k_numKernelRows] =
  {
    0,
    request.rowWeights[0],
    request.rowWeights[0] + request.rowWeights[1],
    request.rowWeights[0] + request.rowWeights[1] + request.rowWeights[2]
  };

  // The index of the first non-kernel row
  int startOfNonKernel =
    kernelRowPositions[SimdLdpc::k_numKernelRows - 1] + request.rowWeights[SimdLdpc::k_numKernelRows - 1];

  int iter;
  //Early termination initialiser. If > 0, then will never terminate early
  int earlyTerminateInitialiser = (request.enableEarlyTermination == true) ? 0 : 1;

  //Main decoder iterations loop
  int lastIter;
  int updateNorm[SimdLdpc::k_numKernelRows] = { 0 };
  int32_t parityErrorCount;

  if (request.maxIterations < 1) // Unlikely
    request.maxIterations  = 1;

  for (iter = 0; iter < request.maxIterations; ++iter)
  {
    //Go through each row, starting with the kernel rows
    SimdLdpc::LayerOutputsInt16 layerResponse;
    parityErrorCount = earlyTerminateInitialiser;

    for (int nIdx = 0; nIdx < request.kernelRowsExecuted; ++nIdx)
    {
      const int n = layerSequence[nIdx];
      layerRequest.layerIndex = n;

      //Modify the circulants for aligned-writes of every circulant matrix back to position zero
      //This is required for all layers after the first layer

      //Pointer to this row of basegraph circulants
      int16_t* explicitCirculants = g_circulantsInPosition + n * request.nCols;
      int16_t* colPosPtr = request.circulantsColPositions + kernelRowPositions[n];

      //Adjust the circulants from the previous aligned write so that the new non-aligned reads are
      //in the correct position
      AdjustCirculantReads(layerRequest, explicitCirculants, colPosPtr);

      //Update the buffer states
      for (int c = 0; c < request.rowWeights[n]; ++c)
        layerRequest.bufferStates[colPosPtr[c]] = !layerRequest.bufferStates[colPosPtr[c]];

      layerRequest.circulantsColPositions = request.circulantsColPositions + kernelRowPositions[n];

      //Call the single layer LDPC function
      SimdLdpc::LdpcLayerAlignedInt16<SIMD>(layerRequest, layerResponse);

      //Kernel-only parity-check
      parityErrorCount += (layerResponse.parityCheckErrors != 0) ? 1 : 0;
      updateNorm[n] = layerResponse.updateNorm;
    }

    //Early termination (before we start the non-kernel rows)
    //Break the iterations loop
    //Only break if more than one iteration has executed. This avoids the
    //all-zeros codeword trap, as we currently treat an LLR of 0 as a +VE number
    //in GetNegativeMask()
    lastIter = iter;
    if ((parityErrorCount == 0) && (iter > 1))
      break;

    // Get the execution sequence for the next iteration (if necessary)
    // Otherwise it remains at {0,1,2,3}
    // For BG1 there is no constraint in order
    // For BG2:
    // Row weights {8,10,8,10}
    //1 1 1 1 0 0 1 0 0 1 1 1 0 0
    //1 0 0 1 1 1 1 1 1 1 0 1 1 0
    //1 1 0 1 1 0 0 0 1 0 1 0 1 1
    //0 1 1 0 1 1 1 1 1 1 1 0 0 1
    // Row 0 & Row 1 best executed together
    // Row 2 & Row 3 best executed together
    if (request.kernelRowsExecuted < SimdLdpc::k_numKernelRows)
    {
      SortIndex(updateNorm, layerSequence);
      //Reset the update norms
      std::fill_n(updateNorm, SimdLdpc::k_numKernelRows, 0);
    }

    //BG2 2-kernel special handling
    if ((request.basegraph == SimdLdpc::BaseGraph::BG2) && (request.kernelRowsExecuted == 2))
    {
      switch (layerSequence[0])
      {
        case 0: layerSequence[1] = 1; break;
        case 1: layerSequence[1] = 0; break;
        case 2: layerSequence[1] = 3; break;
        case 3: layerSequence[1] = 2; break;
      }
    }

    //These request assignments need to be re-assigned to the start of the non-kernel rows
    layerRequest.circulantsColPositions = request.circulantsColPositions + startOfNonKernel;

    //Execute the non-kernel rows
    for (int n = SimdLdpc::k_numKernelRows; n < request.nRows; ++n)
    {
      layerRequest.layerIndex = n;

      //Pointer to this row of basegraph circulants
      int16_t* explicitCirculants = g_circulantsInPosition + n*request.nCols;
      int16_t* colPosPtr = layerRequest.circulantsColPositions;

      //Adjust the circulants from the previous aligned write so that the new non-aligned reads are
      //in the correct position
      AdjustCirculantReads(layerRequest, explicitCirculants, colPosPtr);

      //Update the buffer states
      //Non-kernel layers, so the final column is not written (and buffer state is not updated)
      for (int c = 0; c < request.rowWeights[n] - 1; ++c)
        layerRequest.bufferStates[colPosPtr[c]] = !layerRequest.bufferStates[colPosPtr[c]];

      //Call the single layer LDPC function
      SimdLdpc::LdpcLayerAlignedInt16<SIMD>(layerRequest, layerResponse);


      //Increase the indices
      layerRequest.circulantsColPositions += request.rowWeights[n];
    }

  }

  // Remove the double-buffering: re-align data into request.varNodes
  SimdLdpc::LdpcAlignedRestore<SIMD>(layerRequest, response);


  response.iter = lastIter + 1;
  response.parityErrorCount = parityErrorCount - earlyTerminateInitialiser;
}

template void
SimdLdpc::LdpcLayeredDecoderAlignedInt16<Is16vec16>(SimdLdpc::DecoderParamsInt16& request,
                                                                  SimdLdpc::DecoderResponseInt16& response);
template void
SimdLdpc::LdpcLayeredDecoderAlignedInt16<Is16vec32>(SimdLdpc::DecoderParamsInt16& request,
                                                             SimdLdpc::DecoderResponseInt16& response);
