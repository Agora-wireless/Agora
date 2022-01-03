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

#include "InternalApi.hpp"
#include "LayerUtilities.hpp"
#include <type_traits>
#include <stdexcept>

// Not all functions from LayerUtils are used, and since they
// are static the compiler will complain
#pragma warning(disable:177)

static inline int ModuloAddress(int x, int y)
{
  const auto s = x - y;
  return s >= 0 ? s : x;
}

// Used to infer the parity type based on SIMD
template<typename T>
struct GetParityType { using type = T; };

template<class T>
struct ParityType
{
  static_assert(!std::is_pointer<T>::value,"Pointer does not have built in type.");
  using type = typename GetParityType< typename std::decay<T>::type >::type;
};

template<>
struct GetParityType<Is16vec16> { using type = int16_t; };
template<>
struct GetParityType<Is16vec32> { using type = int32_t; };


/// Precompute the addresses of each column buffer to avoid having to compute them in the inner-most
/// loop. They are also reused across every SIMD block.
static void ComputeBufferAddresses(SimdLdpc::LayerParamsInt16& request)
{
  const auto buf0 = request.varNodesDbl;
  const auto buf1 = request.varNodesDbl + request.decoder->nCols * request.z_SIMD;

  const auto rowWeight = request.decoder->rowWeights[request.layerIndex];
  const int16_t* columnPositionIndex = request.circulantsColPositions; // :Todo: remove.

  for (int c = 0; c < rowWeight; ++c)
  {
    int colPosition = columnPositionIndex[c];

    const int colOffset = colPosition * request.z_SIMD;
    const auto readBuffer  = request.bufferStates[colPosition] ? buf0 : buf1;
    const auto writeBuffer = request.bufferStates[colPosition] ? buf1 : buf0;

    request.readBufferAddresses[c] = readBuffer + colOffset;
    request.writeBufferAddresses[c] = writeBuffer + colOffset;
  }
}


/// Remove the kernel check node contributions for one set of rows.
/// \param ROW_WEIGHT The number of weights to process for this layer.
/// \param SIMD The type of SIMD to use to process the weights. Typically Is16vec32 or similar.
/// \param PARITY The type of the bits used to process the SIMD. One bit for each element in the
/// SIMD type (e.g., an Is16vec16 would have an int16_t).
template<int ROW_WEIGHT, typename SIMD, typename PARITY>
void LdpcRemoveKernelCheckNodesAligned(const SimdLdpc::LayerParamsInt16& request,
                                       SIMD* scratch, int nz,
                                       const SIMD min1, const SIMD min2, const SIMD min1pos,
                                       SIMD& min1Update, SIMD& min2Update, SIMD& min1PosUpdate,
                                       PARITY& sumProduct, PARITY& sumProductParityCheck,
                                       PARITY* addSubBits)
{
  // The number of 16-bit elements in the given SIMD type.
  constexpr int k_numElements = sizeof(SIMD) / sizeof(int16_t);

  //Variable node adjustments from the previous iteration
  for (int n = 0; n < ROW_WEIGHT; n++)
  {
    // Wrapped offset within the column. :TODO: Compute this as a simd relative offset, so that a
    // direct indexed load can be done? That would move the nz * k_numElements multiplication (which
    // is just an indexed SIMD offset) into the load unit and avoid the latency and insn that is
    // otherwise required.
    const int addrWithinColumn = ModuloAddress(request.circulants[n] + nz * k_numElements, request.decoder->z);

    //Load the variable-node data as an unaligned SIMD data-type
    const auto originalVn = *(SIMD*)(request.readBufferAddresses[n] + addrWithinColumn);

    // Update the sumProduct (parity equation check) by finding the negative values before any updates.
    sumProductParityCheck ^= GetNegativeMask(originalVn);

    // Choose the minimum value, except for when this is the position of the minimum already.
    const auto minValue = SelectEqWorkaround(BroadcastInt16<SIMD>(n), min1pos, min2, min1);

    const auto delta = ApplyParityCorrection(addSubBits[n], minValue);

    const auto vnIn = sat_sub(originalVn, delta);

    // Apply offsets
    const auto newVn = sat_sub_unsigned(abs(vnIn), BroadcastInt16<SIMD>(request.decoder->beta));

    // Now update the new min1, min2 and min1pos
    InsertSort(min1Update, min2Update, min1PosUpdate, n, newVn);

    // Update sumProduct by finding the negative elements.
    const PARITY vnNegBits = GetNegativeMask(vnIn);
    sumProduct ^= vnNegBits;
    addSubBits[n] = vnNegBits;

    //Save back to the write buffer as an aligned write
    scratch[n] = vnIn;
  }

}

/// Add the kernel check node contributions for one set of rows.
/// \param ROW_WEIGHT The number of weights to process for this layer.
/// \param SIMD The type of SIMD to use to process the weights. Typically Is16vec32 or similar.
/// \param PARITY The type of the bits used to process the SIMD. One bit for each element in the
/// SIMD type (e.g., an Is16vec16 would have an int16_t).
template<int ROW_WEIGHT, typename SIMD, typename PARITY>
void LdpcAddKernelCheckNodesAligned(SimdLdpc::LayerParamsInt16& request,
                                    const SIMD* scratch, int nz,
                                    const SIMD min1, const SIMD min2, const SIMD min1pos,
                                    PARITY sumProduct,
                                    PARITY& sumProductParityCheck,
                                    PARITY* addSubBits,
                                    SIMD& scheduleNorm)
{
  //Variable node adjustments to the next iteration
  for (int n = 0; n < ROW_WEIGHT; n++)
  {
    // Choose the minimum value, except for when this is the position of the minimum already.
    const auto delta = SelectEqWorkaround(BroadcastInt16<SIMD>(n), min1pos, min2, min1);

    // Update the addSub parity check. Note that it also updates for the next time this layer is processed.
    const PARITY addSub = PARITY(addSubBits[n] ^ sumProduct);
    addSubBits[n] = addSub;

    const auto delta_signed = ApplyParityCorrection(addSub, delta);

    // Add the variable node update
    auto vnIn = scratch[n];
    auto vnInUpdated = sat_add(vnIn, delta_signed);

    // Update the norm (-ve results imply updates run against the sign of the variable nodes)
    // So, layers with lower norms have more "correction" occurring and can be scheduled in preference
    //scheduleNorm = sat_add(scheduleNorm, ApplyParityCorrection(addSubBits[n], delta_signed));
    scheduleNorm = UpdateScheduleNorm(scheduleNorm, vnIn, delta_signed);

    // Compute the parity, which is set if the value is negative.
    sumProductParityCheck ^= GetNegativeMask(vnInUpdated);

    ((SIMD*)request.writeBufferAddresses[n])[nz] = vnInUpdated;

    // Extra write for the first rows of this layer. This is not an aligned write as it is advanced
    // to + request.decoder->z
    if (nz == 0)
    {
      int16_t* colPtrAsInt = request.writeBufferAddresses[n];
      *(SIMD*)(colPtrAsInt + request.decoder->z) = vnInUpdated;
    }

  }
}

/// Process one simd row-set of the LDPC decoder.
/// \param ROW_WEIGHT The number of weights to process for this layer.
/// \param SIMD The type of SIMD to use to process the weights. Typically Is16vec32 or similar.
/// \param PARITY The type of the bits used to process the SIMD. One bit for each element in the
/// SIMD type (e.g., an Is16vec16 would have an int16_t).
template<int ROW_WEIGHT, typename SIMD>
void LdpcKernelLayerInt16Aligned(SimdLdpc::LayerParamsInt16& request,
                                 SimdLdpc::LayerOutputsInt16& response)
{
  using PARITY = typename ParityType<SIMD>::type;

  constexpr int k_numParityBits = sizeof(PARITY) * 8;
  constexpr float k_recipRowWeight = (float)SimdLdpc::k_maxRowWeight / (float)ROW_WEIGHT;

  const int k_numSimdLoops = GetNumAlignedSimdLoops<SIMD>(request.decoder->z);

  const int cnIdx = request.layerIndex * request.z_SIMD;

  response.parityCheckErrors = 0;

  // Aliases for common pointers.
  SIMD* min1p = (SIMD*)(request.min1 + cnIdx);
  SIMD* min2p = (SIMD*)(request.min2 + cnIdx);
  SIMD* min1posp = (SIMD*)(request.min1pos + cnIdx);

  // The update norm (approx the 1-norm of the difference between the
  // variable nodes and the updates
  SIMD scheduleNorm = SIMD();

  //Remove the old check-node updates from the current VNs
  for (int n = 0; n < k_numSimdLoops; ++n)
  {
    SIMD scratch[ROW_WEIGHT];

    // Each cnIdx points to a single 32-bit block in which addSub information is stored (i.e., each
    // such value can store up to 19-bits of addSub decisions, corresponding to each row. Each loop
    // iteration here processes 16 such blocks. The internal storage is actually used in the
    // opposite direction, but that doesn't matter here.
    PARITY* addSubBits = (PARITY*)(request.addSub + cnIdx + n * k_numParityBits);

    auto min1Update = BroadcastInt16<SIMD>(0x7FFF);
    auto min2Update = BroadcastInt16<SIMD>(0x7FFF);
    auto min1PosUpdate = SIMD();

    PARITY sumProduct = 0;
    PARITY sumProductBeforeUpdate = 0;
    PARITY sumProductAfterUpdate = 0;

    //Remove the old check-nodes
    LdpcRemoveKernelCheckNodesAligned<ROW_WEIGHT>(request, scratch, n, min1p[n], min2p[n],
                                                  min1posp[n], min1Update, min2Update, min1PosUpdate,
                                                  sumProduct, sumProductBeforeUpdate, addSubBits);

    //Variable node updates from this iteration
    LdpcAddKernelCheckNodesAligned<ROW_WEIGHT>(request, scratch, n, min1Update, min2Update, min1PosUpdate,
                                               sumProduct, sumProductAfterUpdate, addSubBits, scheduleNorm);

    //Write out the new check-nodes that we have so far
    min1p[n] = min1Update;
    min2p[n] = min2Update;
    min1posp[n] = min1PosUpdate;

    // Check the before and after parity checks are all zero. The check confirms that the (kernel)
    //layer passed parity before and after the updates.
    response.parityCheckErrors |= (sumProductBeforeUpdate | sumProductAfterUpdate);
  }
  //Update norm computation
  float weighted_norm = (float)GetHorizontalSum(scheduleNorm) * k_recipRowWeight;
  response.updateNorm = (int32_t)weighted_norm;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ORTHOGONAL Layers
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// Process one layer of the LDPC decoder.
/// \param ROW_WEIGHT The number of weights to process for this layer.
/// \param SIMD The type of SIMD to use to process the weights. Typically Is16vec32 or similar.
/// \param PARITY The type of the bits used to process the SIMD. One bit for each element in the
/// SIMD type (e.g., an Is16vec16 would have an int16_t).
template<typename SIMD, typename PARITY>
static void LdpcRemoveOrthogonalCheckNodesAligned(SimdLdpc::LayerParamsInt16& request, int nz,
                                                  const int16_t colIdx, const int16_t addSubIdx,
                                                  SIMD& min1Update, SIMD& min2Update, SIMD& min1PosUpdate,
                                                  PARITY& sumProduct)
{

  // The number of 16-bit elements in the given SIMD type.
  constexpr int k_numElements = sizeof(SIMD)/sizeof(int16_t);

  //Get the address index for this column and buffer
  const int columnPosition = (int)request.circulantsColPositions[colIdx];
  const int readColIndex = request.bufferStates[columnPosition]
                           ? columnPosition + request.decoder->nCols
                           : columnPosition;

  //Get the un-aligned read address for the variable node read
  //We know that request.circulants[colIdx] is zero
  //int addrZ = request.circulants[colIdx] + nz*k_numElements;
  int addrZ = nz * k_numElements;
  addrZ = ModuloAddress(addrZ, request.decoder->z);

  //Add the column (including buffer offset) offset
  addrZ += readColIndex * request.z_SIMD;

  //Load the variable-node data as an unaligned SIMD data-type
  const SIMD vnIn = *(SIMD*)(request.varNodesDbl + addrZ);

  //Apply offsets
  const auto vnWithOffset = sat_sub_unsigned(abs(vnIn), BroadcastInt16<SIMD>(request.decoder->beta));

  //Now update the new min1, min2 and min1pos
  InsertSort(min1Update, min2Update, min1PosUpdate, colIdx, vnWithOffset);

  // Update sumProduct from the mask of negative elements.
  sumProduct ^= GetNegativeMask(vnIn);

  // It is not necessary to save back to SCRATCH variable nodes, as this value will not be updated
  // This is guaranteed to be a parity-bit. No need to store the sign value either for the same
  // reason.
}

/// \param ROW_WEIGHT The number of weights to process for this layer.
/// \param SIMD The type of SIMD to use to process the weights. Typically Is16vec32 or similar.
/// \param PARITY The type of the bits used to process the SIMD. One bit for each element in the
/// SIMD type (e.g., an Is16vec16 would have an int16_t).
template<int ROW_WEIGHT, typename SIMD>
void LdpcOrthogonalLayerInt16Aligned(SimdLdpc::LayerParamsInt16& request)
{
  using PARITY = typename ParityType<SIMD>::type;

  constexpr int k_numParityBits = sizeof(PARITY) * 8;
  const int k_numSimdLoops = GetNumAlignedSimdLoops<SIMD>(request.decoder->z);

  SIMD scratch[ROW_WEIGHT];

  const int cnIdx = request.layerIndex * request.z_SIMD;

  SIMD* min1p = (SIMD*)(request.min1 + cnIdx);
  SIMD* min2p = (SIMD*)(request.min2 + cnIdx);
  SIMD* min1posp = (SIMD*)(request.min1pos + cnIdx);
  SIMD unusedScheduleNorm;

  //Remove the old check-node updates from the current VNs
  for (int n = 0; n < k_numSimdLoops; ++n)
  {
    // Each cnIdx points to a single 32-bit block in which addSub information is stored (i.e., each
    // such value can store up to 19-bits of addSub decisions, corresponding to each row. Each loop
    // iteration here processes 16 such blocks. The internal storage is actually used in the
    // opposite direction, but that doesn't matter here.
    PARITY* addSubBlock = (PARITY*)(request.addSub + cnIdx + n * k_numParityBits);

    auto min1Update = BroadcastInt16<SIMD>(0x7FFF);
    auto min2Update = BroadcastInt16<SIMD>(0x7FFF);
    auto min1PosUpdate = SIMD();

    PARITY sumProduct = 0;
    PARITY unused = 0;

    //Load the check-node data
    SIMD min1 = min1p[n];
    SIMD min2 = min2p[n];
    SIMD min1pos = min1posp[n];

    // Remove the old check-nodes.
    LdpcRemoveKernelCheckNodesAligned<ROW_WEIGHT - 1>(request, scratch, n, min1, min2, min1pos,
                                                      min1Update, min2Update, min1PosUpdate,
                                                      sumProduct, unused, addSubBlock);

    LdpcRemoveOrthogonalCheckNodesAligned(request, n, ROW_WEIGHT - 1, ROW_WEIGHT - 1,
                                          min1Update, min2Update, min1PosUpdate, sumProduct);

    //Variable node updates from this iteration
    LdpcAddKernelCheckNodesAligned<ROW_WEIGHT - 1>(request, scratch, n, min1Update, min2Update,
                                                   min1PosUpdate, sumProduct, unused, addSubBlock,
                                                   unusedScheduleNorm);

    //Write out the new check-nodes that we have so far
    min1p[n] = min1Update;
    min2p[n] = min2Update;
    min1posp[n] = min1PosUpdate;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Top Level Calling functions that select the templates
//////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename SIMD>
static void SimdLdpc::LdpcLayerAlignedInt16(SimdLdpc::LayerParamsInt16& request, SimdLdpc::LayerOutputsInt16& response)
{
  const auto rowWeight = request.decoder->rowWeights[request.layerIndex];

  bool isKernel = (request.layerIndex < SimdLdpc::k_numKernelRows);

  ComputeBufferAddresses(request);

  // Call the single layer LDPC function
  switch (isKernel)
  {
    case 1:
      //The row is a kernel row
      //Weights are 19 for BG1 and 8 & 10 for BG2
      switch (rowWeight)
      {
        case 8:
          LdpcKernelLayerInt16Aligned<8, SIMD>(request, response);
          break;
        case 10:
          LdpcKernelLayerInt16Aligned<10, SIMD>(request, response);
          break;
        case 19:
          LdpcKernelLayerInt16Aligned<19, SIMD>(request, response);
          break;
        default:
          throw std::runtime_error("No Template defined for requested KERNEL row-weight in ldpcLayerInt16TemplateSelect.\n");
      }
      break;

    case 0:
      //The row is not a kernel row
      //Build all weights except 19 (exclusively kernel type for BG1)
      switch (rowWeight)
      {
        case 3: LdpcOrthogonalLayerInt16Aligned<3, SIMD>(request); break;
        case 4: LdpcOrthogonalLayerInt16Aligned<4, SIMD>(request); break;
        case 5: LdpcOrthogonalLayerInt16Aligned<5, SIMD>(request); break;
        case 6: LdpcOrthogonalLayerInt16Aligned<6, SIMD>(request); break;
        case 7: LdpcOrthogonalLayerInt16Aligned<7, SIMD>(request); break;
        case 8: LdpcOrthogonalLayerInt16Aligned<8, SIMD>(request); break;
        case 9: LdpcOrthogonalLayerInt16Aligned<9, SIMD>(request); break;
        case 10: LdpcOrthogonalLayerInt16Aligned<10, SIMD>(request); break;
        case 11: LdpcOrthogonalLayerInt16Aligned<11, SIMD>(request); break;
        case 12: LdpcOrthogonalLayerInt16Aligned<12, SIMD>(request); break;
        case 13: LdpcOrthogonalLayerInt16Aligned<13, SIMD>(request); break;
        case 14: LdpcOrthogonalLayerInt16Aligned<14, SIMD>(request); break;
        case 15: LdpcOrthogonalLayerInt16Aligned<15, SIMD>(request); break;
        case 16: LdpcOrthogonalLayerInt16Aligned<16, SIMD>(request); break;
        case 17: LdpcOrthogonalLayerInt16Aligned<17, SIMD>(request); break;
        case 18: LdpcOrthogonalLayerInt16Aligned<18, SIMD>(request); break;

        default:
          throw std::runtime_error("No template defined for requested row-weight in ldpcLayerInt16TemplateSelect.\n");
      }
      break;

    default:
      throw std::runtime_error("isKernel should be 1 or 0.\n");
  }
}

template <typename SIMD>
static void SimdLdpc::LdpcAlignedRestore(SimdLdpc::LayerParamsInt16& request, SimdLdpc::DecoderResponseInt16& response)
{
  const int k_numSimdLoops = GetNumAlignedSimdLoops<SIMD>(request.decoder->z);
  constexpr int k_numElements = sizeof(SIMD) / sizeof(int16_t);

  //Go through each LDPC column in turn
  //TODO:: ONLY DO THIS WITH SYSTEMATIC COLS per BASEGRAPH
  for (int nc = 0; nc < request.decoder->nCols; ++nc)
  {
    // Read from the last buffer written to. :TODO: Use the read/write buffer addresses?
    const int buffState = request.bufferStates[nc] ? request.decoder->nCols * request.z_SIMD : 0;
    const int zr = (request.decoder->z - request.oldCirculantsInPosition[nc]) % (request.decoder->z);
    const int colAddrRd = nc * request.z_SIMD + buffState;
    const int colAddrWr = nc * request.decoder->z;

    for (int n = 0; n < k_numSimdLoops; ++n)
    {
      int zi = zr + n * k_numElements;
      zi = zi % request.decoder->z;

      // Read addr is an index into an int16_t
      // Write addr in an index into a SIMD type
      const int rdIdx = colAddrRd + zi;

      SIMD rdVal = *((SIMD*)(request.varNodesDbl + rdIdx));
      ((SIMD*)(response.varNodes + colAddrWr))[n] = rdVal;
    }
  }
}


template void
SimdLdpc::LdpcAlignedRestore<Is16vec16>(LayerParamsInt16& request, DecoderResponseInt16& response);

template void
SimdLdpc::LdpcAlignedRestore<Is16vec32>(LayerParamsInt16& request, DecoderResponseInt16& response);

template void
SimdLdpc::LdpcLayerAlignedInt16<Is16vec16>(LayerParamsInt16& request, LayerOutputsInt16& response);

template void
SimdLdpc::LdpcLayerAlignedInt16<Is16vec32>(LayerParamsInt16& request, LayerOutputsInt16& response);
