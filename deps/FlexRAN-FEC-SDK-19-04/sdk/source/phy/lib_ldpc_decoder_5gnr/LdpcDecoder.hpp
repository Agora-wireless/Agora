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

#pragma once

#include "ProjectConfig.hpp"

namespace SimdLdpc
{

  /// \enum  BaseGraph
  /// There are two broad basegraphs defined by 3GPP:
  /// Basegraph#1 (BG1) in table 5.3.2-2 of TS38.212v15
  /// Basegraph#2 (BG2) in table 5.3.2-3 of TS38.212v15
  enum class BaseGraph { BG1 = 1, BG2 = 2 };

  /// \struct Request
  /// API request for a top-level invocation of the decoder
  struct Request
  {
    /// Pointer to the buffer used to store the code word 8-bit integer LLRs.
    ///    This should be z*22 + z*nRows - z*2 - numFillerBits in length for BG1
    ///    This should be z*10 + z*nRows - z*2 - numFillerBits in length for BG2
    int8_t* varNodes;

    /// The "expansion-factor" or "lifting-factor" defined in Table 5.3.2-1 of TS38.212v15
    /// This must be a value equal to z = A_factor * {1,2,4,8,16,32,64,128} for all Z <= 384
    /// A_factor is taken from the set {2,3,5,7,9,11,13,15}
    /// The permissible values of z are:
    /// {2, 4, 8, 16, 32, 64, 128, 256}, {3, 6, 12, 24, 48, 96, 192, 384}
    /// {5, 10, 20, 40, 80, 160, 320}, {7, 14, 28, 56, 112, 224}
    /// {9, 18, 36, 72, 144, 288}, {11, 22, 44, 88, 176, 352}
    /// {13, 26, 52, 104, 208}, {15, 30, 60, 120, 240}
    int16_t z;

    /// This is the number of channel bits (LLRs) presented to the decoder after reverse rate-matching
    int16_t numChannelLlrs;

    /// The number of message bits that have been added to the message into the *encoder* that were required
    /// to fulfill the allowed codeblock size for the selected basegraph and lifting-factor (z)
    /// When the allowed codeblock size exceeds the message size, logical ZERO is appended to the message.
    /// The number of logical zeros appended is indicated here. These "filler" or "shortening" bits are
    /// removed from the decoded message, as they conveyed no information.
    int16_t numFillerBits;

    /// The number of active rows in the basegraph. It is a reverse rate-matching output parameter.
    /// MUST be >= 4
    int16_t nRows;

    /// The maximum number of iterations that the decoder will perform before it is forced to terminate
    int16_t maxIterations;

    /// If true --> the decoder will terminate when the parity checks pass if current iteration number is
    /// equal to or less than maxIterations.
    bool enableEarlyTermination;

    /// The basegraph type (BG1 or BG2) as defined in section 5.3.2 of TS38.212v15
    /// This, along with z and nRows allows the decoder to construct the parity-check matrix.
    BaseGraph basegraph;
  };

  /// \struct Response
  /// API response from top-level invocation of the decoder
  struct Response
  {
    /// Pointer to the buffer used to store the code word 16-bit LLR outputs
    ///    Space allocation *must* be  >= z*22 + z*nRows for BG1
    ///    Space allocation *must* be  >= z*10 + z*nRows for BG2
    ///    These represent the decoders estimation of the entire codeword, with per-bit reliability
    ///    represented as LLRs (Log Likelihood Ratios)
    int16_t* varNodes;

    /// Output message stored as individual bits in a pre-allocated buffer.
    /// Number of bytes allocation *must* be  >= ceil((z*22 - numFillerBits)/8) for BG1
    /// Number of bytes allocation *must* be  >= ceil((z*10 - numFillerBits)/8) for BG2
    int numMsgBits;

    /// The decoded output is compacted into bytes.
    /// Bit#0 of the message is placed into Bit#7 of Byte#0 of the compacted message
    /// Bit#1 of the message is placed into Bit#6 of Byte#0 of the compacted message
    /// ..etc
    uint8_t* compactedMessageBytes;

    /// The number of iterations executed before termination.
    int iterationAtTermination;

    /// True if the parity checks all had passed at termination  (Always true if
    /// response.iterationAtTermination  < request.maxIterations).
    bool parityPassedAtTermination;
  };

  /// Top level AVX2 decoder function
  /// \param [in] request structure
  /// \param [out] response structure
  void DecodeAvx2(const SimdLdpc::Request* request, SimdLdpc::Response *response);

  /// Top level AVX512 decoder function
  /// \param [in] request structure
  /// \param [out] response structure
  void DecodeAvx512(const SimdLdpc::Request* request, SimdLdpc::Response *response);
};
