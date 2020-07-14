#!/bin/bash
# Compile the encoder test with Millipede's internal AVX2 encoder and FlexRAN's
# AVX-512 encoder

FLEXRAN_FEC_SDK_DIR="/opt/FlexRAN-FEC-SDK-19-04/sdk"
SOURCES="encoder_test.cpp encoder.cpp cyclic_shift.cpp iobuffer.cpp"

compile_with_millipede_encoder() {
  g++ -std=c++11 -mavx2 -Wall \
    -I. \
    -isystem ${FLEXRAN_FEC_SDK_DIR}/source/phy/lib_ldpc_encoder_5gnr \
    -isystem ${FLEXRAN_FEC_SDK_DIR}/source/phy/lib_common \
    ${SOURCES} -o test_avx2
}

compile_with_flexran_encoder() {
  FLEXRAN_FEC_LIB_DIR=${FLEXRAN_FEC_SDK_DIR}/build-avx512-icc
  g++ -g -std=c++11 -march=native -Wall -no-pie \
    -DUSE_LDPC=on  -D_BBLIB_AVX512_ \
    -I. \
    -isystem ${FLEXRAN_FEC_SDK_DIR}/source/phy/lib_ldpc_encoder_5gnr \
    -isystem ${FLEXRAN_FEC_SDK_DIR}/source/phy/lib_common \
    ${SOURCES} -o test_avx512 \
    ${FLEXRAN_FEC_LIB_DIR}/source/phy/lib_ldpc_encoder_5gnr/libldpc_encoder_5gnr.a \
    ${FLEXRAN_FEC_LIB_DIR}/source/phy/lib_common/libcommon.a
}

compile_with_millipede_encoder
compile_with_flexran_encoder
