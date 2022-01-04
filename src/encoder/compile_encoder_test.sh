#!/bin/bash
# Compile the encoder test with Agora's internal AVX2 encoder and FlexRAN's
# AVX-512 encoder

FLEXRAN_FEC_SDK_DIR="/opt/FlexRAN-FEC-SDK-19-04/sdk"
SOURCES="encoder_test.cc encoder.cc cyclic_shift.cc iobuffer.cc"
CPU_FEATURES_DETECT_AVX512=`cat /proc/cpuinfo | grep avx512 | wc -l`

compile_with_agora_encoder() {
  g++ -std=c++17 -mavx2 -Wall -DUSE_AVX2_ENCODER \
    -I. \
    -isystem ${FLEXRAN_FEC_SDK_DIR}/source/phy/lib_ldpc_encoder_5gnr \
    -isystem ${FLEXRAN_FEC_SDK_DIR}/source/phy/lib_common \
    ${SOURCES} -o test_avx2
}

compile_with_flexran_encoder() {
  FLEXRAN_FEC_LIB_DIR=${FLEXRAN_FEC_SDK_DIR}/build-avx512-icc
  g++ -g -std=c++17 -march=native -Wall -no-pie \
    -D_BBLIB_AVX512_ \
    -I. \
    -isystem ${FLEXRAN_FEC_SDK_DIR}/source/phy/lib_ldpc_encoder_5gnr \
    -isystem ${FLEXRAN_FEC_SDK_DIR}/source/phy/lib_common \
    ${SOURCES} -o test_avx512 \
    ${FLEXRAN_FEC_LIB_DIR}/source/phy/lib_ldpc_encoder_5gnr/libldpc_encoder_5gnr.a \
    ${FLEXRAN_FEC_LIB_DIR}/source/phy/lib_common/libcommon.a
}

compile_with_agora_encoder

if [ ${CPU_FEATURES_DETECT_AVX512} -ne 0 ]; then
  echo "Compiling with FlexRAN's encoder"
  compile_with_flexran_encoder
else
  echo "Skipping compiling with FlexRAN's encoder"
fi
