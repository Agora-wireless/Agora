# Compile with local libraries and avx2

FLEXRAN_FEC_SDK_DIR="${HOME}/FlexRAN-FEC-SDK-19-04/sdk"

compile_with_millipede_encoder() {
  g++ -std=c++11 -mavx2 -Wall \
    -I. \
    -isystem ${FLEXRAN_FEC_SDK_DIR}/source/phy/lib_ldpc_encoder_5gnr \
    -isystem ${FLEXRAN_FEC_SDK_DIR}/source/phy/lib_common \
    encoder_test.cpp encoder.cpp cyclic_shift.cpp iobuffer.cpp -o test_avx2
}

compile_with_flexran_encoder() {
  FLEXRAN_FEC_LIB_DIR=${FLEXRAN_FEC_SDK_DIR}/build-avx512-icc
  g++ -g -std=c++11 -march=native -Wall -no-pie \
    -DUSE_LDPC=on  -D_BBLIB_AVX512_ \
    -I. \
    -isystem ${FLEXRAN_FEC_SDK_DIR}/source/phy/lib_ldpc_encoder_5gnr \
    -isystem ${FLEXRAN_FEC_SDK_DIR}/source/phy/lib_common \
    encoder_test.cpp \
    ${FLEXRAN_FEC_LIB_DIR}/source/phy/lib_ldpc_encoder_5gnr/libldpc_encoder_5gnr.a \
    ${FLEXRAN_FEC_LIB_DIR}/source/phy/lib_common/libcommon.a \
    -o test_avx512
}

compile_with_flexran_encoder
