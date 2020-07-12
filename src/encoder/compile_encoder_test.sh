# Compile with local libraries and avx2

FLEXRAN_SDK="${HOME}/FlexRAN-FEC-SDK-19-04/sdk"

compile_with_millipede_encoder() {
  g++ -std=c++11 -mavx2 -Wall \
    -I. \
    -isystem ${FLEXRAN_SDK}/source/phy/lib_ldpc_encoder_5gnr \
    -isystem ${FLEXRAN_SDK}/source/phy/lib_common \
    encoder_test.cpp encoder.cpp cyclic_shift.cpp iobuffer.cpp -o test_avx2
}

compile_with_flexran_encoder() {
  icpc -g -debug all -std=c++11 -march=native -Wall -check-pointers=write \
    -DUSE_LDPC=on  -D_BBLIB_AVX512_ \
    -I. \
    -isystem ${FLEXRAN_SDK}/source/phy/lib_ldpc_encoder_5gnr \
    -isystem ${FLEXRAN_SDK}/source/phy/lib_common \
    encoder_test.cpp \
    ${FLEXRAN_SDK}/source/phy/lib_ldpc_encoder_5gnr/phy_ldpc_encoder_5gnr_avx512.cpp \
    ${FLEXRAN_SDK}/source/phy/lib_ldpc_encoder_5gnr/phy_ldpc_encoder_5gnr.cpp \
    ${FLEXRAN_SDK}/source/phy/lib_ldpc_encoder_5gnr/ldpc_encoder_cycshift.cpp \
    -L ${FLEXRAN_SDK}/build-avx512-icc/source/phy/lib_common \
    -lcommon \
    -o test_avx512
}

compile_with_flexran_encoder
