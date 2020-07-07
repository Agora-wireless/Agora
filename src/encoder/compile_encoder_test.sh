# Compile with local libraries and avx2
#g++ -std=c++11 -mavx2 -Wall \
#  -I. \
#  -isystem /opt/FlexRAN-FEC-SDK-19-04/sdk/source/phy/lib_ldpc_encoder_5gnr \
#  -isystem /opt/FlexRAN-FEC-SDK-19-04/sdk/source/phy/lib_common \
#  encoder_test.cpp encoder.cpp cyclic_shift.cpp iobuffer.cpp -o test_avx2

# Compile with FlexRAN
FLEXRAN_ENCODER_SRC=/users/ankalia/sandbox/FlexRAN-FEC-SDK-19-04/sdk/source/phy/lib_ldpc_encoder_5gnr

icpc -std=c++11 -march=native -xSKYLAKE-AVX512 -Wall \
  -DUSE_LDPC=on  -D_BBLIB_AVX512_ \
  -I. \
  -isystem /opt/FlexRAN-FEC-SDK-19-04/sdk/source/phy/lib_ldpc_encoder_5gnr \
  -isystem /opt/FlexRAN-FEC-SDK-19-04/sdk/source/phy/lib_common \
  encoder_test.cpp \
  ${FLEXRAN_ENCODER_SRC}/phy_ldpc_encoder_5gnr_avx512.cpp \
  ${FLEXRAN_ENCODER_SRC}/phy_ldpc_encoder_5gnr.cpp \
  ${FLEXRAN_ENCODER_SRC}/ldpc_encoder_cycshift.cpp \
  -L /users/ankalia/sandbox/FlexRAN-FEC-SDK-19-04/sdk/build-avx512-icc/source/phy/lib_common \
  -lcommon \
  -o test_avx512

#icpc -std=c++11 -mavx2 -Wall -I. encoder_test.cpp encoder.cpp cyclic_shift.cpp iobuffer.cpp -o test
