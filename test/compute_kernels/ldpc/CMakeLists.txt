# Cmake 2.8.12 needed for "add_compile_options" command
cmake_minimum_required(VERSION 2.8.12)
include(CheckCSourceRuns)
set(CMAKE_CXX_FLAGS "-std=c++11 -O3 -march=native")

# New versions of GCC enable position-independent executables by default. We
# don't want to compile FlexRAN libraries with -fPIC because it reduces
# performance by around 40%.
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -no-pie")

set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../..)
set(FLEXRAN_FEC_SDK_DIR "${SOURCE_DIR}/src/third_party/FlexRAN-FEC-SDK-19-04/sdk")

CHECK_C_SOURCE_RUNS("int main() { asm volatile(\"vmovdqu64 %zmm0, %zmm1\"); return 0; }" ISA_AVX512)
if (ISA_AVX512)
  message(STATUS "AVX-512 supported.")
  set(FLEXRAN_FEC_LIB_DIR ${FLEXRAN_FEC_SDK_DIR}/build-avx512-icc)
else()
  message(STATUS "AVX-512 not supported.")
  set(FLEXRAN_FEC_LIB_DIR ${FLEXRAN_FEC_SDK_DIR}/build-avx2-icc)
endif()

set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../..)
include_directories(
  /opt/intel/mkl/include
  ${FLEXRAN_FEC_SDK_DIR}/test/phy/common
  ${FLEXRAN_FEC_SDK_DIR}/source/phy/lib_ldpc_decoder_5gnr
  ${FLEXRAN_FEC_SDK_DIR}/source/phy/lib_ldpc_encoder_5gnr
  ${FLEXRAN_FEC_SDK_DIR}/source/phy/lib_common
  ${SOURCE_DIR}/src
  ${SOURCE_DIR}/src/encoder)

set(SOURCES 
  ${SOURCE_DIR}/src/encoder/cyclic_shift.cpp
  ${SOURCE_DIR}/src/encoder/encoder.cpp
  ${SOURCE_DIR}/src/encoder/iobuffer.cpp)

add_definitions(-DUSE_LDPC)

add_executable(test_ldpc test_ldpc.cpp ${SOURCES})
target_link_libraries (test_ldpc
  ${FLEXRAN_FEC_LIB_DIR}/source/phy/lib_ldpc_decoder_5gnr/libldpc_decoder_5gnr.a
  ${FLEXRAN_FEC_LIB_DIR}/source/phy/lib_common/libcommon.a)

if(${ISA_AVX512}) 
  add_executable(test_ldpc_flexran test_ldpc_flexran.cpp ${SOURCES})
  target_link_libraries (test_ldpc_flexran
    ${FLEXRAN_FEC_LIB_DIR}/source/phy/lib_ldpc_decoder_5gnr/libldpc_decoder_5gnr.a
    ${FLEXRAN_FEC_LIB_DIR}/source/phy/lib_ldpc_encoder_5gnr/libldpc_encoder_5gnr.a
    ${FLEXRAN_FEC_LIB_DIR}/source/phy/lib_common/libcommon.a)
endif()

#set(MOD_SOURCES ../../src/common/modulation.cpp)
#include_directories(../../src/common)
#add_executable(test_mod_ldpc test_mod_ldpc.cpp ${SOURCES} ${MOD_SOURCES})
#target_link_libraries(test_mod_ldpc
#  ${FLEXRAN_FEC_LIB_DIR}/source/phy/lib_ldpc_decoder_5gnr/libldpc_decoder_5gnr.a
#  ${FLEXRAN_FEC_LIB_DIR}/source/phy/lib_common/libcommon.a)
