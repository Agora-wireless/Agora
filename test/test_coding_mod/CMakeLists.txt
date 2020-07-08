
# Cmake 2.8.12 needed for "add_compile_options" command
cmake_minimum_required(VERSION 2.8.12)

find_program(ICPC_EXISTS "icpc")
if(NOT ICPC_EXISTS)
  message(FATAL_ERROR "Intel C++ compiler not found. Exiting.")
endif()

set(CMAKE_CXX_COMPILER icpc)  # Intel compiler required
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")

# Set preprocessor definitions for target ISA
if(${ISA_SSE4_2})
  add_definitions("-D_BBLIB_SSE4_2_")
elseif(${ISA_AVX2})
  add_definitions("-D_BBLIB_SSE4_2_ -D_BBLIB_AVX2_")
elseif(${ISA_AVX512})
  add_definitions("-D_BBLIB_SSE4_2_ -D_BBLIB_AVX2_ -D_BBLIB_AVX512_")
endif()

# Set ISA specific compile flags (do not get passed to linker)
if(${ISA_SSE4_2})
  # Compile flags / defintions for SSE4_2 (Linux)
  add_compile_options("-xSSE4.2")
elseif(${ISA_AVX2})
  # Compile flags / defintions for AVX2 (Linux)
 add_compile_options("-xCORE-AVX2")

elseif(${ISA_AVX512})
  # Compile flags / defintions for AVX512 (Linux)
  add_compile_options("-xCORE-AVX512")
endif()

if(RTE_SDK)
  add_definitions("-D_BBLIB_DPDK_")
endif()

set(FLEXRAN_FEC_SDK_DIR "/opt/FlexRAN-FEC-SDK-19-04/sdk")

# FlexRAN FEC libraries can be in one of two directories depending on arch
if (EXISTS ${FLEXRAN_FEC_SDK_DIR}/build-avx512-icc)
  set(FLEXRAN_FEC_LIB_DIR ${FLEXRAN_FEC_SDK_DIR}/build-avx512-icc)
elseif (EXISTS ${FLEXRAN_FEC_SDK_DIR}/build-avx2-icc)
  set(FLEXRAN_FEC_LIB_DIR ${FLEXRAN_FEC_SDK_DIR}/build-avx2-icc)
else()
  message(FATAL_ERROR "FlexRAN FEC libraries required but not found")
endif()

include_directories(
  /opt/intel/mkl/include
  ${FLEXRAN_FEC_SDK_DIR}/test/phy/common
  ${FLEXRAN_FEC_SDK_DIR}/source/phy/lib_ldpc_decoder_5gnr
  ${FLEXRAN_FEC_SDK_DIR}/source/phy/lib_common
  ../../src
  ../../src/encoder)

set(SOURCES 
  ../../src/encoder/cyclic_shift.cpp
  ../../src/encoder/encoder.cpp
  ../../src/encoder/iobuffer.cpp)

add_executable(test_ldpc test_ldpc.cpp ${SOURCES})
target_link_libraries (test_ldpc
  ${FLEXRAN_FEC_LIB_DIR}/source/phy/lib_ldpc_decoder_5gnr/libldpc_decoder_5gnr.a
  ${FLEXRAN_FEC_LIB_DIR}/source/phy/lib_common/libcommon.a)
add_definitions(-DUSE_LDPC)

#set(MOD_SOURCES ../../src/common/modulation.cpp)
#include_directories(../../src/common)
#add_executable(test_mod_ldpc test_mod_ldpc.cpp ${SOURCES} ${MOD_SOURCES})
#target_link_libraries(test_mod_ldpc
#  ${FLEXRAN_FEC_LIB_DIR}/source/phy/lib_ldpc_decoder_5gnr/libldpc_decoder_5gnr.a
#  ${FLEXRAN_FEC_LIB_DIR}/source/phy/lib_common/libcommon.a)
