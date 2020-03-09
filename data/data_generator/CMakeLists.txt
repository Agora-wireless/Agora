cmake_minimum_required (VERSION 2.8)
cmake_policy(SET CMP0054 NEW)
project (Millipede)

if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7.0)
  set(GCC_COVERAGE_COMPILE_FLAGS "-faligned-new")
endif()

message(STATUS "CMAKE_C_COMPILER: ${CMAKE_C_COMPILER}")
message(STATUS "CMAKE_CXX_COMPILER: ${CMAKE_C_COMPILER}")

set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../..)
if(NOT ${CMAKE_C_COMPILER_ID} STREQUAL "Intel")
  message(STATUS "CMAKE_C_COMPILER_ID: ${CMAKE_C_COMPILER_ID}")
  message(FATAL_ERROR "Intel compiler is required for LDPC library!" )
endif()

# set compiler options
include(${SOURCE_DIR}/cmake/intel-compile-options.cmake)
set(MKL_LIBS -lpthread -lm -ldl)
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -mavx2 -mavx -march=broadwell -mkl=sequential")
option(DEBUG "Enable debugging" OFF)
if(NOT DEBUG)
  message(STATUS "Debugging is disabled")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
else()
  message(STATUS "Debugging is enabled. Performance will be low")
endif()

message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

# Intel MKL
set(BLA_VENDOR Intel10_64lp)
find_package(BLAS)

message(STATUS "CURRENT DIRECTORY: ${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DPROJECT_DIRECTORY=${SOURCE_DIR})


include_directories(${SOURCE_DIR}/src/common/)
set(COMMON_SOURCES 
  ${SOURCE_DIR}/src/common/config.cpp
  ${SOURCE_DIR}/src/common/utils.cpp
  ${SOURCE_DIR}/src/common/comms-lib.cpp
  ${SOURCE_DIR}/src/common/modulation.cpp)

add_definitions(-DUSE_LDPC)
set(SDK_DIR /home/argos/Jian/FlexRAN-FEC-SDK-19-04/FlexRAN-FEC-SDK-19-04/sdk/)
set(LIB_DIR ${SDK_DIR}/build-avx2-icc)
set(LDPC_LIBS ${LIB_DIR}/source/phy/lib_ldpc_decoder_5gnr/libldpc_decoder_5gnr.a
              ${LIB_DIR}/test/phy/common/utility.a
              ${LIB_DIR}/source/phy/lib_common/libcommon.a)

include_directories(
  ${SDK_DIR}/decoder/src/common
  ${SDK_DIR}/decoder/src/lib_ldpc_decoder_5gnr
  ${SDK_DIR}/decoder/src/lib_common
  ${SOURCE_DIR}/src/encoder)

set(HEADER_FILES ${SDK_DIR}/source/phy/lib_ldpc_decoder_5gnr/phy_ldpc_decoder_5gnr.h)

set(LDPC_SOURCES  
  ${SOURCE_DIR}/src/encoder/cyclic_shift.cpp
  ${SOURCE_DIR}/src/encoder/encoder.cpp
  ${SOURCE_DIR}/src/encoder/iobuffer.cpp)

add_executable(data_generator data_generator.cpp ${LDPC_SOURCES} ${COMMON_SOURCES} ${HEADER_FILES})
target_link_libraries(data_generator
  ${LDPC_LIBS} ${MKL_LIBS})

