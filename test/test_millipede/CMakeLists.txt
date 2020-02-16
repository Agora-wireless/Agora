cmake_minimum_required (VERSION 2.8)
cmake_policy(SET CMP0054 NEW)
project (Millipede)

if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7.0)
  set(GCC_COVERAGE_COMPILE_FLAGS "-faligned-new")
endif()

message(STATUS "CMAKE_C_COMPILER: ${CMAKE_C_COMPILER}")
message(STATUS "CMAKE_CXX_COMPILER: ${CMAKE_C_COMPILER}")

set (SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../..)

if (${CMAKE_C_COMPILER_ID} STREQUAL "GNU")
  set (CMAKE_C_FLAGS "-std=c99 -Wall -g -O3 -mavx2 -march=broadwell")
  set (CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -O3 -mavx2 -mavx -march=broadwell  -m64")
  set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
  set (MKL_LIBS -lmkl_rt -lmkl_intel_ilp64 -lmkl_sequential -lmkl_core -lm -ldl)
elseif (${CMAKE_C_COMPILER_ID} STREQUAL "Intel")
  include (${SOURCE_DIR}/cmake/intel-compile-options.cmake)
  set (MKL_LIBS  -lmkl_rt -lpthread -lm -ldl)
  set (CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -O3 -mavx2 -mavx -march=broadwell")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")
  set (CMAKE_CXX_STANDARD 14)
else ()
  set (CMAKE_CXX_STANDARD 11)
endif()

message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

message(STATUS "CURRENT DIRECTORY: ${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DPROJECT_DIRECTORY=${SOURCE_DIR})
add_definitions(-DWRITE_DEMUL)
add_definitions(-DWRITE_IFFT)


find_package(Armadillo)

SET(USE_DPDK False CACHE STRING "USE_DPDK defaulting to 'False'")
SET(USE_ARGOS False CACHE STRING "USE_ARGOS defaulting to 'False'")
SET(USE_LDPC False CACHE STRING "USE_LDPC defaulting to 'False'")

message(STATUS "USE_DPDK: ${USE_DPDK}")
message(STATUS "USE_ARGOS: ${USE_ARGOS}")
message(STATUS "USE_LDPC: ${USE_LDPC}")


set (TXRX_SOURCES ${SOURCE_DIR}/src/millipede/txrx/txrx.cpp)


# Intel MKL
set(BLA_VENDOR Intel10_64lp)
find_package(BLAS)

include_directories(
	${SOURCE_DIR}/src/common/
  ${SOURCE_DIR}/src/millipede/
  ${SOURCE_DIR}/src/millipede/txrx/
  ${SOURCE_DIR}/src/client/
	)

set(COMMON_SOURCES ${SOURCE_DIR}/src/common/config.cpp
                   ${SOURCE_DIR}/src/common/utils.cpp
                   ${SOURCE_DIR}/src/common/comms-lib.cpp
                   ${SOURCE_DIR}/src/common/signalHandler.cpp
                   ${SOURCE_DIR}/src/common/modulation.cpp
                   ${SOURCE_DIR}/src/common/net.cpp)

set(MILLIPEDE_SOURCES 
       ${SOURCE_DIR}/src/millipede/millipede.cpp
       ${SOURCE_DIR}/src/millipede/stats.cpp 
       ${SOURCE_DIR}/src/millipede/dofft.cpp
       ${SOURCE_DIR}/src/millipede/dozf.cpp
       ${SOURCE_DIR}/src/millipede/dodemul.cpp
       ${SOURCE_DIR}/src/millipede/doprecode.cpp
       ${SOURCE_DIR}/src/millipede/reciprocity.cpp)

add_library(comp MODULE 
       ${MILLIPEDE_SOURCES}
       ${TXRX_SOURCES}
       ${COMMON_SOURCES}
       ${ARGOS_SOURCES})

target_link_libraries(comp -lpthread -lmufft -larmadillo ${MKL_LIBS} ${DPDK_LIB} ${SOAPY_LIB} ${PYTHON_LIB})

# LDPC support
if (${USE_LDPC})
  add_definitions(-DUSE_LDPC)
  # set(SDK_DIR /home/argos/Jian/FlexRAN-FEC-SDK-19-04/FlexRAN-FEC-SDK-19-04/sdk/)
  set(SDK_DIR /home/argos/Jian/5gnr_ldpc/intel_library)
  set(LIB_DIR ${SDK_DIR}/build-avx2-icc)
  #set(LDPC_LIBS ${LIB_DIR}/source/phy/lib_ldpc_decoder_5gnr/libldpc_decoder_5gnr.a
  #              ${LIB_DIR}/test/phy/common/utility.a
  #              ${LIB_DIR}/source/phy/lib_common/libcommon.a)

  set(LDPC_LIBS  ${SDK_DIR}/decoder/src/lib_ldpc_decoder_5gnr/libldpc_decoder_5gnr.a
                 ${SDK_DIR}/decoder/src/common/libutility.a
                 ${SDK_DIR}/decoder/src/lib_common/libcommon.a)

  #include_directories(
  #        ${SDK_DIR}/test/phy/common
  #        ${SDK_DIR}/source/phy/lib_ldpc_decoder_5gnr
  #        ${SDK_DIR}/source/phy/lib_common
  #        src/encoder/
  #)

  include_directories(${SDK_DIR}/decoder/src/common
                    ${SDK_DIR}/decoder/src/lib_ldpc_decoder_5gnr
                    ${SDK_DIR}/decoder/src/lib_common
                    ${SOURCE_DIR}/src/encoder)

  #set(HEADER_FILES ${SDK_DIR}/source/phy/lib_ldpc_decoder_5gnr/phy_ldpc_decoder_5gnr.h)
  set(HEADER_FILES ${SDK_DIR}/decoder/src/lib_ldpc_decoder_5gnr/phy_ldpc_decoder_5gnr.h)

  set(LDPC_SOURCES  ${SOURCE_DIR}/src/encoder/cyclic_shift.cpp
                    ${SOURCE_DIR}/src/encoder/encoder.cpp
                    ${SOURCE_DIR}/src/encoder/iobuffer.cpp
                    ${SOURCE_DIR}/src/millipede/docoding.cpp)

endif()

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

set(SOURCES main.cpp 
            ${MILLIPEDE_SOURCES}
	    ${TXRX_SOURCES}
            ${COMMON_SOURCES}
            ${ARGOS_SOURCES}
            ${LDPC_SOURCES})
			
add_executable(millipede ${SOURCES} ${HEADER_FILES})
target_link_libraries(millipede -lpthread /usr/local/lib/libmufft.a -larmadillo ${MKL_LIBS} ${DPDK_LIB} ${SOAPY_LIB} ${PYTHON_LIB} ${LDPC_LIBS})

set(SENDER_SOURCES ${SOURCE_DIR}/simulator/sender_cli.cpp
                   ${SOURCE_DIR}/simulator/sender.cpp 
                   ${COMMON_SOURCES})

add_executable(sender ${SENDER_SOURCES})
target_link_libraries(sender -lpthread -lmufft)


set(SIMULATOR_SOURCES ${SOURCE_DIR}/simulator/main.cpp
                      ${SOURCE_DIR}/simulator/simulator.cpp
                      ${SOURCE_DIR}/simulator/sender.cpp
                      ${SOURCE_DIR}/simulator/receiver.cpp 
                      ${COMMON_SOURCES})

add_executable(sim ${SIMULATOR_SOURCES})
target_link_libraries(sim -lpthread -lmufft)



