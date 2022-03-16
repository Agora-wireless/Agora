cmake_minimum_required(VERSION 2.8.8)
include(CheckCSourceRuns)
cmake_policy(SET CMP0054 NEW)
project(Agora)

if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7.0)
  set(GCC_COVERAGE_COMPILE_FLAGS "-faligned-new")
endif()

set (PROJECT_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)

option(FORCE_BUILD_PATH "Hardcode the build directory path to be 'Agora/build/'" ON)
if(FORCE_BUILD_PATH)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build)
endif()

# New versions of GCC enable position-independent executables by default. We
# don't want to compile FlexRAN libraries with -fPIC because it reduces
# performance by around 40%.
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -no-pie")

if(${CMAKE_C_COMPILER_ID} STREQUAL "GNU")
  message(STATUS "Using GNU compiler, compiler ID ${CMAKE_C_COMPILER_ID}")
  set(CMAKE_C_FLAGS "-std=gnu11 -Wall -g -march=native -m64")
  set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -march=native -m64")
  set(MKL_LIBS -Wl,--no-as-needed -lmkl_intel_lp64 -lmkl_sequential -lmkl_core -lpthread -lm -ldl)
elseif(${CMAKE_C_COMPILER_ID} STREQUAL "Intel")
  message(STATUS "Using Intel compiler, compiler ID ${CMAKE_C_COMPILER_ID}")
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/intel-compile-options.cmake)
  set(MKL_LIBS -lpthread -lm -ldl)
  set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -march=native -mkl=sequential")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")
  message(STATUS "Using Clang compiler, compiler ID ${CMAKE_C_COMPILER_ID}")
  set(CMAKE_CXX_STANDARD 14)
else()
  set(CMAKE_CXX_STANDARD 11)
endif()

option(DEBUG "Enable debugging" OFF)
if(NOT DEBUG)
  message(STATUS "Debugging is disabled")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -DNDEBUG -Wno-unused-result")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG -Wno-unused-result")
else()
  message(STATUS "Debugging is enabled. Performance will be low")
endif()

message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

message(STATUS "CURRENT DIRECTORY: ${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DPROJECT_DIRECTORY=${CMAKE_CURRENT_SOURCE_DIR})

find_package(Armadillo PATHS ${PROJECT_ROOT_DIR}/armadillo REQUIRED)
include_directories(SYSTEM ${PROJECT_ROOT_DIR}/armadillo/include)
find_package(nlohmann_json PATHS ${PROJECT_ROOT_DIR}/json REQUIRED)
include_directories(SYSTEM ${PROJECT_ROOT_DIR}/json/include)

set(USE_DPDK True CACHE STRING "USE_DPDK defaulting to 'False'")
set(LOG_LEVEL "warn" CACHE STRING "Console logging level (none/error/warn/info/frame/subframe/trace)") 
set(USE_MLX_NIC True CACHE STRING "USE_MLX_NIC defaulting to 'True'")
set(USE_AVX2_ENCODER False CACHE STRING "Use Agora's AVX2 encoder instead of FlexRAN's AVX512 encoder")

message(STATUS "USE_ARGOS: ${USE_ARGOS}")
message(STATUS "ENABLE_MAC: ${ENABLE_MAC}")

set(FLEXRAN_FEC_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/FlexRAN-FEC-SDK-19-04/sdk)

# Determine if the current machine supports AVX-512
CHECK_C_SOURCE_RUNS("int main() { asm volatile(\"vmovdqu64 %zmm0, %zmm1\"); return 0; }" ISA_AVX512)
if (ISA_AVX512)
  message(STATUS "Processor supports AVX-512")
  add_definitions(-DISA_AVX512)
  set(FLEXRAN_FEC_LIB_DIR ${FLEXRAN_FEC_SDK_DIR}/build-avx512-icc)
  # Leave USE_AVX2_ENCODER unmodified
else()
  message(STATUS "Processor does not support AVX-512")
  set(FLEXRAN_FEC_LIB_DIR ${FLEXRAN_FEC_SDK_DIR}/build-avx2-icc)
  set(USE_AVX2_ENCODER True)
endif()

if(USE_AVX2_ENCODER)
  message(STATUS "Using Agora's (i.e., not FlexRAN's) AVX2 encoder")
  add_definitions(-DUSE_AVX2_ENCODER)
else()
  message(STATUS "Using FlexRAN's (i.e., not Agora's) AVX512 encoder")
endif()

if(${USE_DPDK})
  message(STATUS "DPDK is enabled for Agora")

  if(NOT DEFINED ENV{RTE_SDK})
    message(FATAL_ERROR "Please define RTE_SDK")
  endif()

  set(DPDK_INSTALL_DIR "$ENV{RTE_SDK}/build/install/usr/local")

  # DPDK installs libraries into local/lib on Mariner, but
  # local/lib/x86_64-linux-gnu on Ubuntu
  set(DPDK_LIB_DIR "${DPDK_INSTALL_DIR}/lib/x86_64-linux-gnu")
  if(NOT EXISTS ${DPDK_LIB_DIR})
    set(DPDK_LIB_DIR "${DPDK_INSTALL_DIR}/lib")
    if(NOT EXISTS ${DPDK_LIB_DIR})
      message(FATAL_ERROR "${DPDK_LIB_DIR} not found. Did you run `ninja install`?")
    endif()
  endif()

  link_directories(${DPDK_LIB_DIR})
  include_directories(SYSTEM ${DPDK_INSTALL_DIR}/include)

  # We use some hacky command-line and CMake magic to construct DPDK library list
  execute_process(
    COMMAND bash -c "PKG_CONFIG_PATH=${DPDK_LIB_DIR}/pkgconfig pkg-config --static --libs-only-l libdpdk"
    OUTPUT_VARIABLE DPDK_PKGCONFIG_OUT)
  string(STRIP ${DPDK_PKGCONFIG_OUT} DPDK_PKGCONFIG_OUT) # Remove newline from pkg-config output

  # Delete some DPDK libraries from the list:
  # * Baseband-related libraries, since they require FlexRAN
  # * DPDK's crypto libraries require an extra OpenSSL dependency.
  # * DPDK 20.11's pkgconfig output contains multiple occurences of -lpthread,
  #   which can cause build errors. So we delete -lpthread, and re-add it later.
  set(DELETE_DPDK_LIBS
    -librte_bbdev.a -l:librte_baseband_null.a -l:librte_baseband_turbo_sw.a
    -l:librte_baseband_fpga_lte_fec.a -l:librte_baseband_fpga_5gnr_fec.a
    -l:librte_baseband_acc100.a -lrte_bbdev
    -l:librte_crypto_openssl.a -l:librte_crypto_ccp.a
    -lpthread)
  foreach(l IN LISTS DELETE_DPDK_LIBS)
    string(REPLACE ${l} "" DPDK_PKGCONFIG_OUT ${DPDK_PKGCONFIG_OUT})
  endforeach()

  set(DPDK_LIBRARIES
  "-Wl,--whole-archive -Wl,--as-needed -L${DPDK_LIB_DIR} ${DPDK_PKGCONFIG_OUT} -lpthread -Wl,--no-whole-archive")

  # set(DPDK_LIBRARIES ${DPDK_LIBRARIES} -Wl,--whole-archive -L${DPDK_LIB_DIR} -lnuma -Wl,--no-whole-archive dl)
endif()

message(STATUS "Use DPDK for agora: ${USE_DPDK}")

message(STATUS "CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")
# find_package(PythonLibs REQUIRED)
# message(STATUS "PYTHON_LIBRARIES: ${PYTHON_LIBRARIES}")
# include_directories(${PYTHON_INCLUDE_DIRS})
# set(PYTHON_LIB ${PYTHON_LIBRARIES}) 
add_definitions(-DTHREADED_INIT)

# Intel MKL
set(BLA_VENDOR Intel10_64lp)
find_package(BLAS)

# Console logging level
if(LOG_LEVEL STREQUAL "none")
  message(STATUS "Logging level = none.")
  add_definitions(-DMLPD_LOG_LEVEL=0)
elseif(LOG_LEVEL STREQUAL "error")
  message(STATUS "Logging level = error.")
  add_definitions(-DMLPD_LOG_LEVEL=1)
elseif(LOG_LEVEL STREQUAL "warn")
  message(STATUS "Logging level = warn.")
  add_definitions(-DMLPD_LOG_LEVEL=2)
elseif(LOG_LEVEL STREQUAL "info")
  message(STATUS "Logging level = info.")
  add_definitions(-DMLPD_LOG_LEVEL=3)
elseif(LOG_LEVEL STREQUAL "frame")
  message(STATUS "Logging level = frame. Warning: Performance will be low.")
  add_definitions(-DMLPD_LOG_LEVEL=4)
elseif(LOG_LEVEL STREQUAL "subframe")
  message(STATUS "Logging level = subframe. Warning: Performance will be low.")
  add_definitions(-DMLPD_LOG_LEVEL=5)
elseif(LOG_LEVEL STREQUAL "trace")
  message(STATUS "Logging level = trace. Warning: Performance will be low.")
  add_definitions(-DMLPD_LOG_LEVEL=6)
else()
  message(STATUS "No logging level specified. Using warning level.")
  add_definitions(-DMLPD_LOG_LEVEL=2)
endif()

include_directories(
	src/common/
	src/mac/
  src/agora/
  src/agora/txrx/)

include_directories(SYSTEM src/third_party)

set(COMMON_SOURCES
  src/common/config.cpp
  src/common/utils.cpp
  src/common/comms-lib.cpp
  src/common/comms-lib-avx.cpp
  src/common/signalHandler.cpp
  src/common/modulation.cpp
  src/common/modulation_srslte.cpp
  src/common/net.cpp
  src/common/crc.cpp
  src/common/shared_counters.cpp
  src/common/profiler.cpp
  src/encoder/cyclic_shift.cpp
  src/encoder/encoder.cpp
  src/encoder/iobuffer.cpp)
add_library(common_sources_lib OBJECT ${COMMON_SOURCES})

set(AGORA_SOURCES 
  src/agora/agora.cpp
  src/agora/dofft.cpp
  src/agora/dyzf.cpp
  src/agora/dydemul.cpp
  src/agora/dycoding.cpp
  src/agora/dysubcarrier.cpp
  src/agora/dyprecode.cpp
  src/mac/mac_thread.cpp
  src/agora/diagnosis.cpp)

if(${USE_DPDK})
  add_definitions(-DUSE_DPDK)
  set(AGORA_SOURCES ${AGORA_SOURCES} 
    src/agora/txrx/txrx_DPDK.cpp
    src/common/dpdk_transport.cpp)
else()
  message(FATAL_ERROR "DPDK should be enabled")
endif()
add_library(agora_sources_lib OBJECT ${AGORA_SOURCES})

include_directories(
  ${FLEXRAN_FEC_SDK_DIR}/source/phy/lib_ldpc_decoder_5gnr
  ${FLEXRAN_FEC_SDK_DIR}/source/phy/lib_ldpc_encoder_5gnr
  ${FLEXRAN_FEC_SDK_DIR}/source/phy/lib_common
  ${CMAKE_CURRENT_SOURCE_DIR}/src/encoder)

set(FLEXRAN_LDPC_LIBS
  ${FLEXRAN_FEC_LIB_DIR}/source/phy/lib_ldpc_encoder_5gnr/libldpc_encoder_5gnr.a
  ${FLEXRAN_FEC_LIB_DIR}/source/phy/lib_ldpc_decoder_5gnr/libldpc_decoder_5gnr.a
  ${FLEXRAN_FEC_LIB_DIR}/source/phy/lib_common/libcommon.a)

# set(COMMON_LIBS armadillo ${MKL_LIBS} ${DPDK_LIBRARIES}
#   ${PYTHON_LIB} ${FLEXRAN_LDPC_LIBS} util gflags gtest)
set(COMMON_LIBS armadillo ${MKL_LIBS} ${DPDK_LIBRARIES}
   ${FLEXRAN_LDPC_LIBS} util nlohmann_json::nlohmann_json)

# TODO: The main agora executable is performance-critical, so we need to
# test if compiling against precompiled objects instead of compiling directly
# against sources reduces performance.
add_executable(agora
  src/agora/main.cpp
  $<TARGET_OBJECTS:agora_sources_lib>
  $<TARGET_OBJECTS:common_sources_lib>)
target_link_libraries(agora ${COMMON_LIBS})

# add_executable(sim
#   simulator/main.cpp
#   simulator/simulator.cpp
#   simulator/dynamic_sender.cpp
#   simulator/receiver.cpp
#   src/common/dpdk_transport.cpp
#   $<TARGET_OBJECTS:common_sources_lib>)
# target_link_libraries(sim ${COMMON_LIBS})

# End-to-end test
add_executable(test_agora
  test/test_agora/main.cpp
  $<TARGET_OBJECTS:agora_sources_lib>
  $<TARGET_OBJECTS:common_sources_lib>)
target_link_libraries(test_agora ${COMMON_LIBS})

add_executable(test_ldpc
  test/compute_kernels/ldpc/test_ldpc.cpp
  $<TARGET_OBJECTS:common_sources_lib>)
target_link_libraries(test_ldpc ${COMMON_LIBS})
add_test(NAME test_ldpc COMMAND test_ldpc)

add_executable(test_clock_sender
  tool/test_clock_sender.cpp
  src/common/dpdk_transport.cpp
  $<TARGET_OBJECTS:common_sources_lib>)
target_link_libraries(test_clock_sender ${COMMON_LIBS})

add_executable(test_clock_receiver
  tool/test_clock_receiver.cpp
  src/common/dpdk_transport.cpp
  $<TARGET_OBJECTS:common_sources_lib>)
target_link_libraries(test_clock_receiver ${COMMON_LIBS})

add_executable(control_generator
  data/control_generator/control_generator.cpp
  $<TARGET_OBJECTS:common_sources_lib>)
target_link_libraries(control_generator ${COMMON_LIBS})

add_executable(dynamic_generator
  ${CMAKE_CURRENT_SOURCE_DIR}/data/dynamic_generator/dynamic_generator.cpp
  $<TARGET_OBJECTS:common_sources_lib>)
target_link_libraries(dynamic_generator ${COMMON_LIBS})
target_compile_definitions(dynamic_generator PRIVATE GENERATE_DATA)

add_executable(dynamic_sender
  simulator/dynamic_sender_cli.cpp
  simulator/dynamic_sender.cpp
  src/common/dpdk_transport.cpp
  $<TARGET_OBJECTS:common_sources_lib>)
target_link_libraries(dynamic_sender ${COMMON_LIBS})

add_executable(test_decode_spec
  test/compute_kernels/ldpc/test_decode_spec.cpp
  $<TARGET_OBJECTS:common_sources_lib>)
target_link_libraries(test_decode_spec ${COMMON_LIBS})

add_executable(benchmark
  test/benchmark/benchmark.cpp
  $<TARGET_OBJECTS:agora_sources_lib>
  $<TARGET_OBJECTS:common_sources_lib>)
target_link_libraries(benchmark ${COMMON_LIBS})