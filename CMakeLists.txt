cmake_minimum_required(VERSION 2.8.8)
include(CheckCSourceRuns)
cmake_policy(SET CMP0054 NEW)
project(Agora)

if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7.0)
  set(GCC_COVERAGE_COMPILE_FLAGS "-faligned-new")
endif()

set(SOURCE_DIR ".")

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
  set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -g -march=native -m64")
  set(MKL_LIBS -Wl,--no-as-needed -lmkl_intel_lp64 -lmkl_sequential -lmkl_core -lpthread -lm -ldl)
elseif(${CMAKE_C_COMPILER_ID} STREQUAL "Intel")
  message(STATUS "Using Intel compiler, compiler ID ${CMAKE_C_COMPILER_ID}")
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/intel-compile-options.cmake)
  set(MKL_LIBS -lpthread -lm -ldl)
  set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -g -march=native -mkl=sequential")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")
  message(STATUS "Using Clang compiler, compiler ID ${CMAKE_C_COMPILER_ID}")
  set(CMAKE_CXX_STANDARD 17)
else()
  set(CMAKE_CXX_STANDARD 17)
endif()

# Unit tests
enable_testing()
find_package(GTest REQUIRED)

option(DEBUG "Enable debugging" OFF)
if(NOT DEBUG)
  message(STATUS "Debugging is disabled")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -DNDEBUG")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
else()
  message(STATUS "Verbose Makefile")
  set( CMAKE_VERBOSE_MAKEFILE on )
  message(STATUS "Debugging is enabled. Performance will be low")
  set(CMAKE_BUILD_TYPE Debug)
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wshadow -ggdb3 -fstack-protector-all -fsanitize=undefined -fsanitize=address -fsanitize=null")
  #-fsanitize=thread- fstack-protector-all -fsanitize=address -fstack-check
  #-fsanitize=address -fsanitize=pointer-compare
  #-fsanitize=address -fsanitize=pointer-subtract
  #-fsanitize=leak -fsanitize=undefined -fsanitize=null
endif()

#if(CMAKE_BUILD_TYPE MATCHES Debug)

message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

message(STATUS "CURRENT DIRECTORY: ${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DPROJECT_DIRECTORY=${SOURCE_DIR})

find_package(Armadillo)

set(USE_DPDK False CACHE STRING "USE_DPDK defaulting to 'False'")
set(USE_ARGOS False CACHE STRING "USE_ARGOS defaulting to 'False'")
set(ENABLE_MAC False CACHE STRING "ENABLE_MAC defaulting to 'False'")
set(LOG_LEVEL "info" CACHE STRING "Console logging level (none/error/warn/info/frame/subframe/trace)") 
set(USE_MLX_NIC True CACHE STRING "USE_MLX_NIC defaulting to 'True'")
set(USE_AVX2_ENCODER False CACHE STRING "Use Agora's AVX2 encoder instead of FlexRAN's AVX512 encoder")
# TODO: add SoapyUHD check
set(USE_UHD False CACHE STRING "USE_UHD defaulting to 'False'")

message(STATUS "USE_UHD: ${USE_UHD}")
message(STATUS "USE_ARGOS: ${USE_ARGOS}")
message(STATUS "ENABLE_MAC: ${ENABLE_MAC}")

set(FLEXRAN_FEC_SDK_DIR /opt/FlexRAN-FEC-SDK-19-04/sdk)

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

# DPDK
if(${USE_DPDK})
  message(STATUS "DPDK is enabled for Agora")

  find_library(DPDK_LIB dpdk)
  message(STATUS "DPDK_LIB: ${DPDK_LIB}")
  if(NOT DPDK_LIB)
    message(FATAL_ERROR "DPDK library not found")
  endif()

  if(${USE_MLX_NIC})
    set(MLX_LIBRARIES -libverbs -lmlx5)
  endif()
  set(DPDK_LIBRARIES ${DPDK_LIBRARIES} -Wl,--whole-archive dpdk ${MLX_LIBRARIES} -Wl,--no-whole-archive dl)

  # DPDK include directory. Locating rte_config.h does not work on some systems.
  # Example: it may be kept in /usr/include/x86_64-linux-gnu/, and symlinked
  # from the real DPDK include directory (/usr/include/dpdk/).
  find_path(DPDK_INCLUDE_DIR NAMES rte_ethdev.h PATH_SUFFIXES dpdk)
  if (DPDK_INCLUDE_DIR)
    message(STATUS "DPDK include directory = ${DPDK_INCLUDE_DIR}")
  else()
    message(FATAL_ERROR "DPDK include directory not found")
  endif()
  include_directories(SYSTEM ${DPDK_INCLUDE_DIR})

  add_definitions(-DUSE_DPDK)
endif()

message(STATUS "Use DPDK for agora: ${USE_DPDK}")

# MAC
if(${ENABLE_MAC})
  add_definitions(-DENABLE_MAC)
endif()

set(MAC_CLIENT_SOURCES
  src/mac/ul_mac_sender.cc
  src/mac/mac_client.cc)
set(MAC_BS_SOURCES
  src/mac/ul_mac_receiver.cc
  src/mac/mac_basestation.cc)

# Argos support
if(${USE_ARGOS})
  add_definitions(-DUSE_ARGOS)
endif()

if(${USE_UHD})
  message(STATUS "Enabled USE_UHD: ${USE_UHD}")
  add_definitions(-DUSE_UHD)
endif()

#External Libraries
find_package(HDF5 1.10 REQUIRED)
if (NOT HDF5_FOUND)
    message(FATAL_ERROR "HDF5 development files not found")
    return()
endif ()

find_package(SoapySDR "0.7" CONFIG)
if(NOT SoapySDR_FOUND)
  message(FATAL_ERROR "SoapySDR development files not found")
endif()
message(STATUS "CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")
find_package(PythonLibs REQUIRED)
message(STATUS "PYTHON_LIBRARIES: ${PYTHON_LIBRARIES}")
message(STATUS "SoapySDR_INCLUDE_DIRS: ${SoapySDR_INCLUDE_DIRS}")
message(STATUS "SoapySDR_LIBRARIES: ${SoapySDR_LIBRARIES}")
message(STATUS "HDF5_INCLUDE_DIRS: ${HDF5_INCLUDE_DIRS}")
message(STATUS "HDF5_LIBRARIES: ${HDF5_LIBRARIES}")
include_directories(${PYTHON_INCLUDE_DIRS} ${SoapySDR_INCLUDE_DIRS} ${HDF5_INCLUDE_DIRS})
set(SOAPY_LIB ${SoapySDR_LIBRARIES})
set(PYTHON_LIB ${PYTHON_LIBRARIES}) 
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
  src/agora/txrx/
  src/client/
  /usr/include/mkl/
  src/data_generator)

include_directories(SYSTEM src/third_party src/third_party/nlohmann/single_include)

set(COMMON_SOURCES
  src/agora/stats.cc
  src/agora/phy_stats.cc
  src/common/framestats.cc
  src/agora/doencode.cc
  src/common/config.cc
  src/common/utils.cc
  src/common/comms-lib.cc
  src/common/comms-lib-avx.cc
  src/common/signal_handler.cc
  src/common/modulation.cc
  src/common/modulation_srslte.cc
  src/common/net.cc
  src/common/crc.cc
  src/common/memory_manage.cc
  src/common/scrambler.cc
  src/common/recorder.cc
  src/common/recorder_thread.cc
  src/common/recorder_worker.cc
  src/encoder/cyclic_shift.cc
  src/encoder/encoder.cc
  src/encoder/iobuffer.cc
  )
add_library(common_sources_lib OBJECT ${COMMON_SOURCES})

set(AGORA_SOURCES 
  src/agora/agora.cc
  src/agora/dofft.cc
  src/agora/dozf.cc
  src/agora/dodemul.cc
  src/agora/doprecode.cc
  src/agora/dodecode.cc
  src/agora/radio_lib.cc
  src/agora/radio_calibrate.cc
  src/mac/mac_thread.cc)

if(${USE_DPDK})
  add_definitions(-DUSE_DPDK)
  set(AGORA_SOURCES ${AGORA_SOURCES} 
    src/agora/txrx/txrx_DPDK.cc
    src/common/dpdk_transport.cc)
else()
  set(AGORA_SOURCES ${AGORA_SOURCES} 
    src/agora/txrx/txrx.cc
    src/agora/txrx/txrx_argos.cc
    src/agora/txrx/txrx_usrp.cc)
endif()
add_library(agora_sources_lib OBJECT ${AGORA_SOURCES})

set(CLIENT_SOURCES
  src/client/doifft_client.cc
  src/client/dodecode_client.cc
  src/client/client_radio.cc
  src/client/ue_worker.cc
  src/client/phy-ue.cc
  src/client/txrx_client.cc
  src/mac/mac_thread.cc)
add_library(client_sources_lib OBJECT ${CLIENT_SOURCES})
  
include_directories(
  ${FLEXRAN_FEC_SDK_DIR}/source/phy/lib_ldpc_decoder_5gnr
  ${FLEXRAN_FEC_SDK_DIR}/source/phy/lib_ldpc_encoder_5gnr
  ${FLEXRAN_FEC_SDK_DIR}/source/phy/lib_common
  ${SOURCE_DIR}/src/encoder)

set(FLEXRAN_LDPC_LIBS
  ${FLEXRAN_FEC_LIB_DIR}/source/phy/lib_ldpc_encoder_5gnr/libldpc_encoder_5gnr.a
  ${FLEXRAN_FEC_LIB_DIR}/source/phy/lib_ldpc_decoder_5gnr/libldpc_decoder_5gnr.a
  ${FLEXRAN_FEC_LIB_DIR}/source/phy/lib_common/libcommon.a)

set(COMMON_LIBS armadillo -lnuma -lhdf5_cpp ${HDF5_LIBRARIES} ${MKL_LIBS} ${DPDK_LIBRARIES} ${SOAPY_LIB}
  ${PYTHON_LIB} ${FLEXRAN_LDPC_LIBS} util gflags gtest)

# TODO: The main agora executable is performance-critical, so we need to
# test if compiling against precompiled objects instead of compiling directly
# against sources reduces performance.
add_executable(agora
  src/agora/main.cc
  $<TARGET_OBJECTS:agora_sources_lib>
  $<TARGET_OBJECTS:common_sources_lib>)
target_link_libraries(agora ${COMMON_LIBS})

add_executable(data_generator
  ${CMAKE_CURRENT_SOURCE_DIR}/src/data_generator/data_generator_main.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/src/data_generator/data_generator.cc
  $<TARGET_OBJECTS:common_sources_lib>)
target_link_libraries(data_generator ${COMMON_LIBS})
target_compile_definitions(data_generator PRIVATE GENERATE_DATA)

add_executable(user
  src/client/user-main.cc
  $<TARGET_OBJECTS:client_sources_lib>
  $<TARGET_OBJECTS:common_sources_lib>)
target_link_libraries(user ${COMMON_LIBS})

add_executable(sender
  simulator/sender_cli.cc
  simulator/sender.cc
  src/common/dpdk_transport.cc
  $<TARGET_OBJECTS:common_sources_lib>)
target_link_libraries(sender ${COMMON_LIBS})

add_executable(sim
  simulator/main.cc
  simulator/simulator.cc
  simulator/sender.cc
  simulator/receiver.cc
  src/common/dpdk_transport.cc
  $<TARGET_OBJECTS:common_sources_lib>)
target_link_libraries(sim ${COMMON_LIBS})

add_executable(chsim
  simulator/chsim_main.cc
  simulator/channel_sim.cc
  simulator/channel.cc
  $<TARGET_OBJECTS:common_sources_lib>)
target_link_libraries(chsim ${COMMON_LIBS})

add_executable(macuser
     ${MAC_CLIENT_SOURCES}
     ${COMMON_SOURCES})
target_link_libraries(macuser ${COMMON_LIBS})

add_executable(macbs
     ${MAC_BS_SOURCES}
     ${COMMON_SOURCES})
target_link_libraries(macbs ${COMMON_LIBS})


# End-to-end test
add_executable(test_agora
  test/test_agora/main.cc
  $<TARGET_OBJECTS:agora_sources_lib>
  $<TARGET_OBJECTS:common_sources_lib>)
target_link_libraries(test_agora ${COMMON_LIBS})


set(LDPC_TESTS test_ldpc test_ldpc_mod test_ldpc_baseband)
foreach(test_name IN LISTS LDPC_TESTS)
  add_executable(${test_name}
    test/compute_kernels/ldpc/${test_name}.cc
    $<TARGET_OBJECTS:common_sources_lib>)
  # Unit tests should pass without memory errors
  set_target_properties(${test_name} PROPERTIES CMAKE_CXX_FLAGS "-fsanitize=address")

  target_link_libraries(${test_name} ${COMMON_LIBS})
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()


# Unit tests
set(UNIT_TESTS test_datatype_conversion test_udp_client_server
  test_concurrent_queue test_zf test_zf_threaded test_demul_threaded 
  test_ptr_grid test_recipcal test_avx512_complex_mul test_scrambler
  test_256qam_demod)

foreach(test_name IN LISTS UNIT_TESTS)
  add_executable(${test_name}
    test/unit_tests/${test_name}.cc
    $<TARGET_OBJECTS:agora_sources_lib>
    $<TARGET_OBJECTS:common_sources_lib>)

  # Unit tests should pass without memory errors
  set_target_properties(${test_name} PROPERTIES CMAKE_CXX_FLAGS "-fsanitize=address")

  target_link_libraries(${test_name} ${COMMON_LIBS})
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()

# if(NOT ${USE_DPDK})
#   # Create shared libraries for Python
#   # DPDK is currently not supported
#    add_library(comp MODULE
#      $<TARGET_OBJECTS:agora_sources_lib>
#      $<TARGET_OBJECTS:common_sources_lib>)
#    target_link_libraries(comp ${COMMON_LIBS})

#   add_library(ue_phy MODULE
#     $<TARGET_OBJECTS:client_sources_lib>
#     $<TARGET_OBJECTS:common_sources_lib>)
#   target_link_libraries(ue_phy ${COMMON_LIBS})
# else()
#   message(STATUS "Warning: DPDK is not supported for creating shared libraries!")
# endif()

