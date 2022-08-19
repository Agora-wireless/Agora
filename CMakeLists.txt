cmake_minimum_required(VERSION 3.10)
include(CheckCSourceRuns)
cmake_policy(SET CMP0054 NEW)
#Allow project version
cmake_policy(SET CMP0048 NEW)
project(Agora VERSION 1.1.0)

if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7.0)
  set(GCC_COVERAGE_COMPILE_FLAGS "-faligned-new")
endif()

set(SOURCE_DIR ".")
configure_file(${CMAKE_SOURCE_DIR}/src/common/version_config.h.in ${CMAKE_SOURCE_DIR}/src/common/version_config.h)

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
  if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.2)
    message(STATUS "GCC version: ${CMAKE_CXX_COMPILER_VERSION} (versions newer than 9.2 are suggested for using AVX512 optimizations)")
  endif()
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

option(DEBUG "Enable debugging" OFF)

if(CMAKE_BUILD_TYPE MATCHES Debug)
  message(STATUS "Enabling debug mode due to build type")
  set(DEBUG ON)
endif()

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

message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
message(STATUS "CURRENT DIRECTORY: ${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DPROJECT_DIRECTORY=${SOURCE_DIR})

set(USE_DPDK False CACHE STRING "USE_DPDK defaulting to 'False'")
set(ENABLE_MAC False CACHE STRING "ENABLE_MAC defaulting to 'False'")
set(LOG_LEVEL "info" CACHE STRING "Console logging level (none/error/warn/info/frame/subframe/trace)") 
set(USE_MLX_NIC True CACHE STRING "USE_MLX_NIC defaulting to 'True'")
set(USE_AVX2_ENCODER False CACHE STRING "Use Agora's AVX2 encoder instead of FlexRAN's AVX512 encoder")

message(STATUS "ENABLE_MAC: ${ENABLE_MAC}")

set(RADIO_TYPE SIMULATION CACHE STRING "RADIO_TYPE defaulting to 'SIMULATION', valid types are SIMULATION / SOAPY_IRIS / SOAPY_UHD")
message(STATUS "RADIO_TYPE: ${RADIO_TYPE}")

if(RADIO_TYPE STREQUAL SOAPY_IRIS)
  message(STATUS "Enabled SOAPY Iris radio type")
  add_definitions(-DUSE_ARGOS)
elseif(RADIO_TYPE STREQUAL SOAPY_UHD)
  message(STATUS "Enabled SOAPY UHD radio type")
  add_definitions(-DUSE_UHD)
else()
  message(STATUS "Enabled SIMULATION radio type")
endif()

set(FLEXRAN_FEC_SDK_DIR /opt/FlexRAN-FEC-SDK-19-04/sdk)

# Determine if the current machine supports AVX-512
CHECK_C_SOURCE_RUNS("int main() { asm volatile(\"vmovdqu64 %zmm0, %zmm1\"); return 0; }" ISA_AVX512)
if (ISA_AVX512)
  message(STATUS "Processor supports AVX-512")
  add_definitions(-DISA_AVX512)
  # Leave USE_AVX2_ENCODER unmodified
else()
  message(STATUS "Processor does not support AVX-512")
  set(USE_AVX2_ENCODER True)
endif()

if(USE_AVX2_ENCODER)
  message(STATUS "Using Agora's (i.e., not FlexRAN's) AVX2 encoder")
  set(FLEXRAN_FEC_LIB_DIR ${FLEXRAN_FEC_SDK_DIR}/build-avx2-icc)
  add_definitions(-DUSE_AVX2_ENCODER)
else()
  message(STATUS "Using FlexRAN's (i.e., not Agora's) AVX512 encoder")
  set(FLEXRAN_FEC_LIB_DIR ${FLEXRAN_FEC_SDK_DIR}/build-avx512-icc)
endif()

# DPDK
message(STATUS "Use DPDK for agora: ${USE_DPDK}")

if(${USE_DPDK})
  find_library(DPDK_LIB dpdk)
  if(NOT DPDK_LIB)
    #If cmake doesn't find DPDK than have pkg-config attempt the same
    find_package(PkgConfig REQUIRED)
    pkg_search_module(DPDK REQUIRED libdpdk>=20.11.1)
    message(STATUS "  DPDK version ${DPDK_VERSION} is enabled for Agora")

    set(DPDK_LIBRARIES ${DPDK_LINK_LIBRARIES})
  else()
    message(STATUS "  DPDK is enabled for Agora")
    if(${USE_MLX_NIC})
      set(MLX_LIBRARIES -libverbs -lmlx5)
    endif()
    set(DPDK_LIBRARIES ${DPDK_LIBRARIES} -Wl,--whole-archive dpdk ${MLX_LIBRARIES} -Wl,--no-whole-archive dl)
    # DPDK include directory. Locating rte_config.h does not work on some systems.
    # Example: it may be kept in /usr/include/x86_64-linux-gnu/, and symlinked
    # from the real DPDK include directory (/usr/include/dpdk/).
    find_path(DPDK_INCLUDE_DIRS NAMES rte_ethdev.h PATH_SUFFIXES dpdk)
  endif()

  if(NOT DPDK_LIBRARIES)
    message(FATAL_ERROR "DPDK library not found")
  endif()

  if (DPDK_INCLUDE_DIRS)
    message(STATUS "  DPDK include directory = ${DPDK_INCLUDE_DIRS}")
    message(STATUS "  DPDK_LIBRARIES: ${DPDK_LIBRARIES}")
  else()
    message(FATAL_ERROR "DPDK include directory not found")
  endif()
  include_directories(SYSTEM ${DPDK_INCLUDE_DIRS})

  add_definitions(-DUSE_DPDK)
  if (ISA_AVX512)
    message(STATUS "Enabling AVX-512 memcpy implementation for dpdk")
    add_definitions(-DRTE_MEMCPY_AVX512)
  endif()
endif()

# MAC
if(${ENABLE_MAC})
  add_definitions(-DENABLE_MAC)
endif()

set(MAC_CLIENT_SOURCES
  src/mac/mac_sender.cc
  src/mac/mac_receiver.cc
  src/mac/video_receiver.cc
  src/mac/file_receiver.cc
  src/mac/mac_client.cc)
set(MAC_BS_SOURCES
  src/mac/mac_sender.cc
  src/mac/mac_receiver.cc
  src/mac/video_receiver.cc
  src/mac/file_receiver.cc
  src/mac/mac_basestation.cc)

#External Libraries
find_package(Armadillo "11.0.0" REQUIRED)

set(ENABLE_HDF5 False CACHE STRING "ENABLE_HDF5 defaulting to 'False'")
message(STATUS "HDF5 Support Enabled: ${ENABLE_HDF5}")
if (${ENABLE_HDF5})
  add_definitions(-DENABLE_HDF5)
  find_package(HDF5 1.10 REQUIRED COMPONENTS CXX)
  if (NOT HDF5_FOUND)
      message(FATAL_ERROR "HDF5 development files not found")
      return()
  endif()
  message(STATUS "HDF5_INCLUDE_DIRS: ${HDF5_INCLUDE_DIRS}")
  message(STATUS "HDF5_LIBRARIES: ${HDF5_LIBRARIES}")
  include_directories(${HDF5_INCLUDE_DIRS})
endif()

find_package(PythonLibs REQUIRED)
set(PYTHON_LIB ${PYTHON_LIBRARIES})
include_directories(${PYTHON_INCLUDE_DIRS})

message(STATUS "CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "PYTHON_LIBRARIES: ${PYTHON_LIBRARIES}")

#Soapy
find_package(SoapySDR "0.8.0" CONFIG)
if(NOT SoapySDR_FOUND)
  message(FATAL_ERROR "SoapySDR development files not found")
endif()
set(SOAPY_LIB ${SoapySDR_LIBRARIES})
include_directories(${SOAPY_INCLUDE_DIRS})

message(STATUS "SoapySDR_INCLUDE_DIRS: ${SoapySDR_INCLUDE_DIRS}")
message(STATUS "SoapySDR_LIBRARIES: ${SoapySDR_LIBRARIES}")
add_definitions(-DTHREADED_INIT)

# Intel MKL
set(BLA_VENDOR Intel10_64lp)
find_package(BLAS)

set(USE_SPDLOG False CACHE STRING "USE_SPDLOG defaulting to 'False'")
message(STATUS "USE_SPDLOG: ${USE_SPDLOG}")
if(USE_SPDLOG)
  if(LOG_LEVEL STREQUAL "none")
    add_definitions(-DSPDLOG_LEVEL=spdlog::level::off)
    add_definitions(-DSPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_OFF)
  elseif(LOG_LEVEL STREQUAL "error")
    add_definitions(-DSPDLOG_LEVEL=spdlog::level::err)
    add_definitions(-DSPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_ERROR)
  elseif(LOG_LEVEL STREQUAL "warn")
    add_definitions(-DSPDLOG_LEVEL=spdlog::level::warn)
    add_definitions(-DSPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_WARN)
  elseif(LOG_LEVEL STREQUAL "info")
    add_definitions(-DSPDLOG_LEVEL=spdlog::level::info)
    add_definitions(-DSPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_INFO)
  elseif(LOG_LEVEL STREQUAL "frame")
    add_definitions(-DSPDLOG_LEVEL=spdlog::level::debug)
    add_definitions(-DSPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG)
  elseif(LOG_LEVEL STREQUAL "subframe")
    add_definitions(-DSPDLOG_LEVEL=spdlog::level::debug)
    add_definitions(-DSPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG)
  elseif(LOG_LEVEL STREQUAL "trace")
    add_definitions(-DSPDLOG_LEVEL=spdlog::level::trace)
    add_definitions(-DSPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)
  endif()

  message(STATUS "Using spdlog async logger as the default logger")
  add_definitions(-DUSE_SPDLOG=true)
  add_definitions(-DSPDLOG_COMPILED_LIB)
  add_subdirectory(src/third_party/spdlog)
  set(COMMON_LIBS spdlog::spdlog)
else ()
  message(STATUS "Using agora raw logger")
endif()

set(ENABLE_CSV_LOG False CACHE STRING "ENABLE_CSV_LOG defaulting to 'False'")
set(ENABLE_MAT_LOG False CACHE STRING "ENABLE_MAT_LOG defaulting to 'False'")
message(STATUS "ENABLE_CSV_LOG: ${ENABLE_CSV_LOG}")
message(STATUS "ENABLE_MAT_LOG: ${ENABLE_MAT_LOG}")
if(USE_SPDLOG AND ENABLE_CSV_LOG)
  add_definitions(-DENABLE_CSV_LOG=true)
  message(STATUS "Enabled Csv Logger")
  if(ENABLE_MAT_LOG)
    add_definitions(-DENABLE_MAT_LOG=true)
    message(STATUS "Enabled Mat Logger")
  else()
    message(STATUS "Disabled Mat Logger")
  endif()
else()
  message(STATUS "Disabled Csv/Mat Logger")
endif()

# Console logging level
if(LOG_LEVEL STREQUAL "none")
  message(STATUS "Logging level = none.")
  add_definitions(-DAGORA_LOG_LEVEL=0)
elseif(LOG_LEVEL STREQUAL "error")
  message(STATUS "Logging level = error.")
  add_definitions(-DAGORA_LOG_LEVEL=1)
elseif(LOG_LEVEL STREQUAL "warn")
  message(STATUS "Logging level = warn.")
  add_definitions(-DAGORA_LOG_LEVEL=2)
elseif(LOG_LEVEL STREQUAL "info")
  message(STATUS "Logging level = info.")
  add_definitions(-DAGORA_LOG_LEVEL=3)
elseif(LOG_LEVEL STREQUAL "frame")
  message(STATUS "Logging level = frame. Warning: Performance will be low.")
  add_definitions(-DAGORA_LOG_LEVEL=4)
elseif(LOG_LEVEL STREQUAL "subframe")
  message(STATUS "Logging level = subframe. Warning: Performance will be low.")
  add_definitions(-DAGORA_LOG_LEVEL=5)
elseif(LOG_LEVEL STREQUAL "trace")
  message(STATUS "Logging level = trace. Warning: Performance will be low.")
  add_definitions(-DAGORA_LOG_LEVEL=6)
else()
  message(STATUS "No logging level specified. Using warning level.")
  add_definitions(-DAGORA_LOG_LEVEL=2)
endif()

find_package(gflags REQUIRED)
message(STATUS "gflags include directory found: ${GFLAGS_INCLUDE_DIR}")

# Unit tests
enable_testing()
find_package(GTest REQUIRED)

include_directories(
  src/common/
  src/common/ipc
  src/common/loggers
  src/recorder
  src/mac/
  src/agora/
  src/agora/radio
  src/agora/txrx
  src/agora/txrx/workers
  src/client
  src/client/txrx
  src/client/txrx/workers
  src/data_generator
  /usr/include/mkl)

include_directories(SYSTEM src/third_party src/third_party/spdlog/include src/third_party/nlohmann/single_include)

set(RECORDER_SOURCES
  src/recorder/recorder_thread.cc
  src/recorder/recorder_worker.cc
  src/recorder/recorder_worker_multifile.cc)

if (${ENABLE_HDF5})
  set(RECORDER_SOURCES
      ${RECORDER_SOURCES}
      src/recorder/hdf5_lib.cc
      src/recorder/recorder_worker_hdf5.cc)
endif()
add_library(recorder_sources_lib OBJECT ${RECORDER_SOURCES})

set(COMMON_SOURCES
  src/agora/stats.cc
  src/common/phy_stats.cc
  src/common/framestats.cc
  src/agora/doencode.cc
  src/common/utils.cc
  src/common/config.cc
  src/common/comms-lib.cc
  src/common/comms-lib-avx.cc
  src/common/signal_handler.cc
  src/common/modulation.cc
  src/common/modulation_srslte.cc
  src/common/net.cc
  src/common/crc.cc
  src/common/memory_manage.cc
  src/common/scrambler.cc
  src/agora/radio/radio.cc
  src/agora/radio/radio_soapysdr.cc
  src/agora/radio/radio_data_plane.cc
  src/agora/radio/radio_socket.cc
  src/agora/radio/radio_data_plane_soapy.cc
  src/agora/radio/radio_data_plane_socket.cc
  src/common/ipc/udp_client.cc
  src/common/ipc/udp_server.cc
  src/common/ipc/udp_server_ipv6.cc
  src/common/ipc/network_utils.cc
  src/common/loggers/csv_logger.cc
  src/common/loggers/mat_logger.cc
  src/encoder/cyclic_shift.cc
  src/encoder/encoder.cc
  src/encoder/iobuffer.cc)
add_library(common_sources_lib OBJECT ${COMMON_SOURCES})

set(SHARED_TXRX_SOURCES
  src/agora/txrx/packet_txrx.cc
  src/agora/txrx/workers/txrx_worker.cc)
add_library(shared_txrx_sources_lib OBJECT ${SHARED_TXRX_SOURCES})

if(${USE_DPDK})
  add_definitions(-DUSE_DPDK)
  set(AGORA_SOURCES
    src/common/dpdk_transport.cc
    src/agora/txrx/packet_txrx_dpdk.cc
    src/agora/txrx/workers/txrx_worker_dpdk.cc)
endif()

set(AGORA_SOURCES ${AGORA_SOURCES}
  src/agora/agora.cc
  src/agora/agora_worker.cc
  src/agora/dofft.cc
  src/agora/doifft.cc
  src/agora/dobeamweights.cc
  src/agora/dodemul.cc
  src/agora/doprecode.cc
  src/agora/dodecode.cc
  src/agora/radio/radio_lib.cc
  src/agora/radio/radio_calibrate_digital.cc
  src/agora/radio/radio_calibrate_analog.cc
  src/mac/mac_thread_basestation.cc
  src/agora/txrx/packet_txrx_sim.cc
  src/agora/txrx/packet_txrx_radio.cc
  src/agora/txrx/workers/txrx_worker_sim.cc
  src/agora/txrx/workers/txrx_worker_hw.cc
  src/agora/txrx/workers/txrx_worker_usrp.cc)

add_library(agora_sources_lib OBJECT ${AGORA_SOURCES})

set(CLIENT_SOURCES
  src/client/doifft_client.cc
  src/client/dodecode_client.cc
  src/client/doencode_client.cc
  src/client/client_radio.cc
  src/client/ue_worker.cc
  src/client/phy-ue.cc
  src/client/txrx/packet_txrx_client_sim.cc
  src/client/txrx/packet_txrx_client_radio.cc
  src/client/txrx/workers/txrx_worker_client_sim.cc
  src/client/txrx/workers/txrx_worker_client_hw.cc
  src/mac/mac_thread_client.cc)
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

set(COMMON_LIBS ${COMMON_LIBS} armadillo numa ${HDF5_LIBRARIES} ${DPDK_LIBRARIES} ${MKL_LIBS} ${SOAPY_LIB}
    ${PYTHON_LIB} ${FLEXRAN_LDPC_LIBS} util gflags gtest)

# TODO: The main agora executable is performance-critical, so we need to
# test if compiling against precompiled objects instead of compiling directly
# against sources reduces performance.
add_executable(agora
  src/agora/main.cc
  $<TARGET_OBJECTS:agora_sources_lib>
  $<TARGET_OBJECTS:shared_txrx_sources_lib>
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
  $<TARGET_OBJECTS:recorder_sources_lib>
  $<TARGET_OBJECTS:client_sources_lib>
  $<TARGET_OBJECTS:shared_txrx_sources_lib>
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

if(RADIO_TYPE STREQUAL SOAPY_IRIS)
add_executable(radio_test
    test/radio_test/main.cc
    $<TARGET_OBJECTS:agora_sources_lib>
    $<TARGET_OBJECTS:recorder_sources_lib>
    $<TARGET_OBJECTS:client_sources_lib>
    $<TARGET_OBJECTS:shared_txrx_sources_lib>
    $<TARGET_OBJECTS:common_sources_lib>)
target_link_libraries(radio_test ${COMMON_LIBS})
endif()

# End-to-end test
add_executable(test_agora
  test/test_agora/main.cc
  $<TARGET_OBJECTS:agora_sources_lib>
  $<TARGET_OBJECTS:shared_txrx_sources_lib>
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
  test_ptr_grid test_avx512_complex_mul test_scrambler
  test_256qam_demod)

foreach(test_name IN LISTS UNIT_TESTS)
  add_executable(${test_name}
    test/unit_tests/${test_name}.cc
    $<TARGET_OBJECTS:agora_sources_lib>
    $<TARGET_OBJECTS:shared_txrx_sources_lib>
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

