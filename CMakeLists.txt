cmake_minimum_required(VERSION 2.8)
cmake_policy(SET CMP0054 NEW)
project(Millipede)

if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7.0)
  set(GCC_COVERAGE_COMPILE_FLAGS "-faligned-new")
endif()

message(STATUS "CMAKE_C_COMPILER: ${CMAKE_C_COMPILER}")
message(STATUS "CMAKE_CXX_COMPILER: ${CMAKE_C_COMPILER}")

set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

if(${CMAKE_C_COMPILER_ID} STREQUAL "GNU")
  message(STATUS "Using GNU compiler, compiler ID ${CMAKE_C_COMPILER_ID}")
  set(CMAKE_C_FLAGS "-std=gnu11 -Wall -g -march=native -m64")
  set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -march=native -m64")
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
  set(MKL_LIBS -Wl,--no-as-needed -lmkl_intel_lp64 -lmkl_sequential -lmkl_core -lmkl_avx2 -lmkl_def -lpthread -lm -ldl)
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

# Unit tests
enable_testing()
find_package(GTest REQUIRED)

option(DEBUG "Enable debugging" OFF)
if(NOT DEBUG)
  message(STATUS "Debugging is disabled")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
else()
  message(STATUS "Debugging is enabled. Performance will be low")
endif()

message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

message(STATUS "CURRENT DIRECTORY: ${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DPROJECT_DIRECTORY=${CMAKE_CURRENT_SOURCE_DIR})

find_package(Armadillo)

set(USE_DPDK False CACHE STRING "USE_DPDK defaulting to 'False'")
set(USE_DPDK_SENDER True CACHE STRING "USE_DPDK_SENDER defaulting to the same value as USE_DPDK")
set(USE_ARGOS False CACHE STRING "USE_ARGOS defaulting to 'False'")
set(USE_LDPC False CACHE STRING "USE_LDPC defaulting to 'False'")
set(ENABLE_MAC False CACHE STRING "ENABLE_MAC defaulting to 'False'")
set(LOG_LEVEL "warn" CACHE STRING "Console logging level (none/error/warn/info/frame/subframe/trace)") 

message(STATUS "USE_ARGOS: ${USE_ARGOS}")
message(STATUS "USE_LDPC: ${USE_LDPC}")
message(STATUS "ENABLE_MAC: ${ENABLE_MAC}")

set(TXRX_SOURCES src/millipede/txrx/txrx.cpp)

# DPDK
if(${USE_DPDK})
  message(STATUS "DPDK is enabled for Millipede")

  find_library(DPDK_LIB dpdk)
  message(STATUS "DPDK_LIB: ${DPDK_LIB}")
  if(NOT DPDK_LIB)
    message(FATAL_ERROR "DPDK library not found")
  endif()

  set(DPDK_LIBRARIES ${DPDK_LIBRARIES} -Wl,--whole-archive dpdk -libverbs -lmlx5 -lrt -lnuma -Wl,--no-whole-archive dl)

  set(TXRX_SOURCES 
    src/millipede/txrx/txrx_DPDK.cpp
    src/common/dpdk_transport.cpp)

  # DPDK include directory. Locating rte_config.h does not work on some systems.
  # Example: it may be kept in /usr/include/x86_64-linux-gnu/, and symlinked
  # from the real DPDK include directory (/usr/include/dpdk/).
  find_path(DPDK_INCLUDE_DIR NAMES rte_ethdev.h PATH_SUFFIXES dpdk)
  if (DPDK_INCLUDE_DIR)
    message(STATUS "DPDK include directory = ${DPDK_INCLUDE_DIR}")
  else()
    message(FATAL_ERROR "DPDK include directory not found")
  endif()
  include_directories(${DPDK_INCLUDE_DIR})

  add_definitions(-DUSE_DPDK)
  if (${USE_DPDK_SENDER})
    message(STATUS "DPDK is enabled for sender")
    add_definitions(-DUSE_DPDK_SENDER)
    set(SENDER_TXRX_SOURCES 
      simulator/sender_dpdk.cpp
      src/common/dpdk_transport.cpp)
  endif()
else()
  set(USE_DPDK_SENDER False)
  message(STATUS "DPDK is disabled")
endif()

if (NOT ${USE_DPDK_SENDER})
  message(STATUS "DPDK is disabled for sender")
  set(USE_DPDK_SENDER False)
  set(SENDER_TXRX_SOURCES simulator/sender.cpp)
endif()

message(STATUS "USE_DPDK: ${USE_DPDK}")
message(STATUS "USE_DPDK_SENDER: ${USE_DPDK_SENDER}")


if(${ENABLE_MAC})
  add_definitions(-DENABLE_MAC)
  set(MAC_CLIENT_SOURCES
	  src/mac/simple_mac_client.cpp
	  src/mac/mac_client_main.cpp)
  set(MAC_BS_SOURCES
	  src/mac/simple_mac_bs.cpp
	  src/mac/mac_bs_main.cpp)
endif()
set(TXRX_MAC_SOURCES src/millipede/txrx/txrx_mac.cpp)

# Argos support
if(${USE_ARGOS})
  find_package(SoapySDR "0.7" CONFIG)
  if(NOT SoapySDR_FOUND)
    message(STATUS "SoapySDR development files not found")
    set(USE_ARGOS False)
  else()
    add_definitions(-DUSE_ARGOS)
  endif()
  message(STATUS "CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")
  find_package(PythonLibs REQUIRED)
  message(STATUS "PYTHON_LIBRARIES: ${PYTHON_LIBRARIES}")
  message(STATUS "SoapySDR_INCLUDE_DIRS: ${SoapySDR_INCLUDE_DIRS}")
  message(STATUS "SoapySDR_LIBRARIES: ${SoapySDR_LIBRARIES}")
  include_directories(${PYTHON_INCLUDE_DIRS} ${SoapySDR_INCLUDE_DIRS})
  set(SOAPY_LIB ${SoapySDR_LIBRARIES})
  set(ARGOS_SOURCES src/millipede/radio_lib.cpp src/millipede/radio_calibrate.cpp)
  set(TXRX_SOURCES src/millipede/txrx/txrx_argos.cpp)
  set(CLIENT_SOURCES
    src/common/modulation.cpp
    src/client/client_radio.cpp
    src/client/txrx_argos.cpp
    src/client/phy-ue.cpp
    src/client/txrx_mac.cpp
  )
  set(PYTHON_LIB ${PYTHON_LIBRARIES}) 
  add_definitions(-DTHREADED_INIT)
endif()

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
  src/millipede/
  src/millipede/txrx/
  src/client/)

include_directories(SYSTEM src/third_party)

set(COMMON_SOURCES
  src/common/config.cpp
  src/common/utils.cpp
  src/common/comms-lib.cpp
  src/common/comms-lib-avx.cpp
  src/common/signalHandler.cpp
  src/common/modulation.cpp
  src/common/net.cpp)

set(MILLIPEDE_SOURCES 
  src/millipede/millipede.cpp
  src/millipede/stats.cpp 
  src/millipede/dofft.cpp
  src/millipede/dozf.cpp
  src/millipede/dodemul.cpp
  src/millipede/doprecode.cpp
  src/millipede/reciprocity.cpp
  src/encoder/cyclic_shift.cpp
  src/encoder/encoder.cpp
  src/encoder/iobuffer.cpp)

# LDPC headers and sources that do not require an Intel compiler
set(FLEXRAN_SDK_DIR /opt/FlexRAN-FEC-SDK-19-04/sdk/)
include_directories(
  ${FLEXRAN_SDK_DIR}/source/phy/lib_ldpc_decoder_5gnr
  ${FLEXRAN_SDK_DIR}/source/phy/lib_common
  ${SOURCE_DIR}/src/encoder)

set(LDPC_SOURCES
  src/encoder/encoder.cpp
  src/encoder/cyclic_shift.cpp
  src/encoder/iobuffer.cpp)

# LDPC sources that require an Intel compiler
if(${USE_LDPC})
  message(STATUS "LDPC is enabled")
  if(NOT ${CMAKE_C_COMPILER_ID} STREQUAL "Intel")
    message(STATUS "CMAKE_C_COMPILER_ID: ${CMAKE_C_COMPILER_ID}")
    message(FATAL_ERROR "Intel compiler is required for LDPC library!" )
  endif()
  add_definitions(-DUSE_LDPC)
  set(FLEXRAN_LIB_DIR ${FLEXRAN_SDK_DIR}/build-avx2-icc)
  set(FLEXRAN_LDPC_LIBS
    ${FLEXRAN_LIB_DIR}/source/phy/lib_ldpc_decoder_5gnr/libldpc_decoder_5gnr.a
    ${FLEXRAN_LIB_DIR}/source/phy/lib_common/libcommon.a)

  set(MILLIPEDE_SOURCES ${MILLIPEDE_SOURCES} src/millipede/docoding.cpp)
endif()

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

set(SOURCES
  ${MILLIPEDE_SOURCES}
  ${TXRX_SOURCES}
  ${TXRX_MAC_SOURCES}
  ${COMMON_SOURCES}
  ${ARGOS_SOURCES}
  ${LDPC_SOURCES})
			
add_executable(millipede src/millipede/main.cpp ${SOURCES} ${HEADER_FILES})
target_link_libraries(millipede -larmadillo
  ${MKL_LIBS} ${DPDK_LIBRARIES} ${SOAPY_LIB} ${PYTHON_LIB} ${FLEXRAN_LDPC_LIBS})


add_library(comp MODULE ${SOURCES} ${HEADER_FILES})
target_link_libraries(comp -larmadillo ${MKL_LIBS} ${SOAPY_LIB} ${PYTHON_LIB} ${FLEXRAN_LDPC_LIBS})

add_executable(data_generator 
  ${CMAKE_CURRENT_SOURCE_DIR}/data/data_generator/data_generator.cpp 
  ${LDPC_SOURCES} 
  ${COMMON_SOURCES} 
  ${HEADER_FILES})
target_link_libraries(data_generator -larmadillo ${FLEXRAN_LDPC_LIBS} ${MKL_LIBS})
target_compile_definitions(data_generator PRIVATE GENERATE_DATA)

if(${USE_ARGOS})
  add_executable(user 
       src/client/user-main.cpp
       ${CLIENT_SOURCES}
       ${COMMON_SOURCES}
       ${LDPC_SOURCES}
       ${HEADER_FILES})
  target_link_libraries(user -larmadillo ${MKL_LIBS} ${SOAPY_LIB} ${PYTHON_LIB} ${LDPC_LIBS})

  add_library(ue_phy MODULE 
       ${CLIENT_SOURCES}
       ${COMMON_SOURCES}
       ${ARGOS_SOURCES}
       ${LDPC_SOURCES}
       ${HEADER_FILES})
  target_link_libraries(ue_phy -larmadillo ${MKL_LIBS} ${SOAPY_LIB} ${PYTHON_LIB} ${LDPC_LIBS})

  if(${ENABLE_MAC})
    add_executable(macuser
         ${MAC_CLIENT_SOURCES}
         ${COMMON_SOURCES})
    target_link_libraries(macuser -larmadillo ${MKL_LIBS})

    add_executable(macbs
         ${MAC_BS_SOURCES}
         ${COMMON_SOURCES})
    target_link_libraries(macbs -larmadillo ${MKL_LIBS})
  endif()
else()
  set(SENDER_SOURCES
    simulator/sender_cli.cpp
    ${COMMON_SOURCES}
    ${SENDER_TXRX_SOURCES}
    ${LDPC_SOURCES}
    ${HEADER_FILES})

  add_executable(sender ${SENDER_SOURCES})
  target_link_libraries(sender ${MKL_LIBS} ${DPDK_LIBRARIES} ${LDPC_LIBS} gflags)

  set(SIMULATOR_SOURCES
    simulator/main.cpp
    simulator/simulator.cpp
    simulator/sender.cpp
    simulator/receiver.cpp 
    ${COMMON_SOURCES}
    ${LDPC_SOURCES}
    ${HEADER_FILES})

  add_executable(sim ${SIMULATOR_SOURCES})
  target_link_libraries(sim ${MKL_LIBS} ${LDPC_LIBS})
  if(${ENABLE_MAC})
    add_executable(macbs
         ${MAC_BS_SOURCES}
         ${COMMON_SOURCES})
    target_link_libraries(macbs -larmadillo ${MKL_LIBS})
  endif()
endif()

# The tests to run using ctest
add_executable(test_zf test/test_zf.cc 
  ${COMMON_SOURCES} 
  src/millipede/stats.cpp 
  src/millipede/dozf.cpp
  ${LDPC_SOURCES}
  ${HEADER_FILES})
target_link_libraries(test_zf ${GTEST_LIBRARIES} ${MKL_LIBS} -larmadillo ${LDPC_LIBS})
add_test(NAME test_zf COMMAND test_zf)
