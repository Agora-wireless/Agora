cmake_minimum_required (VERSION 2.8)
cmake_policy(SET CMP0054 NEW)
project (Millipede)

if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7.0)
  set(GCC_COVERAGE_COMPILE_FLAGS "-faligned-new")
endif()

message(STATUS "CMAKE_C_COMPILER: ${CMAKE_C_COMPILER}")
message(STATUS "CMAKE_CXX_COMPILER: ${CMAKE_C_COMPILER}")

set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../..)

if(${CMAKE_C_COMPILER_ID} STREQUAL "GNU")
  set(CMAKE_C_FLAGS "-std=c99 -Wall -g -march=native -m64")
  set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -march=native  -m64")
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
  set(MKL_LIBS -Wl,--no-as-needed -lmkl_intel_lp64 -lmkl_sequential -lmkl_core -lpthread -lm -ldl)
elseif(${CMAKE_C_COMPILER_ID} STREQUAL "Intel")
  include(${SOURCE_DIR}/cmake/intel-compile-options.cmake)
  set(MKL_LIBS -lpthread -lm -ldl)
  set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -march=native -mkl=sequential")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")
  set(CMAKE_CXX_STANDARD 14)
else()
  set(CMAKE_CXX_STANDARD 11)
endif()

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
add_definitions(-DPROJECT_DIRECTORY=${SOURCE_DIR})
add_definitions(-DWRITE_DEMUL)
add_definitions(-DWRITE_TX)


find_package(Armadillo)

set(USE_DPDK False CACHE STRING "USE_DPDK defaulting to 'False'")
set(USE_ARGOS False CACHE STRING "USE_ARGOS defaulting to 'False'")
set(USE_LDPC False CACHE STRING "USE_LDPC defaulting to 'False'")
set(ENABLE_MAC False CACHE STRING "ENABLE_MAC defaulting to 'False'")

message(STATUS "USE_DPDK: ${USE_DPDK}")
message(STATUS "USE_ARGOS: ${USE_ARGOS}")
message(STATUS "USE_LDPC: ${USE_LDPC}")
message(STATUS "ENABLE_MAC: ${ENABLE_MAC}")


set(TXRX_SOURCES ${SOURCE_DIR}/src/millipede/txrx/txrx.cpp)


# Intel MKL
set(BLA_VENDOR Intel10_64lp)
find_package(BLAS)

# DPDK
if(${USE_DPDK})
  message(STATUS "DPDK is enabled")

  find_library(DPDK_LIB dpdk)
  message(STATUS "DPDK_LIB: ${DPDK_LIB}")
  if(NOT DPDK_LIB)
    message(FATAL_ERROR "DPDK library not found")
  endif()

  set(DPDK_LIBRARIES ${DPDK_LIBRARIES} -Wl,--whole-archive dpdk -libverbs -lmlx5 -lrt -lm -lnuma -Wl,--no-whole-archive dl)
  add_definitions(-DUSE_DPDK)
  set(TXRX_SOURCES ${SOURCE_DIR}/src/millipede/txrx/txrx_DPDK.cpp)
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
else()
  message(STATUS "DPDK is disabled")
endif()

if(${ENABLE_MAC})
  add_definitions(-DENABLE_MAC)
endif()
set(MAC_TX_SOURCES ${SOURCE_DIR}/src/millipede/txrx/txrx_mac.cpp)

include_directories(
	${SOURCE_DIR}/src/common/
  ${SOURCE_DIR}/src/millipede/
  ${SOURCE_DIR}/src/millipede/txrx/
  ${SOURCE_DIR}/src/client/
	)

set(COMMON_SOURCES 
  ${SOURCE_DIR}/src/common/config.cpp
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

add_library(comp MODULE ${MILLIPEDE_SOURCES} ${TXRX_SOURCES} ${COMMON_SOURCES} ${ARGOS_SOURCES})
target_link_libraries(comp -lpthread -larmadillo ${MKL_LIBS} ${SOAPY_LIB} ${PYTHON_LIB})


# LDPC support
if(${USE_LDPC})
  if(NOT ${CMAKE_C_COMPILER_ID} STREQUAL "Intel")
    message(STATUS "CMAKE_C_COMPILER_ID: ${CMAKE_C_COMPILER_ID}")
    message(FATAL_ERROR "Intel compiler is required for LDPC library!" )
  endif()
  add_definitions(-DUSE_LDPC)
  set(SDK_DIR /home/argos/Jian/FlexRAN-FEC-SDK-19-04/FlexRAN-FEC-SDK-19-04/sdk/)
  set(LIB_DIR ${SDK_DIR}/build-avx2-icc)
  set(LDPC_LIBS ${LIB_DIR}/source/phy/lib_ldpc_decoder_5gnr/libldpc_decoder_5gnr.a
                ${LIB_DIR}/test/phy/common/utility.a
                ${LIB_DIR}/source/phy/lib_common/libcommon.a)

  include_directories(
    ${SDK_DIR}/test/phy/common
    ${SDK_DIR}/source/phy/lib_ldpc_decoder_5gnr
    ${SDK_DIR}/source/phy/lib_common
    ${SOURCE_DIR}/src/encoder)

  set(HEADER_FILES ${SDK_DIR}/source/phy/lib_ldpc_decoder_5gnr/phy_ldpc_decoder_5gnr.h)

  set(LDPC_SOURCES  
    ${SOURCE_DIR}/src/encoder/cyclic_shift.cpp
    ${SOURCE_DIR}/src/encoder/encoder.cpp
    ${SOURCE_DIR}/src/encoder/iobuffer.cpp
    ${SOURCE_DIR}/src/millipede/docoding.cpp)
endif()


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
  set(ARGOS_SOURCES 
    ${SOURCE_DIR}/src/millipede/radio_lib.cpp 
    ${SOURCE_DIR}/src/millipede/radio_calibrate.cpp)
  set(TXRX_SOURCES 
    ${SOURCE_DIR}/src/millipede/txrx/txrx_argos.cpp)
  set(CLIENT_SOURCES
    ${SOURCE_DIR}/src/common/modulation.cpp
    ${SOURCE_DIR}/src/client/client_radio.cpp
    ${SOURCE_DIR}/src/client/ru_argos.cpp
    ${SOURCE_DIR}/src/client/phy-ue.cpp)
  set(PYTHON_LIB ${PYTHON_LIBRARIES}) 
  add_definitions(-DTHREADED_INIT)
endif()


get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

set(SOURCES
  ${MILLIPEDE_SOURCES}
  ${TXRX_SOURCES}
  ${MAC_TX_SOURCES}
  ${COMMON_SOURCES}
  ${ARGOS_SOURCES}
  ${LDPC_SOURCES})
      
add_executable(millipede main.cpp ${SOURCES} ${HEADER_FILES})
target_link_libraries(millipede -larmadillo
  ${MKL_LIBS} ${DPDK_LIBRARIES} ${SOAPY_LIB} ${PYTHON_LIB} ${LDPC_LIBS})

set(SENDER_SOURCES 
  ${SOURCE_DIR}/simulator/sender_cli.cpp
  ${SOURCE_DIR}/simulator/sender.cpp 
  ${COMMON_SOURCES}
  ${LDPC_SOURCES}
  ${HEADER_FILES})

add_executable(sender ${SENDER_SOURCES})
target_link_libraries(sender -lpthread gflags ${MKL_LIBS} ${LDPC_LIBS})

set(SIMULATOR_SOURCES 
  ${SOURCE_DIR}/simulator/main.cpp
  ${SOURCE_DIR}/simulator/simulator.cpp
  ${SOURCE_DIR}/simulator/sender.cpp
  ${SOURCE_DIR}/simulator/receiver.cpp 
  ${COMMON_SOURCES}
  ${LDPC_SOURCES}
  ${HEADER_FILES})

add_executable(sim ${SIMULATOR_SOURCES})
target_link_libraries(sim -lpthread ${MKL_LIBS} ${LDPC_LIBS})

add_executable(data_generator 
  ${SOURCE_DIR}/data/data_generator/data_generator.cpp 
  ${LDPC_SOURCES} ${COMMON_SOURCES} ${HEADER_FILES})
target_link_libraries(data_generator -larmadillo ${LDPC_LIBS} ${MKL_LIBS})
target_compile_definitions(data_generator PRIVATE GENERATE_DATA)



