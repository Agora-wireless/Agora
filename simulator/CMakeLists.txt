cmake_minimum_required(VERSION 2.8)
cmake_policy(SET CMP0054 NEW)
project(Millipede)

if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7.0)
  set(GCC_COVERAGE_COMPILE_FLAGS "-faligned-new")
endif()

message(STATUS "CMAKE_C_COMPILER: ${CMAKE_C_COMPILER}")
message(STATUS "CMAKE_CXX_COMPILER: ${CMAKE_C_COMPILER}")

if(${CMAKE_C_COMPILER_ID} STREQUAL "GNU")
  set(CMAKE_C_FLAGS "-std=c99 -Wall -g -march=native -m64")
  set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -march=native -m64")
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
  set(MKL_LIBS -Wl,--no-as-needed -lmkl_intel_lp64 -lmkl_sequential -lmkl_core -lpthread -lm -ldl)
elseif(${CMAKE_C_COMPILER_ID} STREQUAL "Intel")
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/intel-compile-options.cmake)
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
add_definitions(-DPROJECT_DIRECTORY=${CMAKE_CURRENT_SOURCE_DIR}/..)

find_package(Armadillo)

set(USE_DPDK False CACHE STRING "USE_DPDK defaulting to 'False'")
set(USE_DPDK_SENDER True CACHE STRING "USE_DPDK_SENDER defaulting to the same value as USE_DPDK")

if(${USE_DPDK})
  message(STATUS "DPDK is enabled for BS")
  add_definitions(-DUSE_DPDK)
else()
  set(USE_DPDK_SENDER False)
  message(STATUS "DPDK is disabled")
endif()


# DPDK
if(${USE_DPDK_SENDER})
  message(STATUS "DPDK is enabled for Millipede")

  find_library(DPDK_LIB dpdk)
  message(STATUS "DPDK_LIB: ${DPDK_LIB}")
  if(NOT DPDK_LIB)
    message(FATAL_ERROR "DPDK library not found")
  endif()

  set(DPDK_LIBRARIES ${DPDK_LIBRARIES} -Wl,--whole-archive dpdk -libverbs -lmlx5 -lrt -lm -lnuma -Wl,--no-whole-archive dl)

  set(TXRX_SOURCES 
    ../src/millipede/txrx/txrx_DPDK.cpp
    ../src/common/dpdk_transport.cpp)

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
  
  message(STATUS "DPDK is enabled for sender")
  add_definitions(-DUSE_DPDK_SENDER)
  set(SENDER_TXRX_SOURCES 
    sender_dpdk.cpp
    ../src/common/dpdk_transport.cpp)
endif()

if (NOT ${USE_DPDK_SENDER})
  message(STATUS "DPDK is disabled for sender")
  set(USE_DPDK_SENDER False)
  set(SENDER_TXRX_SOURCES sender.cpp)
endif()

message(STATUS "USE_DPDK: ${USE_DPDK}")
message(STATUS "USE_DPDK_SENDER: ${USE_DPDK_SENDER}")




include_directories(
  ../src/common/)

set(COMMON_SOURCES 
  ../src/common/config.cpp
  ../src/common/utils.cpp
  ../src/common/comms-lib.cpp
  ../src/common/signalHandler.cpp
  ../src/common/modulation.cpp
  ../src/common/net.cpp)


set(SENDER_SOURCES 
  sender_cli.cpp
  ${SENDER_TXRX_SOURCES}
  ${COMMON_SOURCES})

add_executable(sender ${SENDER_SOURCES})
target_link_libraries(sender -lpthread ${MKL_LIBS} ${DPDK_LIBRARIES})


set(SIMULATOR_SOURCES 
  main.cpp
  simulator.cpp
  ${SENDER_TXRX_SOURCES}
  receiver.cpp 
  ${COMMON_SOURCES})

add_executable(sim ${SIMULATOR_SOURCES})
target_link_libraries(sim -lpthread ${MKL_LIBS} ${DPDK_LIBRARIES})