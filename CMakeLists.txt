cmake_minimum_required(VERSION 2.8)
cmake_policy(SET CMP0054 NEW)
project(Millipede)

if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7.0)
  set(GCC_COVERAGE_COMPILE_FLAGS "-faligned-new")
endif()

message(STATUS "CMAKE_C_COMPILER: ${CMAKE_C_COMPILER}")
message(STATUS "CMAKE_CXX_COMPILER: ${CMAKE_C_COMPILER}")

set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

if(${CMAKE_C_COMPILER_ID} STREQUAL "GNU")
  set(CMAKE_C_FLAGS "-std=gnu11 -Wall -g -march=native -m64")
  set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -march=native -m64")
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
  set(MKL_LIBS -Wl,--no-as-needed -lmkl_intel_lp64 -lmkl_sequential -lmkl_core -lmkl_avx2 -lmkl_def -lpthread -lm -ldl)
elseif(${CMAKE_C_COMPILER_ID} STREQUAL "Intel")
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/intel-compile-options.cmake)
  set(MKL_LIBS -lpthread -lm -ldl)
  set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -march=native -mkl=sequential")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")
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
set(USE_ARGOS False CACHE STRING "USE_ARGOS defaulting to 'False'")
set(USE_LDPC False CACHE STRING "USE_LDPC defaulting to 'False'")
set(LOG_LEVEL "warn" CACHE STRING "Console logging level (none/error/warn/info/frame/subframe/trace)") 

message(STATUS "USE_DPDK: ${USE_DPDK}")
message(STATUS "USE_ARGOS: ${USE_ARGOS}")
message(STATUS "USE_LDPC: ${USE_LDPC}")

set(TXRX_SOURCES src/millipede/txrx/txrx.cpp)

# DPDK
if(${USE_DPDK})
  message(STATUS "DPDK is enabled")
  message(STATUS "RTE_SDK: $ENV{RTE_SDK}")
  message(STATUS "RTE_TARGET: $ENV{RTE_TARGET}")
  find_package(dpdk)
  if(NOT dpdk_FOUND)
    message(STATUS "DPDK Not Found! Excluding from Compile List")
    set(USE_DPDK False)
  else()
    add_definitions(-DUSE_DPDK)
  endif()
  set(TXRX_SOURCES src/millipede/txrx/txrx_DPDK.cpp)
  #set(DPDK_LIB  "-Wl,--whole-archive -ldpdk -lnuma -ldl -Wl,--no-whole-archive" )
  include_directories(${dpdk_INCLUDE_DIRS})
  link_directories($ENV{RTE_SDK}/$ENV{RTE_TARGET}/lib)
else()
  message(STATUS "DPDK is disabled")
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
  set(ARGOS_SOURCES src/millipede/radio_lib.cpp src/millipede/radio_calibrate.cpp)
  set(TXRX_SOURCES src/millipede/txrx/txrx_argos.cpp)
  set(CLIENT_SOURCES
    src/common/modulation.cpp
    src/client/client_radio.cpp
    src/client/ru_argos.cpp
    src/client/phy-ue.cpp)
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

set(COMMON_SOURCES
  src/common/config.cpp
  src/common/utils.cpp
  src/common/comms-lib.cpp
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
  src/millipede/reciprocity.cpp)

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
    src/encoder/cyclic_shift.cpp
    src/encoder/encoder.cpp
    src/encoder/iobuffer.cpp
    src/millipede/docoding.cpp)
endif()

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

set(SOURCES
  src/millipede/main.cpp 
  ${MILLIPEDE_SOURCES}
  ${TXRX_SOURCES}
  ${COMMON_SOURCES}
  ${ARGOS_SOURCES}
  ${LDPC_SOURCES})
			
add_executable(millipede ${SOURCES} ${HEADER_FILES})
target_link_libraries(millipede -larmadillo
  ${MKL_LIBS} ${dpdk_LIBRARIES} ${SOAPY_LIB} ${PYTHON_LIB} ${LDPC_LIBS})


add_library(comp MODULE ${MILLIPEDE_SOURCES} ${TXRX_SOURCES} ${COMMON_SOURCES} ${ARGOS_SOURCES} ${LDPC_SOURCES} ${HEADER_FILES})
target_link_libraries(comp -larmadillo ${MKL_LIBS} ${SOAPY_LIB} ${PYTHON_LIB} ${LDPC_LIBS})

add_executable(data_generator 
  ${CMAKE_CURRENT_SOURCE_DIR}/data/data_generator/data_generator.cpp 
  ${LDPC_SOURCES} ${COMMON_SOURCES} ${HEADER_FILES})
target_link_libraries(data_generator -larmadillo ${LDPC_LIBS} ${MKL_LIBS})
target_compile_definitions(data_generator PRIVATE GENERATE_DATA)

if(${USE_ARGOS})
  add_executable(user 
       src/client/user-main.cpp
       ${CLIENT_SOURCES}
       ${COMMON_SOURCES})
  target_link_libraries(user -larmadillo ${MKL_LIBS} ${SOAPY_LIB} ${PYTHON_LIB})

  add_library(ue_phy MODULE 
       ${CLIENT_SOURCES}
       ${COMMON_SOURCES}
       ${ARGOS_SOURCES})
  target_link_libraries(ue_phy -larmadillo ${MKL_LIBS} ${SOAPY_LIB} ${PYTHON_LIB})
else()
  set(SENDER_SOURCES
    simulator/sender_cli.cpp
    simulator/sender.cpp 
    ${COMMON_SOURCES})

  add_executable(sender ${SENDER_SOURCES})
  target_link_libraries(sender ${MKL_LIBS})

  set(SIMULATOR_SOURCES
    simulator/main.cpp
    simulator/simulator.cpp
    simulator/sender.cpp
    simulator/receiver.cpp 
    ${COMMON_SOURCES})

  add_executable(sim ${SIMULATOR_SOURCES})
  target_link_libraries(sim ${MKL_LIBS})
endif()

# The tests to run using ctest
add_executable(test_zf test/test_zf.cc ${COMMON_SOURCES} src/millipede/stats.cpp src/millipede/dozf.cpp)
target_link_libraries(test_zf ${GTEST_LIBRARIES} ${MKL_LIBS} -larmadillo)
add_test(NAME test_zf COMMAND test_zf)
