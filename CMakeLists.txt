cmake_minimum_required (VERSION 2.8)
cmake_policy(SET CMP0054 NEW)
project (Millipede)

# Enable C++11
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")
    set (CMAKE_CXX_STANDARD 14)
else ()
    set (CMAKE_CXX_STANDARD 11)
endif()

if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7.0)
  set(GCC_COVERAGE_COMPILE_FLAGS "-faligned-new")
endif()

message(STATUS "CMAKE_C_COMPILER: ${CMAKE_C_COMPILER}")
message(STATUS "CMAKE_CXX_COMPILER: ${CMAKE_C_COMPILER}")

if (${CMAKE_C_COMPILER} STREQUAL "/usr/bin/gcc")
  set(CMAKE_C_FLAGS "-std=c99 -Wall -g -O3 -mavx2 -march=broadwell")
  set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -O3 -mavx2 -mavx -march=broadwell  -m64")
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
  set (MKL_LIBS -lmkl_rt -lmkl_intel_ilp64 -lmkl_sequential -lmkl_core -lm -ldl)
else ()
  include (${CMAKE_CURRENT_SOURCE_DIR}/cmake/intel-compile-options.cmake)
  set(MKL_LIBS  -lmkl_rt -lpthread -lm -ldl)
  set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -O3 -mavx2 -mavx -march=broadwell")
endif()

message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

message(STATUS "CURRENT DIRECTORY: ${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DPROJECT_DIRECTORY=${CMAKE_CURRENT_SOURCE_DIR})

find_package(Armadillo)

SET(USE_DPDK False CACHE STRING "USE_DPDK defaulting to 'False'")
SET(USE_ARGOS False CACHE STRING "USE_ARGOS defaulting to 'False'")
SET(USE_LDPC False CACHE STRING "USE_LDPC defaulting to 'False'")

message(STATUS "USE_DPDK: ${USE_DPDK}")
message(STATUS "USE_ARGOS: ${USE_ARGOS}")
message(STATUS "USE_LDPC: ${USE_LDPC}")

set (TXRX_SOURCE src/millipede/txrx/txrx.cpp)

# DPDK
if (${USE_DPDK})
  find_package(dpdk)
  if (NOT dpdk_FOUND)
    message(STATUS "DPDK Not Found! Excluding from Compile List")
    set(USE_DPDK False)
  else ()
    add_definitions(-DUSE_DPDK)
  endif()
  set (TXRX_SOURCE src/millipede/txrx/txrx_DPDK.cpp)
  set (RTE_SDK /home/argos/Jian/dpdk-stable-18.11.1/)
  set (RTE_TARGET x86_64-native-linuxapp-gcc)
  set ( DPDK_LIB  "-Wl,--whole-archive -ldpdk -lnuma -ldl -Wl,--no-whole-archive" )
  include_directories(${RTE_SDK}/${RTE_TARGET}/include)
  link_directories(${RTE_SDK}/${RTE_TARGET}/lib)
endif()


# Argos support
if (${USE_ARGOS})
  find_package(SoapySDR "0.7" CONFIG)
  if (NOT SoapySDR_FOUND)
    message(STATUS "SoapySDR development files not found")
    set(USE_ARGOS False)
  else ()
    add_definitions(-DUSE_ARGOS)
  endif ()
  message(STATUS "CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")
  find_package(PythonLibs REQUIRED)
  message(STATUS "PYTHON_LIBRARIES: ${PYTHON_LIBRARIES}")
  message(STATUS "SoapySDR_INCLUDE_DIRS: ${SoapySDR_INCLUDE_DIRS}")
  message(STATUS "SoapySDR_LIBRARIES: ${SoapySDR_LIBRARIES}")
  include_directories(${PYTHON_INCLUDE_DIRS} ${SoapySDR_INCLUDE_DIRS})
  set (SOAPY_LIB ${SoapySDR_LIBRARIES})
  set (ARGOS_SOURCES src/millipede/radio_lib.cpp)
  set (TXRX_SOURCE src/millipede/txrx/txrx_argos.cpp)
  set (CLIENT_SOURCES
         src/common/modulation.cpp
         src/millipede/ru.cpp
         src/millipede/phy-ue.cpp)
  set (PYTHON_LIB ${PYTHON_LIBRARIES}) 
  add_definitions(-DTHREADED_INIT)
endif ()


# Intel MKL
find_path(MKL_INCLUDE_DIR 
          NAMES mkl_dfti.h
          PATHS /opt/intel/*/linux/mkl/include)
set (MKL_LINK_DIR ${MKL_INCLUDE_DIR}/../lib/intel64)
message(STATUS "MKL_INCLUDE_DIR: ${MKL_INCLUDE_DIR}")
message(STATUS "MKL_LINK_DIR: ${MKL_LINK_DIR}")
include_directories(${MKL_INCLUDE_DIR})
link_directories(${MKL_LINK_DIR})

include_directories(
	src/common/
  src/millipede/
  src/millipede/txrx/
	)



set(COMMON_SOURCES src/common/config.cpp
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
       src/millipede/doprecode.cpp)

add_library(comp MODULE 
       ${MILLIPEDE_SOURCES}
       ${TXRX_SOURCE}
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
                    src/encoder)

  #set(HEADER_FILES ${SDK_DIR}/source/phy/lib_ldpc_decoder_5gnr/phy_ldpc_decoder_5gnr.h)
  set(HEADER_FILES ${SDK_DIR}/decoder/src/lib_ldpc_decoder_5gnr/phy_ldpc_decoder_5gnr.h)

  set(LDPC_SOURCES  src/encoder/cyclic_shift.cpp
                    src/encoder/encoder.cpp
                    src/encoder/iobuffer.cpp
                    src/millipede/docoding.cpp)

endif()

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

set(SOURCES src/millipede/main.cpp 
            ${MILLIPEDE_SOURCES}
            ${TXRX_SOURCE}
            ${COMMON_SOURCES}
            ${ARGOS_SOURCES}
            ${LDPC_SOURCES})
			
add_executable(millipede ${SOURCES} ${HEADER_FILES})
target_link_libraries(millipede -lpthread /usr/local/lib/libmufft.a -larmadillo ${MKL_LIBS} ${DPDK_LIB} ${SOAPY_LIB} ${PYTHON_LIB} ${LDPC_LIBS})



if (${USE_ARGOS})
  add_executable(user 
       src/millipede/user-main.cpp
       ${CLIENT_SOURCES}
       ${COMMON_SOURCES}
       ${ARGOS_SOURCES})
  target_link_libraries(user -lpthread -lmufft -larmadillo ${SOAPY_LIB} ${PYTHON_LIB})

  add_library(ue_phy MODULE 
       ${CLIENT_SOURCES}
       ${COMMON_SOURCES}
       ${ARGOS_SOURCES})
  target_link_libraries(ue_phy -lpthread -lmufft -larmadillo ${SOAPY_LIB} ${PYTHON_LIB})
else()
  set(SENDER_SOURCES simulator/sender_cli.cpp
                     simulator/sender.cpp 
                     ${COMMON_SOURCES})

  add_executable(sender ${SENDER_SOURCES})
  target_link_libraries(sender -lpthread -lmufft)


  set(SIMULATOR_SOURCES simulator/main.cpp
                        simulator/simulator.cpp
                        simulator/sender.cpp
                        simulator/receiver.cpp 
                        ${COMMON_SOURCES})

  add_executable(sim ${SIMULATOR_SOURCES})
  target_link_libraries(sim -lpthread -lmufft)
endif()


