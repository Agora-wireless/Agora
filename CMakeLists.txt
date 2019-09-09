cmake_minimum_required (VERSION 2.8)
cmake_policy(SET CMP0054 NEW)
project (Millipede)

# Enable C++11
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")
    set (CMAKE_CXX_STANDARD 14)
else ()
    set (CMAKE_CXX_STANDARD 11)
endif()


set(CMAKE_C_FLAGS "-std=c99 -w -g -O3 -mavx2 -march=broadwell")
set(CMAKE_CXX_FLAGS "-std=c++11 -w -g -O3 -mavx2 -mavx -march=broadwell")

message(STATUS "CURRENT DIRECTORY: ${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DPROJECT_DIRECTORY=${CMAKE_CURRENT_SOURCE_DIR})

find_package(Armadillo)

set(USE_DPDK False)
set(USE_ARGOS False)


# DPDK
if (${USE_DPDK})
  find_package(dpdk)
  if (NOT dpdk_FOUND)
    message(STATUS "DPDK Not Found! Excluding from Compile List")
    set(USE_DPDK False)
  else ()
    add_definitions(-DUSE_DPDK)
  endif()
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
  set (CLIENT_SOURCES
         src/millipede/compute_common.cpp
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
	)

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

set(COMMON_SOURCES src/common/cpu_attach.cpp 
                   src/common/config.cpp
                   src/common/utils.cpp
                   src/common/comms-lib.cpp
                   src/common/signalHandler.cpp)


set(SOURCES src/millipede/main.cpp 
            src/millipede/packageReceiver.cpp 
            src/millipede/millipede.cpp 
            src/millipede/compute_common.cpp  
            src/millipede/stats.cpp 
            src/millipede/dofft.cpp 
            src/millipede/dozf.cpp 
            src/millipede/dodemul.cpp 
            src/millipede/doprecode.cpp
            ${COMMON_SOURCES}
            ${ARGOS_SOURCES})
			
add_executable(millipede ${SOURCES})
target_link_libraries(millipede -lpthread /usr/local/lib/libmufft.a -larmadillo -lmkl_rt -lmkl_intel_ilp64 -lmkl_sequential -lmkl_core -lm -ldl ${DPDK_LIB} ${SOAPY_LIB} ${PYTHON_LIB})


add_library(comp MODULE 
       src/millipede/packageReceiver.cpp
       src/millipede/millipede.cpp
       src/millipede/compute_common.cpp
       src/millipede/stats.cpp 
       src/millipede/dofft.cpp
       src/millipede/dozf.cpp
       src/millipede/dodemul.cpp
       src/millipede/doprecode.cpp
       ${COMMON_SOURCES}
       ${ARGOS_SOURCES})
target_link_libraries(comp -lpthread -lmufft -larmadillo -lmkl_rt -lmkl_intel_ilp64 -lmkl_sequential -lmkl_core -lm -ldl ${DPDK_LIB} ${SOAPY_LIB} ${PYTHON_LIB})


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
                     simulator/packageSender.cpp 
                     ${COMMON_SOURCES})

  add_executable(sender ${SENDER_SOURCES})
  target_link_libraries(sender -lpthread -lmufft)


  set(SIMULATOR_SOURCES simulator/main.cpp
                        simulator/simulator.cpp
                        simulator/packageSender.cpp
                        simulator/packageReceiver.cpp 
                        ${COMMON_SOURCES})

  add_executable(sim ${SIMULATOR_SOURCES})
  target_link_libraries(sim -lpthread -lmufft)
endif()


