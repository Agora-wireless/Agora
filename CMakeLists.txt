cmake_minimum_required (VERSION 2.8)
cmake_policy(SET CMP0054 NEW)
project (CoMP)

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

if (${USE_DPDK})
  find_package(dpdk)
  if (NOT dpdk_FOUND)
    message(STATUS "DPDK Not Found! Excluding from Compile List")
    set(USE_DPDK False)
  else ()
    add_definitions(-DUSE_DPDK)
  endif()
endif()

if (${USE_ARGOS})
  find_package(SoapySDR "0.7" CONFIG)
  if (NOT SoapySDR_FOUND)
    message(STATUS "SoapySDR development files not found")
    set(USE_ARGOS False)
  else ()
    add_definitions(-DUSE_ARGOS)
  endif ()
endif ()

# Link with the "Threads library (required to link with AFF3CT after)
set(CMAKE_THREAD_PREFER_PTHREAD ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# set (AFF3CT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake-config/")
# find_package (AFF3CT CONFIG REQUIRED)
# message(STATUS "AFF3CT_DIR: ${AFF3CT_DIR}")



# Add the compile option from AFF3CT
# add_compile_options(${AFF3CT_COMPILE_OPTIONS})

# Add the definitions of AFF3CT
# add_definitions(${AFF3CT_DEFINITIONS})

# Add the include directories from AFF3CT
# include_directories(${AFF3CT_INCLUDE_DIRS})

# Intel MKL
# include_directories("/opt/intel/compilers_and_libraries_2019.1.144/linux/mkl/include/")
# link_directories("/opt/intel/compilers_and_libraries_2019.1.144/linux/mkl/lib/intel64/")
# include_directories("/opt/intel/compilers_and_libraries_2019.0.117/linux/mkl/include/")
# link_directories("/opt/intel/compilers_and_libraries_2019.0.117/linux/mkl/lib/intel64/")
find_path(MKL_INCLUDE_DIR 
          NAMES mkl_dfti.h
          PATHS /opt/intel/*/linux/mkl/)
set (MKL_LINK_DIR ${MKL_INCLUDE_DIR}/../lib/intel64)
message(STATUS "MKL_INCLUDE_DIR: ${MKL_INCLUDE_DIR}")
message(STATUS "MKL_LINK_DIR: ${MKL_LINK_DIR}")
include_directories(${MKL_INCLUDE_DIR})
link_directories(${MKL_LINK_DIR})




# DPDK
if (${USE_DPDK})
  set (RTE_SDK /home/argos/Jian//dpdk-stable-18.11.1/)
  set (RTE_TARGET x86_64-native-linuxapp-gcc)
  set ( DPDK_LIB  "-Wl,--whole-archive -ldpdk -lnuma -ldl -Wl,--no-whole-archive" )
  include_directories(${RTE_SDK}/${RTE_TARGET}/include)
  link_directories(${RTE_SDK}/${RTE_TARGET}/lib)
endif()

if (${USE_ARGOS})
  message(STATUS "CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")
  #find_package(PythonLibs REQUIRED)
  #message(STATUS "PYTHON_LIBRARIES: ${PYTHON_LIBRARIES}")
  message(STATUS "SoapySDR_INCLUDE_DIRS: ${SoapySDR_INCLUDE_DIRS}")
  message(STATUS "SoapySDR_LIBRARIES: ${SoapySDR_LIBRARIES}")
  include_directories(${PYTHON_INCLUDE_DIRS} ${SoapySDR_INCLUDE_DIRS})
  set (SOAPY_LIB ${SoapySDR_LIBRARIES})
  set (ARGOS_SOURCES src/receiver/radio_lib.cpp )
  set (PYTHON_LIB ${PYTHON_LIBRARIES}) 
  add_definitions(-DTHREADED_INIT)
endif()

include_directories(
	src/common/
	)

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

set(SENDER_SOURCES src/sender/sender_cli.cpp
                   src/sender/packageSender.cpp 
                   src/common/cpu_attach.cpp 
                   src/common/config.cpp
                   src/common/utils.cpp
                   src/common/comms-lib.cpp
                   src/common/signalHandler.cpp)

add_executable(sender ${SENDER_SOURCES})
target_link_libraries(sender -lpthread -lmufft)

set(SOURCES src/receiver/main.cpp 
			src/receiver/packageReceiver.cpp 
			src/receiver/millipede.cpp 
			src/common/cpu_attach.cpp 
			src/receiver/compute_common.cpp  
			src/receiver/stats.cpp 
			src/receiver/dofft.cpp 
			src/receiver/dozf.cpp 
			src/receiver/dodemul.cpp 
			src/receiver/doprecode.cpp
			src/common/config.cpp
			src/common/utils.cpp
			src/common/comms-lib.cpp
			src/common/signalHandler.cpp
       			${ARGOS_SOURCES})
			
add_executable(millipede ${SOURCES})
target_link_libraries(millipede -lpthread /usr/local/lib/libmufft.a -larmadillo -lmkl_rt -lmkl_intel_ilp64 -lmkl_sequential -lmkl_core -lm -ldl ${DPDK_LIB} ${SOAPY_LIB} ${PYTHON_LIB})

add_library(comp MODULE 
       src/receiver/main.cpp
       src/receiver/packageReceiver.cpp
       src/receiver/millipede.cpp
       src/common/cpu_attach.cpp
       src/receiver/compute_common.cpp
	     src/receiver/stats.cpp 
       src/receiver/dofft.cpp
       src/receiver/dozf.cpp
       src/receiver/dodemul.cpp
       src/receiver/doprecode.cpp
       src/common/config.cpp
       src/common/utils.cpp
       src/common/comms-lib.cpp
       src/common/signalHandler.cpp 
       ${ARGOS_SOURCES})

target_link_libraries(comp -lpthread -lmufft -larmadillo -lmkl_rt -lmkl_intel_ilp64 -lmkl_sequential -lmkl_core -lm -ldl ${DPDK_LIB} ${SOAPY_LIB} ${PYTHON_LIB})

#add_executable(receiver receiver_cli.cpp packageReceiver.cpp cpu_attach.cpp)
#target_link_libraries(receiver -lpthread )

#link_directories("/usr/local/lib/hpctoolkit")
#add_executable(bench_overhead main.cpp packageReceiver.cpp packageSenderBS.cpp bench_overhead.cpp cpu_attach.cpp)
#target_link_libraries(bench_overhead -lpthread /usr/local/lib/libmufft.a -larmadillo aff3ct::aff3ct-static-lib)

#add_executable(bench_data main.cpp packageReceiver.cpp packageSenderBS.cpp bench_data.cpp cpu_attach.cpp)
#target_link_libraries(bench_data -lpthread /usr/local/lib/libmufft.a -larmadillo aff3ct::aff3ct-static-lib)

#add_executable(CoMP main.cpp packageReceiver.cpp CoMP.cpp cpu_attach.cpp)
#target_link_libraries(CoMP -lpthread /usr/local/lib/libmufft.a -larmadillo -lmkl_rt aff3ct::aff3ct-static-lib -lmkl_intel_ilp64 -lmkl_sequential -lmkl_core -lm -ldl ${DPDK_LIB})

#add_executable(trans testTranspose.cpp)
#target_link_libraries(trans -larmadillo) 

#add_library(sounderUL MODULE main.cpp packageReceiver.cpp packageSenderBS.cpp CoMP.cpp cpu_attach.cpp)
#target_link_libraries(sounderUL -lpthread  -lmufft -larmadillo ${AFF3CT_LIBRARIES}) 
