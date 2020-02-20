cmake_minimum_required (VERSION 2.8)
cmake_policy(SET CMP0054 NEW)
project (Test)

if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7.0)
  set(GCC_COVERAGE_COMPILE_FLAGS "-faligned-new")
endif()

message(STATUS "CMAKE_C_COMPILER: ${CMAKE_C_COMPILER}")
message(STATUS "CMAKE_CXX_COMPILER: ${CMAKE_C_COMPILER}")

if(${CMAKE_C_COMPILER_ID} STREQUAL "GNU")
  set(CMAKE_C_FLAGS "-std=c99 -Wall -g -mavx2 -march=broadwell -m64")
  set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -mavx2 -mavx -march=broadwell  -m64")
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
  set(MKL_LIBS -Wl,--no-as-needed -lmkl_intel_lp64 -lmkl_sequential -lmkl_core -lpthread -lm -ldl)
elseif(${CMAKE_C_COMPILER_ID} STREQUAL "Intel")
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/intel-compile-options.cmake)
  set(MKL_LIBS -lpthread -lm -ldl)
  set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -mavx2 -mavx -march=broadwell -mkl=sequential")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")
  set(CMAKE_CXX_STANDARD 14)
else()
  set(CMAKE_CXX_STANDARD 11)
endif()


message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")


include_directories(
  ../src/common/)

message(STATUS "CURRENT DIRECTORY: ${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DPROJECT_DIRECTORY=${CMAKE_CURRENT_SOURCE_DIR}/..)
set(COMMON_SOURCES 
  ../src/common/utils.cpp
  ../src/common/modulation.cpp)


get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

set(SOURCES test_modulation.cpp)
            
add_executable(test_modulate ${SOURCES} ${COMMON_SOURCES})
target_link_libraries(test_modulate -lpthread)