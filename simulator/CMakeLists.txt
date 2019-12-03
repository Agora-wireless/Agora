cmake_minimum_required (VERSION 2.8)
cmake_policy(SET CMP0054 NEW)
project (CoMP)

# Enable C++11
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")
    set (CMAKE_CXX_STANDARD 14)
else ()
    set (CMAKE_CXX_STANDARD 11)
endif()

message(STATUS "CURRENT DIRECTORY: ${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DPROJECT_DIRECTORY=${CMAKE_CURRENT_SOURCE_DIR}/..)

set(CMAKE_C_FLAGS "-std=c99 -w -g -mavx2 -march=broadwell -O3")
set(CMAKE_CXX_FLAGS "-std=c++11 -w -O3 -mavx2 -mavx -g -march=broadwell")

find_package(Armadillo)

include_directories(../src/common/)

set(COMMON_SOURCES ../src/common/config.cpp
                   ../src/common/utils.cpp
                   ../src/common/comms-lib.cpp
                   ../src/common/signalHandler.cpp
                   ../src/common/modulation.cpp
                   ../src/common/net.cpp)


set(SENDER_SOURCES sender_cli.cpp
                   sender.cpp 
                   ${COMMON_SOURCES})

add_executable(sender ${SENDER_SOURCES})
target_link_libraries(sender -lpthread -lmufft)


set(SIMULATOR_SOURCES main.cpp
                      simulator.cpp
                      sender.cpp
                      receiver.cpp 
                      ${COMMON_SOURCES})

add_executable(sim ${SIMULATOR_SOURCES})
target_link_libraries(sim -lpthread -lmufft)