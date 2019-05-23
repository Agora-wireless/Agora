cmake_minimum_required (VERSION 2.8)
cmake_policy(SET CMP0054 NEW)
project (CoMP)

# Enable C++11
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")
    set (CMAKE_CXX_STANDARD 14)
else ()
    set (CMAKE_CXX_STANDARD 11)
endif()


set(CMAKE_C_FLAGS "-std=c99 -w -g -mavx2 -march=broadwell -O3")
set(CMAKE_CXX_FLAGS "-std=c++11 -w -O3 -mavx2 -mavx -g -march=broadwell")

find_package(Armadillo)

# Link with the "Threads library (required to link with AFF3CT after)
set(CMAKE_THREAD_PREFER_PTHREAD ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

set (AFF3CT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake-config/")
find_package (AFF3CT CONFIG REQUIRED)
message(STATUS "AFF3CT_DIR: ${AFF3CT_DIR}")



# Add the compile option from AFF3CT
add_compile_options(${AFF3CT_COMPILE_OPTIONS})

# Add the definitions of AFF3CT
add_definitions(${AFF3CT_DEFINITIONS})

# Add the include directories from AFF3CT
include_directories(${AFF3CT_INCLUDE_DIRS})


include_directories("/opt/intel/compilers_and_libraries_2019.0.117/linux/mkl/include/")
link_directories("/opt/intel/compilers_and_libraries_2019.0.117/linux/mkl/lib/intel64/")

add_executable(sender sender_cli.cpp packageSender.cpp cpu_attach.cpp)
target_link_libraries(sender -lpthread )

#add_executable(receiver receiver_cli.cpp packageReceiver.cpp cpu_attach.cpp)
#target_link_libraries(receiver -lpthread )

#link_directories("/usr/local/lib/hpctoolkit")
#add_executable(bench_overhead main.cpp packageReceiver.cpp packageSenderBS.cpp bench_overhead.cpp cpu_attach.cpp)
#target_link_libraries(bench_overhead -lpthread /usr/local/lib/libmufft.a -larmadillo aff3ct::aff3ct-static-lib)

#add_executable(bench_data main.cpp packageReceiver.cpp packageSenderBS.cpp bench_data.cpp cpu_attach.cpp)
#target_link_libraries(bench_data -lpthread /usr/local/lib/libmufft.a -larmadillo aff3ct::aff3ct-static-lib)

add_executable(CoMP main.cpp packageReceiver.cpp CoMP.cpp cpu_attach.cpp)
target_link_libraries(CoMP -lpthread /usr/local/lib/libmufft.a -larmadillo -lmkl_rt aff3ct::aff3ct-static-lib -lmkl_intel_ilp64 -lmkl_sequential -lmkl_core -lm -ldl)


add_executable(trans testTranspose.cpp)
target_link_libraries(trans -larmadillo) 

#add_library(sounderUL MODULE main.cpp packageReceiver.cpp packageSenderBS.cpp CoMP.cpp cpu_attach.cpp)
#target_link_libraries(sounderUL -lpthread  -lmufft -larmadillo ${AFF3CT_LIBRARIES}) 


