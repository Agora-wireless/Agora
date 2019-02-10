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

set (AFF3CT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake-config/")
find_package (AFF3CT CONFIG REQUIRED)
#message(STATUS "AFF3CT_DIR: ${AFF3CT_DIR}")


if (AFF3CT_FOUND)
    if (NOT AFF3CT_SYSTEMC_MODULE AND (AFF3CT_PREC STREQUAL "MULTI" OR AFF3CT_PREC STREQUAL "32"))
        message(STATUS "AFF3CT - Library found")

        # Add the compile option from AFF3CT
        add_compile_options(${AFF3CT_COMPILE_OPTIONS})

        # Add the definitions of AFF3CT
        add_definitions(${AFF3CT_DEFINITIONS})

        # Add the include directories from AFF3CT
        include_directories(${AFF3CT_INCLUDE_DIRS})

		add_executable(sender sender_cli.cpp packageSender.cpp cpu_attach.cpp)
		target_link_libraries(sender -lpthread )

		#add_executable(receiver receiver_cli.cpp packageReceiver.cpp cpu_attach.cpp)
		#target_link_libraries(receiver -lpthread )


		add_executable(CoMP main.cpp packageReceiver.cpp packageSenderBS.cpp CoMP.cpp cpu_attach.cpp)
		target_link_libraries(CoMP -lpthread /usr/local/lib/libmufft.a -larmadillo ${AFF3CT_LIBRARIES})


		add_executable(trans testTranspose.cpp)
		target_link_libraries(trans -larmadillo) 

		#add_library(sounderUL MODULE main.cpp packageReceiver.cpp packageSenderBS.cpp CoMP.cpp cpu_attach.cpp)
		#target_link_libraries(sounderUL -lpthread  -lmufft -larmadillo ${AFF3CT_LIBRARIES}) 


else()
        message(FATAL_ERROR "Incompatible AFF3CT library build.")
    endif()
endif (AFF3CT_FOUND)