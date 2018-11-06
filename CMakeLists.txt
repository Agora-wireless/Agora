cmake_minimum_required (VERSION 2.8)
project (CoMP)



set(CMAKE_C_FLAGS "-std=c99 -w -g -march=broadwell -O3")
set(CMAKE_CXX_FLAGS "-std=c++11 -w -O3 -mavx2 -mavx -g -march=broadwell")

find_package(Armadillo)

message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
add_executable(sender sender_cli.cpp packageSender.cpp cpu_attach.cpp)
target_link_libraries(sender -lpthread )

add_executable(receiver receiver_cli.cpp packageReceiver.cpp cpu_attach.cpp)
target_link_libraries(receiver -lpthread )


add_executable(CoMP main.cpp packageReceiver.cpp packageSenderBS.cpp CoMP.cpp cpu_attach.cpp)
target_link_libraries(CoMP -lpthread /usr/local/lib/libmufft.a -larmadillo)


add_executable(trans testTranspose.cpp)
target_link_libraries(trans -larmadillo) 

add_library(sounderUL MODULE main.cpp packageReceiver.cpp packageSenderBS.cpp CoMP.cpp cpu_attach.cpp)
target_link_libraries(sounderUL -lpthread  -lmufft -larmadillo) 