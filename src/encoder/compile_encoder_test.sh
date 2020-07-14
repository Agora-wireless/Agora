g++ -std=c++11 -mavx2 -Wall -I. -I/opt/FlexRAN-FEC-SDK-19-04/sdk/source/phy/lib_ldpc_encoder_5gnr -I/opt/FlexRAN-FEC-SDK-19-04/sdk/source/phy/lib_common encoder_test.cpp encoder.cpp cyclic_shift.cpp iobuffer.cpp -o test
#icpc -std=c++11 -mavx2 -Wall -I. encoder_test.cpp encoder.cpp cyclic_shift.cpp iobuffer.cpp -o test
