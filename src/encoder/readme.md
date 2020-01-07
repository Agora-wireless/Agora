#ldpc encoder
an AVX256 implementation based on Intel's AVX512 library </br>

algorithm based on *Efficient QC-LDPC Encoder for 5G New Radio* by Tram Thi Bao Nguyen, Tuy Nguyen Tan, Hanho Lee </br>

to compile, run
`icc encoder_test.cpp encoder.cpp cyclic_shift.cpp iobuffer.cpp -o test`

note </br>
to further accerlerate the encoding process, threading could be used on the XOR tree described in pg 11 & 12 in the paper. the XOR tree operates on entries of each row in the base matrix. one way to add parallelism to the existing frame work is to let each information column in the base matrix take one thread, perform cyclic shifts on the corresonding z-bit message segment as defined by the standard (each entry of the column decides the shift value), store all the shifted messages, and then perform XOR on all the messages that's shifted by values on the same row of the base matrix.
