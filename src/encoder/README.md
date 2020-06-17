# LDPC encoder for AVX2

Intel's FlexRAN requires AVX-512 for LDPC encoding. The code in this directory
provides an LDPC encoder for AVX2 machines, based on FlexRAN's code. The
algorithm is based on *Efficient QC-LDPC Encoder for 5G New Radio* by Tram Thi
Bao Nguyen, Tuy Nguyen Tan, Hanho Lee.

## Compilation

`./compile_encoder.sh`

## Note

We might further accelerate encoding by using threading on the XOR tree,
described in pages 11 and 12 of the paper. The XOR tree operates on entries of
each row in the base matrix. One way to add parallelism to the existing frame
work is to let each information column in the base matrix take one thread,
perform cyclic shifts on the corresonding z-bit message segment as defined by
the standard (each entry of the column decides the shift value), store all the
shifted messages, and then perform XOR on all the messages that's shifted by
values on the same row of the base matrix.
