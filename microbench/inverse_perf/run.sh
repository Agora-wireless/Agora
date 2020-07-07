#!/bin/bash
echo "Matrix_size Formula_us SVD_us SVD/Formula"
for n_rows in 64; do
  for n_cols in 8 16 24 32 40 48 56; do 
    numactl --physcpubind=0 --membind=0 ./bench --n_rows ${n_rows} --n_cols ${n_cols} --n_iters 10000  2>/dev/null
  done
done
