#!/bin/bash
echo "Matrix_size Formula SVD SVD/Formula"
for n_rows in `seq 8 8 64`; do
  for n_cols in `seq 8 8 ${n_rows}`; do 
    ./bench --n_rows ${n_rows} --n_cols ${n_cols} --n_iters 1000 2>/dev/null
  done
done
