#!/bin/bash
# Install Armadillo from source
# --- Armadillo (12.2.0)
wget http://sourceforge.net/projects/arma/files/armadillo-12.2.0.tar.xz
tar -xf armadillo-12.2.0.tar.xz
cd armadillo-12.2.0
cmake -DALLOW_OPENBLAS_MACOS=ON .
make -j`nproc`
sudo make install
sudo ldconfig
cd ../
