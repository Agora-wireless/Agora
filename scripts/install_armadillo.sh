#!/bin/bash
# Install Armadillo from source
wget http://sourceforge.net/projects/arma/files/armadillo-11.2.3.tar.xz .
tar -xf armadillo-11.2.3.tar.xz
(cd armadillo-11.2.3; cmake -DALLOW_OPENBLAS_MACOS=ON .; make -j4; sudo make install)
rm -rf armadillo*
sudo ldconfig
cd ../