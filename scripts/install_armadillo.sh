#!/bin/bash
# Install Armadillo from source
wget http://sourceforge.net/projects/arma/files/armadillo-9.300.2.tar.xz .
tar xf armadillo-9.300.2.tar.xz
(cd armadillo-9.300.2; cmake .; make -j4; sudo make install)
rm -rf armadillo*
