#!/bin/bash
# Install Armadillo from source
wget http://sourceforge.net/projects/arma/files/armadillo-9.900.4.tar.xz .
tar xf armadillo-9.900.4.tar.xz
(cd armadillo-9.900.4; cmake .; make -j4; sudo make install)
rm -rf armadillo*
