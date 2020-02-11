#!/usr/bin/env bash
#see https://github.com/esa/pagmo_plugins_nonfree/issues/2 for why a specific commit is needed
set -e

git clone https://github.com/snopt/snopt-interface.git &&
cd snopt-interface && git checkout 76b166ecdf5c55a3289ce0f849d8d3d101954a22
autoconf && ./configure --prefix=/usr/local &&
make -j8 && sudo make install
